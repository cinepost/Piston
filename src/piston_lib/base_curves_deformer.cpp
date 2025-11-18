#include "curves_deformer_factory.h"
#include "base_curves_deformer.h"
#include "geometry_tools.h"
#include "pxr_points_lru_cache.h"

#include <thread>


namespace Piston {

BaseCurvesDeformer::BaseCurvesDeformer(const BaseCurvesDeformer::Type t, const std::string& name): mPool(std::thread::hardware_concurrency() - 1), mType(t), mName(name), mID(current_id++) {
	dbg_printf("BaseCurvesDeformer::BaseCurvesDeformer()\n");

	mUniqueName = toString() + mName + std::to_string(mID);

	makeDirty();
	mpTempStage = pxr::UsdStage::CreateInMemory();
}

void BaseCurvesDeformer::setMeshGeoPrim(const pxr::UsdPrim& geoPrim) {
	//assert(pGeoPrim);
	if(mMeshGeoPrimHandle == geoPrim) return;
	if(!isMeshGeoPrim(geoPrim)) {
		mMeshGeoPrimHandle.clear();
		std::cerr << "Mesh geometry prim is not \"Mesh\"!" << std::endl;
		return;
	}

	mMeshGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	dbg_printf("Mesh geometry prim is set to: %s\n", mMeshGeoPrimHandle.getPath().GetText());
}

void BaseCurvesDeformer::setCurvesGeoPrim(const pxr::UsdPrim& geoPrim) {
	if(mCurvesGeoPrimHandle == geoPrim) return;
	if(!isCurvesGeoPrim(geoPrim)) {
		mCurvesGeoPrimHandle.clear();
		std::cerr << "Curves geometry prim is not \"BasisCurves\"!" << std::endl;
		return;
	}

	mCurvesGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	dbg_printf("Curves geometry prim is set to: %s\n", mCurvesGeoPrimHandle.getPath().GetText());
}

void BaseCurvesDeformer::setReadJsonDataFromPrim(bool state) {
	if(mReadJsonDeformerData == state) return;
	mDirty = true;
	mReadJsonDeformerData = state;
}

bool BaseCurvesDeformer::writeJsonDataToPrim(pxr::UsdTimeCode time_code) {
	// Write json data if needed
	if(!buildDeformerData(time_code)) {
		std::cerr << "Error building deformer data !" << std::endl;
		return false;
	}

	if(!mMeshGeoPrimHandle.writeDataToBson(mpAdjacencyData.get())) {
		std::cerr << "Error writing " << mpAdjacencyData->typeName() << " deformer mesh data to json !" << std::endl;
		return false;
	}

	if(!mCurvesGeoPrimHandle.writeDataToBson(mpPhantomTrimeshData.get())) {
		std::cerr << "Error writing " << mpPhantomTrimeshData->typeName() << " deformer curves data to json !" << std::endl;
		return false;
	}

	return writeJsonDataToPrimImpl();
}

bool BaseCurvesDeformer::buildDeformerData(pxr::UsdTimeCode reference_time_code) {
	if(!mDirty) return true;

	SimpleProfiler::clear();

	if(!mMeshGeoPrimHandle || !mCurvesGeoPrimHandle) {
		std::cerr << "No mesh or curves UsdPrim is set !" << std::endl;
		return false;
	}

	if(!mpAdjacencyData) {
		mpAdjacencyData = std::make_unique<SerializableUsdGeomMeshFaceAdjacency>();
	}

	// Get primitive adjacency json data if present
	if(!mReadJsonDeformerData || !mMeshGeoPrimHandle.getDataFromBson(mpAdjacencyData.get())) {
		// Build in place if no json data present or not needed
		if(!mpAdjacencyData->buildInPlace(mMeshGeoPrimHandle)) {
			std::cerr << "Error building mesh adjacency data!" << std::endl;
			return false;
		}
	}

	if(!mpPhantomTrimeshData) {
		mpPhantomTrimeshData = std::make_unique<SerializablePhantomTrimesh>();
	}

	// Get phantom mesh json data if present
	if(!mReadJsonDeformerData || !mCurvesGeoPrimHandle.getDataFromBson(mpPhantomTrimeshData.get())) {
		// Build in place if no json data present or not needed
		if(!mpPhantomTrimeshData->buildInPlace(mMeshGeoPrimHandle, getMeshRestPositionAttrName())) {
			std::cerr << "Error building phantom mesh data!" << std::endl;
			return false;
		}
	}
	
	mpCurvesContainer = PxrCurvesContainer::create(mCurvesGeoPrimHandle, reference_time_code);
	if(!mpCurvesContainer) {
		std::cerr << "Error creating curves container !" << std::endl;
		return false;
	}

	if(!buildDeformerDataImpl(reference_time_code)) {
		std::cerr << "Error building deform data !" << std::endl;
		return false;
	}

	mDirty = false;
	return true;
}

bool BaseCurvesDeformer::deform_dbg(pxr::UsdTimeCode time_code) {	
	return deform(time_code);
}

bool BaseCurvesDeformer::deform(pxr::UsdTimeCode time_code, bool multi_threaded) {	
	if(!buildDeformerData(time_code)) {
		return false;
	}

	auto deformPoints = [this](bool multi_threaded, PxrCurvesContainer* pCurves, pxr::UsdTimeCode time_code) {
		printf("lambda deformPoints\n");
		switch(multi_threaded) {
			case true:
				return deformMtImpl(pCurves, time_code);
			default:
				return deformImpl(pCurves, time_code);
		}
	};

	auto getDeformedPoints = [&deformPoints](bool multi_threaded, PxrCurvesContainer* pCurves, PxrPointsLRUCache* pPointsLRUCache, const PxrPointsLRUCache::CompositeKey& key) {
		printf("lambda getDeformedPoints\n");
		static const std::vector<pxr::GfVec3f>* sNull = nullptr;
		const std::vector<pxr::GfVec3f>* _points_list_ptr = pPointsLRUCache ? pPointsLRUCache->get(key) : nullptr;
		if(!_points_list_ptr) {
			if (!deformPoints(multi_threaded, pCurves, key.time)) {
				return sNull;
			}
			_points_list_ptr = pPointsLRUCache ? pPointsLRUCache->put(key, pCurves->getPointsCache()) : pCurves->getPointsCachePtr();
		}
		return _points_list_ptr;
	};

	const PxrPointsLRUCache::CompositeKey curr_key = {uniqueName(), time_code};
	PxrPointsLRUCache* pPointsLRUCache = CurvesDeformerFactory::getInstance().getPxrPointsLRUCachePtr();

	PxrPointsLRUCacheShrinkLock cache_shrink_lock(pPointsLRUCache); // avoid cache shrinking during deformation stage
	printf("pPointsLRUCache locked\n");

	const std::vector<pxr::GfVec3f>* deformed_points_list_ptr = getDeformedPoints(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, curr_key);

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	pxr::VtArray<pxr::GfVec3f> _points_tmp_vt_array = {&mForeignDataSource, (pxr::GfVec3f*)deformed_points_list_ptr->data(), deformed_points_list_ptr->size(), false};
	if(!curves.GetPointsAttr().Set(_points_tmp_vt_array, time_code)) {
		return false;
	}

	if(mCalcMotionVectors) {
		const PxrPointsLRUCache::CompositeKey key_from = {uniqueName(), (mMotionBlurDirection != MotionBlurDirection::LEADING) ? (time_code.GetValue() - 1.0) : time_code};
		const PxrPointsLRUCache::CompositeKey key_to = {uniqueName(), (mMotionBlurDirection != MotionBlurDirection::TRAILING) ? (time_code.GetValue() + 1.0) : time_code};

		dbg_printf("MBlur: %s to %s\n", std::to_string(key_from.time.GetValue()).c_str(), std::to_string(key_to.time.GetValue()).c_str());

		const std::vector<pxr::GfVec3f>* pPointsFrom = (mMotionBlurDirection == MotionBlurDirection::LEADING) ? deformed_points_list_ptr : getDeformedPoints(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, key_from);
		const std::vector<pxr::GfVec3f>* pPointsTo = (mMotionBlurDirection == MotionBlurDirection::TRAILING) ? deformed_points_list_ptr : getDeformedPoints(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, key_to);

		mVelocities.resize(mpCurvesContainer->getPointsCache().size());

		const pxr::GfVec3f* p_pts_from_ptr = pPointsFrom->data();
		const pxr::GfVec3f* p_pts_to_ptr = pPointsTo->data();

		const float k = ((mMotionBlurDirection == MotionBlurDirection::CENTERED) ? .5f : 1.0f) / static_cast<float>(mMeshGeoPrimHandle.getStageTimeCodesPerSecond());

		for(size_t i = 0; i < mVelocities.size(); ++i) {
			mVelocities[i] = (p_pts_to_ptr[i] - p_pts_from_ptr[i]) * k;
		}

		if(pxr::UsdAttribute attr_v = curves.GetVelocitiesAttr()) {
			pxr::VtArray<pxr::GfVec3f> _velocities_tmp_vt_array = {&mVelocitiesForeignDataSource, (pxr::GfVec3f*)mVelocities.data(), mVelocities.size(), false};
			if(!attr_v.Set(_velocities_tmp_vt_array, time_code)) {
				std::cerr << "Error setting velocities attribute !\n";
				return false;
			}
		} else {
			std::cerr << mCurvesGeoPrimHandle.getName() << " has no velocities attribute !\n";
		}
	}

	if(pPointsLRUCache) {
		dbg_printf("Cache utilization %s%\n", pPointsLRUCache->getCacheUtilizationString().c_str());
	}

	return true;
}

void BaseCurvesDeformer::setMeshRestPositionAttrName(const std::string& name) {
	if(mMeshRestPositionAttrName == name) return;
	mMeshRestPositionAttrName = name;
	makeDirty();
}

void BaseCurvesDeformer::setСurvesSkinPrimAttrName(const std::string& name) {
	if(mСurvesSkinPrimAttrName == name) return;
	mСurvesSkinPrimAttrName = name;
	makeDirty();
}

void BaseCurvesDeformer::setVelocityAttrName(const std::string& name) {
	if(mVelocityAttrName == name) return;
	mVelocityAttrName = name;
	makeDirty();
}

void BaseCurvesDeformer::makeDirty() {
	mStats.clear();
	mDirty = true;
}

const std::string& BaseCurvesDeformer::toString() const {
	static const std::string kBaseDeformerString = "BaseCurvesDeformer";
	return kBaseDeformerString;
}

} // namespace Piston

std::atomic_uint32_t Piston::BaseCurvesDeformer::current_id = 0;