#include "curves_deformer_factory.h"
#include "base_curves_deformer.h"
#include "geometry_tools.h"
#include "pxr_points_lru_cache.h"

#include <thread>


namespace Piston {

BaseCurvesDeformer::BaseCurvesDeformer(const BaseCurvesDeformer::Type t, const std::string& name): mPool(std::thread::hardware_concurrency() - 1), mType(t), mName(name) {
	dbg_printf("BaseCurvesDeformer::BaseCurvesDeformer()\n");

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

	auto func = [this](bool multi_threaded, PxrCurvesContainer* pCurves, pxr::UsdTimeCode time_code) {
		switch(multi_threaded) {
			case true:
				return deformMtImpl(pCurves, time_code);
			default:
				return deformImpl(pCurves, time_code);
		}
	};

	const PxrPointsLRUCache::CompositeKey curr_key = {mName, time_code};
	PxrPointsLRUCache* pPointsLRUCache = CurvesDeformerFactory::getInstance().getPxrPointsLRUCachePtr();
	const std::vector<pxr::GfVec3f>* deformed_points_list_ptr = pPointsLRUCache->get(curr_key);
	if(!deformed_points_list_ptr) {
		if (!func(multi_threaded, mpCurvesContainer.get(), time_code)) {
			return false;
		}
		//deformed_points_list_ptr = pPointsLRUCache->put(curr_key, mpCurvesContainer->getPointsCache());
		deformed_points_list_ptr = mpCurvesContainer->getPointsCachePtr();
	}

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	//if(!curves.GetPointsAttr().Set(mpCurvesContainer->getPointsCacheVtArray(), time_code)) {
	//	return false;
	//}

	pxr::VtArray<pxr::GfVec3f> _points_tmp_vt_array = {&mForeignDataSource, (pxr::GfVec3f*)deformed_points_list_ptr->data(), deformed_points_list_ptr->size(), false};
	if(!curves.GetPointsAttr().Set(_points_tmp_vt_array, time_code)) {
		return false;
	}

/*
	if(mCalcMotionVectors) {

		std::cout << "TimeCode: " << std::to_string(time_code.GetValue()) << std::endl;
		std::cout << "Mesh prim stage FPS: " << std::to_string(mMeshGeoPrimHandle.getStageFPS()) << std::endl;

		Points* pPointsFrom = isCenteredMotionBlur() ? nullptr : ;
		Points* pPointsTo = isCenteredMotionBlur() ? nullptr : ;

		PxrPointsLRUCache* pPointsLRUCache = CurvesDeformerFactory::getInstance().getPxrPointsLRUCachePtr();

		if(pPointsLRUCache) {
			Points* pPointsFrom = isCenteredMotionBlur() ? ;
			Points* pPointsTo = isCenteredMotionBlur() ? ;

			pxr::UsdTimeCode t_from = time_code - 1.0;
			copnst PxrPointsLRUCache::CompositeKey key_from = {mName, t_from}
			Points* pPoints = pPointsLRUCache->get(key_from);
			if(!pPoints) {
				// There is no cached points data. Cache current values and calculate one needed

				pPointsLRUCache->put(curr_key, mpCurvesContainer->getPointsCache());

				if(!func(multi_threaded, mpCurvesContainer.get(), t_from)) {
					std::cerr << "Error computing deformation for motion vectors at " << std::to_string(t_from.GetValue()); << std::endl;
					return false;
				}
			} else {
				dbg_printf("We ve got cached points data !!!\n");
			}
		}

		mVelocities.resize(mpCurvesContainer->getPointsCache().size());
	}
*/

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
