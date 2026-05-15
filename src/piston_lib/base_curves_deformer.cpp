#include "deformer_factory.h"
#include "base_curves_deformer.h"
#include "geometry_tools.h"
#include "pxr_points_lru_cache.h"
#include "topology.h"
#include "logging.h"

#include <thread>
#include <atomic>

static std::string gLRUCacheStatsLastUsageStr = "-";

namespace Piston {

BaseCurvesDeformer::BaseCurvesDeformer(const BaseCurvesDeformer::Type t, const std::string& name): 
	mDirty(true), 
	mDeformerDataWritten(false), 
	mPool(std::thread::hardware_concurrency() - 1), 
	mType(t), 
	mName(name), 
	mID(current_id++),
	mRestTimeCode(pxr::UsdTimeCode::Default()) {
	
	DLOG_TRC << "BaseCurvesDeformer::BaseCurvesDeformer()";

	mUniqueName = toString() + mName + std::to_string(mID);
}

void BaseCurvesDeformer::setDataPrimPath(const std::string& path) {
	const pxr::SdfPath new_path(path);
	if(mDataPrimPath == new_path) return;
	if(!new_path.IsPrimPath()) {
		DLOG_ERR << "Unable to set data prim path to \"" << path << "\". Path is not a prim path !";
		return;
	}
	mDataPrimPath = new_path;
	makeDirty();
}

const pxr::SdfPath& BaseCurvesDeformer::getDataPrimPath() const { 
	if(mDataPrimPath.IsPrimPath()) {
		return mDataPrimPath; 
	}

	return CurvesDeformerFactory::getInstance().getDefaultDataPrimPath();
}

void BaseCurvesDeformer::setDeformerGeoPrim(const pxr::UsdPrim& prim) {
	if(!prim.IsValid() || mDeformerGeoPrimHandle == prim) return;
	
	if(!validateDeformerGeoPrim(prim)) {
		mDeformerGeoPrimHandle.clear();
		DLOG_ERR << "Invalid geometry prim " << prim << " type!";
		return;
	}

	if(mCurvesGeoPrimHandle == prim) {
		mDeformerGeoPrimHandle.clear();
		DLOG_ERR << "Can't use the same prim " << mCurvesGeoPrimHandle << " for curves and deformer geometry !!!";
		return;
	}

	auto new_handle = UsdPrimHandle(prim);
	const bool same_topology = mDeformerGeoPrimHandle.isValid() ? isSameTopology(mDeformerGeoPrimHandle, new_handle, getRestTimeCode()) : false;

	mDeformerGeoPrimHandle = std::move(new_handle);
	if(!same_topology) {
		makeDirty();
	}

	DLOG_DBG << "Deformer geometry prim is set to: " << mDeformerGeoPrimHandle;
}

void BaseCurvesDeformer::setDeformerGeoPrim(const BaseCurvesDeformer::SharedPtr& pDeformer) {
	assert(pDeformer);

	const auto& deformer_prim = pDeformer->getOutputPrimHandle().getPrim();

	if(!deformer_prim.IsValid() || mDeformerGeoPrimHandle == deformer_prim) {
		return;
	}

	if(!validateDeformerGeoPrim(deformer_prim)) {
		mDeformerGeoPrimHandle.clear();
		DLOG_ERR << "Invalid geometry prim " << deformer_prim << " type!";
		return;
	}

	if(mCurvesGeoPrimHandle == deformer_prim) {
		mDeformerGeoPrimHandle.clear();
		DLOG_ERR << "Can't use the same prim " << deformer_prim << " for curves and deformer geometry !!!";
		return;
	}

	auto new_handle = UsdPrimHandle(pDeformer);
	const bool same_topology = mDeformerGeoPrimHandle.isValid() ? isSameTopology(mDeformerGeoPrimHandle, new_handle, getRestTimeCode()) : false;

	mDeformerGeoPrimHandle = std::move(new_handle);
	if(!same_topology) {
		makeDirty();
	}

	DLOG_DBG << "Deformer prim is set to " << pDeformer->getName();
}

void BaseCurvesDeformer::setCurvesGeoPrim(const pxr::UsdPrim& prim) {
	if(!prim.IsValid() || mCurvesGeoPrimHandle == prim) return;

	if(!isBasisCurvesGeoPrim(prim)) {
		mCurvesGeoPrimHandle.clear();
		DLOG_ERR << "Curves geometry prim is not \"BasisCurves\"!";
		return;
	}

	if(mDeformerGeoPrimHandle == prim) {
		mCurvesGeoPrimHandle.clear();
		DLOG_ERR << "Can't use the same prim " << mDeformerGeoPrimHandle << " for deformer and curves geometry !!!";
		return;
	}

	auto new_handle = UsdPrimHandle(prim);
	const bool same_topology = mCurvesGeoPrimHandle.isValid() ? isSameTopology(mCurvesGeoPrimHandle, new_handle, getRestTimeCode()) : false;

	mCurvesGeoPrimHandle = std::move(new_handle);
	if(!same_topology) {
		makeDirty();
	}

	DLOG_DBG << "Curves geometry prim is set to: " << mCurvesGeoPrimHandle;
}

const pxr::UsdPrim& BaseCurvesDeformer::getDeformerGeoPrim() const {
	mDeformerGeoPrimHandle.getPrim();
}

const pxr::UsdPrim& BaseCurvesDeformer::getCurvesGeoPrim() const {
	mCurvesGeoPrimHandle.getPrim();
}

void BaseCurvesDeformer::setDeformerRestAttrName(const std::string& name) {
	if(mDeformerRestAttrName == name) return;
	mDeformerRestAttrName = name;
	makeDirty();

	DLOG_DBG << "Geometry rest attribute name is set to: " <<  name;
}

void BaseCurvesDeformer::setCurvesRestAttrName(const std::string& name) {
	if(mCurvesRestAttrName == name) return;
	mCurvesRestAttrName = name;

	mpCurvesContainer = nullptr;
	makeDirty();

	DLOG_DBG << "Curves geometry rest attribute name is set to: " << name;
}

void BaseCurvesDeformer::setReadJsonDataFromPrim(bool state) {
	if(mReadJsonDeformerData == state) return;
	mReadJsonDeformerData = state;
	makeDirty();
}

void BaseCurvesDeformer::setRestTimeCode(pxr::UsdTimeCode time_code) {
	if(getRestTimeCode() == time_code) return;
	mRestTimeCode = time_code;
	makeDirty();
}

pxr::UsdTimeCode BaseCurvesDeformer::getRestTimeCode() const {
	if(mRestTimeCode.IsDefault()) {
		return CurvesDeformerFactory::getDefaultRestTimeCode();
	}

	return mRestTimeCode;
}

bool BaseCurvesDeformer::writeJsonDataToPrim(pxr::UsdTimeCode time_code) {
	if(mDeformerDataWritten) return true;

	mDeformerDataWritten = false;

	if(time_code.IsDefault()) {
		time_code = getRestTimeCode();
	}

	// Write json data if needed
	if(!buildDeformerData(time_code)) {
		DLOG_ERR << "Error building " << mName << " deformer data !";
		return false;
	}

	mDeformerDataWritten = writeJsonDataToPrimImpl();
	return mDeformerDataWritten;
}

bool BaseCurvesDeformer::buildDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!mDirty) return true;

	SimpleProfiler::clear();

	if(!mDeformerGeoPrimHandle) {
		DLOG_ERR << "No deformer UsdPrim is set !!!";
		return false;
	}

	if(!mCurvesGeoPrimHandle) {
		DLOG_ERR << "No curves UsdPrim is set !!!";
		return false;
	}

	if(!mpCurvesContainer) {
		mpCurvesContainer = PxrCurvesContainer::create(mCurvesGeoPrimHandle, getCurvesRestAttrName(), rest_time_code);
		if(!mpCurvesContainer) {
			DLOG_ERR << "Error creating curves container for prim " << mCurvesGeoPrimHandle << " !";
			return false;
		}
	} else {
		if(!mpCurvesContainer->init(mCurvesGeoPrimHandle, getCurvesRestAttrName(), rest_time_code)) {
			DLOG_ERR << "Error initializing curves container for prim " << mCurvesGeoPrimHandle << " !";
			return false;
		}
	}

	if(!mpDeformerMeshContainer) {
		mpDeformerMeshContainer = MeshContainer::create(mDeformerGeoPrimHandle, getDeformerRestAttrName(), rest_time_code);
		if(!mpDeformerMeshContainer) {
			DLOG_ERR << "Error creating deformer mesh container for prim " << mDeformerGeoPrimHandle << " !";
			return false;
		}
	} else {
		if(!mpDeformerMeshContainer->init(mDeformerGeoPrimHandle, getDeformerRestAttrName(), rest_time_code)) {
			DLOG_ERR << "Error initializing curves container for prim " << mDeformerGeoPrimHandle << " !";
			return false;
		}
	}


	{
		const std::string entry_name_str = toString() + ":" + getName() + ":buildDeformerData";
		PROFILE(entry_name_str.c_str());

		if(!buildDeformerDataImpl(rest_time_code, multi_threaded)) {
			DLOG_ERR << "Error building " << mName <<" deformer data !";
			return false;
		}

	}

	mDirty = false;
	return true;
}

void BaseCurvesDeformer::setPointsCacheUsageState(bool state) {
	if(mUsePointsCache == state) return;
	mUsePointsCache = state;

	if(!mUsePointsCache) {
		clearLRUCaches();
	}
}

void BaseCurvesDeformer::setInstancingState(bool state) {
	if(mInstancingEnabled == state) return;
	mInstancingEnabled = state;

	if(mInstancingEnabled && !CurvesDeformerFactory::getDataInstancingState()) {
		DLOG_WRN << "Deformers data instancing is disabled!";
		return;
	}

	makeDirty();
}

bool BaseCurvesDeformer::getInstancingState() const { 
	return CurvesDeformerFactory::getDataInstancingState() && mInstancingEnabled; 
}

bool BaseCurvesDeformer::getPointsCacheUsageState() const {
	return mUsePointsCache && CurvesDeformerFactory::getInstance().getPointsCacheUsageState();
}

bool BaseCurvesDeformer::deform_dbg(pxr::UsdTimeCode time_code, bool ignoreVelocities) {	
	return deform(time_code, false, ignoreVelocities);
}

bool BaseCurvesDeformer::deform(pxr::UsdTimeCode time_code, bool multi_threaded, bool ignoreVelocities) {
	LOG_TRC << "Deform at time code: " << time_code.GetValue();
		
	const pxr::UsdTimeCode bind_time_code = getRestTimeCode();
	if(!buildDeformerData(bind_time_code, multi_threaded)) {
		return false;
	}

	assert(mpDeformerMeshContainer);
	if(!mpDeformerMeshContainer || mpDeformerMeshContainer->getRestPositions().empty()) {
		return false;
	}

	assert(mpCurvesContainer);	
	if(!mpCurvesContainer || mpCurvesContainer->empty()) {
		return false;
	}

	auto deformPoints = [this](bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
		if(!mpDeformerMeshContainer->update(mDeformerGeoPrimHandle, time_code, isDirty())) {
			return false;
		}

		if(!mpCurvesContainer->update(mCurvesGeoPrimHandle, time_code, isDirty())) {
			return false;
		}
//		mpDeformerMeshContainer->makeUnique();
		
		if(multi_threaded) { return deformMtImpl(points, time_code); }

		return deformImpl(points, time_code);
	};

	auto getTempVelocitiesList = [this](size_t list_size) {
		if(!mpTempVelocitiesList) {
			mpTempVelocitiesList = std::make_unique<PointsList>(list_size);
		} else {
			mpTempVelocitiesList->resize(list_size);
		}

		return (PointsList*)mpTempVelocitiesList.get();
	};


	auto getDeformedPoints = [this, &deformPoints](std::unique_ptr<PointsList>& points, bool multi_threaded, PxrCurvesContainer* pCurves, pxr::UsdTimeCode time_code) {
		assert(pCurves);
		
		const size_t points_count = pCurves->getTotalVertexCount();

		if(!points) {
			points = std::make_unique<PointsList>(points_count);
		} else {
			points->resize(points_count);
		}

		PointsList* points_list = points.get(); 

		if (deformPoints(multi_threaded, *points_list, time_code)) {
			return (const PointsList*)points_list;
		}

		return (const PointsList*)nullptr;
	};

	auto getDeformedPointsLRU = [&deformPoints](bool multi_threaded, PxrCurvesContainer* pCurves, PxrPointsLRUCache* pPointsLRUCache, const PxrPointsLRUCache::CompositeKey& key) {
		assert(pCurves);
		assert(pPointsLRUCache);

		static const PointsList* sNull = nullptr;

		const PointsList* p_points_list_ptr = pPointsLRUCache->get(key);
		if(p_points_list_ptr) {
			LOG_TRC << "Cache has entry key " << to_string(key);
			return p_points_list_ptr;
		}

		PointsList* p_new_points_list = pPointsLRUCache->put(key, pCurves->getTotalVertexCount());
		if (deformPoints(multi_threaded, *p_new_points_list, key.time)) {
			return (const PointsList*)p_new_points_list;
		}

		return sNull;
	};

	const PxrPointsLRUCache::CompositeKey curr_key = {uniqueName(), time_code};
	PxrPointsLRUCache* pPointsLRUCache = mUsePointsCache ? CurvesDeformerFactory::getInstance().getPxrPointsLRUCachePtr() : nullptr;

	const PxrPointsLRUCache::CompositeKey key_from = {uniqueName(), (mMotionBlurDirection != MotionBlurDirection::LEADING) ? (time_code.GetValue() - 1.0) : time_code};
	const PxrPointsLRUCache::CompositeKey key_to = {uniqueName(), (mMotionBlurDirection != MotionBlurDirection::TRAILING) ? (time_code.GetValue() + 1.0) : time_code};

	PxrPointsLRUCacheShrinkLock cache_shrink_lock(pPointsLRUCache); // avoid cache shrinking during deformation stage
	if(cache_shrink_lock.isValid()) DLOG_TRC << "pPointsLRUCache locked";

	LOG_TRC << "Deform points at " << time_code;
	const PointsList* deformed_points_list_ptr = pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, curr_key) : getDeformedPoints(mpDeformedPointsList, multi_threaded, mpCurvesContainer.get(), time_code);

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());

	LOG_TRC << "Velocities calculation is possible " << (mDeformerGeoPrimHandle.hasPositionsTimeSamples(key_from.time, key_to.time) ? "YES" : "NO");
	LOG_TRC << "Velocities calculation is ignored " << (ignoreVelocities ? "YES" : "NO");

	if(!ignoreVelocities && mCalcMotionVectors && mDeformerGeoPrimHandle.hasPositionsTimeSamples(key_from.time, key_to.time)) {

		const PxrPointsLRUCache::CompositeKey key_vel = {velocityKeyName(), time_code};
		const PointsList* veolcities_list_ptr = pPointsLRUCache ? pPointsLRUCache->get(key_vel) : nullptr;

		if(!veolcities_list_ptr) {
			PointsList* tmp_velicities_list_ptr = pPointsLRUCache ? pPointsLRUCache->put(key_vel, mpCurvesContainer->getTotalVertexCount()) : getTempVelocitiesList(mpCurvesContainer->getTotalVertexCount());

			LOG_TRC << "Calc velocities from " <<  std::to_string(key_from.time.GetValue()) << " to " <<  std::to_string(key_to.time.GetValue());

			const PointsList* pPointsFrom = (mMotionBlurDirection == MotionBlurDirection::LEADING) ? deformed_points_list_ptr : 
				(pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, key_from) : getDeformedPoints(mpDeformedPointsListStep, multi_threaded, mpCurvesContainer.get(), key_from.time));
			
			const PointsList* pPointsTo = (mMotionBlurDirection == MotionBlurDirection::TRAILING) ? deformed_points_list_ptr : 
				(pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, key_to) : getDeformedPoints(mpDeformedPointsListStep, multi_threaded, mpCurvesContainer.get(), key_to.time));

			assert(pPointsFrom && pPointsTo);

			const pxr::GfVec3f* p_pts_from_ptr = pPointsFrom->data();
			const pxr::GfVec3f* p_pts_to_ptr = pPointsTo->data();

			assert(p_pts_from_ptr != p_pts_to_ptr);

			const float k = ((mMotionBlurDirection == MotionBlurDirection::CENTERED) ? .5f : 1.0f) * static_cast<float>(mDeformerGeoPrimHandle.getStageTimeCodesPerSecond());

			auto calcVectorsFunc = [&](const std::size_t start, const std::size_t end) {
				for(size_t i = start; i < end; ++i) {
					(*tmp_velicities_list_ptr)[i] = (p_pts_to_ptr[i] - p_pts_from_ptr[i]) * k;
				}
			};

			if(multi_threaded) {
				mPool.detach_blocks(0u, tmp_velicities_list_ptr->size(), calcVectorsFunc);
				mPool.wait();
			} else {
				calcVectorsFunc(0u, tmp_velicities_list_ptr->size());
			}

			veolcities_list_ptr = (const PointsList*)tmp_velicities_list_ptr;
		}

		assert(veolcities_list_ptr);

		if(pxr::UsdAttribute attr_v = curves.GetVelocitiesAttr()) {
			if(!attr_v.Set(veolcities_list_ptr->getVtArray(), time_code)) {
				DLOG_ERR << "Error setting velocities attribute !";
				return false;
			}
		} else {
			DLOG_ERR << mCurvesGeoPrimHandle << " has no velocities attribute !";
		}
	}

	if(!curves.GetPointsAttr().Set(deformed_points_list_ptr->getVtArray(), time_code)) {
		return false;
	}

	if(pPointsLRUCache) {
		const std::string current_usage_str = pPointsLRUCache->getCacheUtilizationString();
		if(gLRUCacheStatsLastUsageStr != current_usage_str) {
			DLOG_DBG << "Points cache utilization: " << current_usage_str << "%";
			gLRUCacheStatsLastUsageStr = current_usage_str;
		}
	}

	if(mShowDebugGeometry) {
		drawDebugGeometry(time_code);
	}

	return true;
}

void BaseCurvesDeformer::setMotionBlurState(bool state) {
	if(mCalcMotionVectors == state) return;
	mCalcMotionVectors = state;
	makeDirty();

	DLOG_DBG << "Motion blur calculation " << (mCalcMotionVectors ? "enabled." : "disabled.");
}

void BaseCurvesDeformer::setVelocityAttrName(const std::string& name) {
	if(mVelocityAttrName == name) return;
	mVelocityAttrName = name;
	makeDirty();

	DLOG_DBG << "Velocity attribute name is set to: " << mVelocityAttrName;
}

void BaseCurvesDeformer::setSkinPrimAttrName(const std::string& name) {
	if(mSkinPrimAttrName == name) return;
	mSkinPrimAttrName = name;
	makeDirty();

	DLOG_DBG << "Skin prim ID attribute name is set to: " << mSkinPrimAttrName;
}


void BaseCurvesDeformer::makeDirty() {
	if(mDirty) return;

	DLOG_TRC << "BaseCurvesDeformer::makeDirty()";
	mStats.clear();
	mDirty = true;
	mDeformerDataWritten = false;

	static auto& cache = DeformerDataCache::getInstance();

	clearLRUCaches();
	invalidateData(DeformerDataCache::getInstance());
	DLOG_TRC << "BaseCurvesDeformer::makeDirty() done";
}

void BaseCurvesDeformer::clearLRUCaches() {
	if(PxrPointsLRUCache* pPointsLRUCache = CurvesDeformerFactory::getInstance().getPxrPointsLRUCachePtr()) {
		pPointsLRUCache->removeByName(uniqueName());
		pPointsLRUCache->removeByName(velocityKeyName());
	}
}

void BaseCurvesDeformer::showDebugGeometry(bool state) {
	if(mShowDebugGeometry == state) return;
	mShowDebugGeometry = state;
}

const std::string& BaseCurvesDeformer::toString() const {
	static const std::string kBaseDeformerString = "BaseCurvesDeformer";
	return kBaseDeformerString;
}

std::string BaseCurvesDeformer::repr() const {
	std::stringstream ss;
    ss << toString() << "(name='" << getName() << "')";
    return ss.str();
}

} // namespace Piston

std::string to_string(const Piston::BaseCurvesDeformer::Type& mt) {
#define t2s(t_) case Piston::BaseCurvesDeformer::Type::t_: return #t_;
    switch (mt) {
        t2s(FAST);
        t2s(WRAP);
        t2s(GUIDES);
        default:
            assert(false);
            return "UNKNOWN";
    }
#undef t2s
}

std::atomic_uint32_t Piston::BaseCurvesDeformer::current_id = 0;