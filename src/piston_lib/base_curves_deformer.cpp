#include "deformer_factory.h"
#include "base_curves_deformer.h"
#include "geometry_tools.h"
#include "pxr_points_lru_cache.h"
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

	makeDirty();
	mpTempStage = pxr::UsdStage::CreateInMemory();
}

void BaseCurvesDeformer::setDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	if(mDeformerGeoPrimHandle == geoPrim) return;
	
	if(!validateDeformerGeoPrim(geoPrim)) {
		mDeformerGeoPrimHandle.clear();
		DLOG_ERR << "Deformer " << mName << " invalid geometry prim " <<  geoPrim.GetPath().GetText() << " type!";
		return;
	}

	mDeformerGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	DLOG_DBG << "Deformer " << mName << " geometry prim is set to: " << mDeformerGeoPrimHandle;
}

void BaseCurvesDeformer::setCurvesGeoPrim(const pxr::UsdPrim& geoPrim) {
	if(mCurvesGeoPrimHandle == geoPrim) return;
	if(!isBasisCurvesGeoPrim(geoPrim)) {
		mCurvesGeoPrimHandle.clear();
		DLOG_ERR << "Curves geometry prim is not \"BasisCurves\"!";
		return;
	}

	mCurvesGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	DLOG_DBG << "Curves geometry prim is set to: " << mCurvesGeoPrimHandle;
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

	if(!mDeformerGeoPrimHandle || !mCurvesGeoPrimHandle) {
		DLOG_ERR << "No mesh or curves UsdPrim is set on deformer " << mName << " !";
		return false;
	}

	mpCurvesContainer = PxrCurvesContainer::create(mCurvesGeoPrimHandle, rest_time_code);
	if(!mpCurvesContainer) {
		DLOG_ERR << "Error creating curves container for deformer " << mName << " !";
		return false;
	}

	if(!buildDeformerDataImpl(rest_time_code, multi_threaded)) {
		DLOG_ERR << "Error building " << mName <<" deformer data !";
		return false;
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

bool BaseCurvesDeformer::getPointsCacheUsageState() const {
	return mUsePointsCache && CurvesDeformerFactory::getInstance().getPointsCacheUsageState();
}

bool BaseCurvesDeformer::deform_dbg(pxr::UsdTimeCode time_code) {	
	return deform(time_code, false);
}

bool BaseCurvesDeformer::deform(pxr::UsdTimeCode time_code, bool multi_threaded) {
	DLOG_DBG << "Deform at time code: " << time_code.GetValue();
		
	const pxr::UsdTimeCode bind_time_code = time_code.IsDefault() ? getRestTimeCode() : time_code;
	if(!buildDeformerData(bind_time_code, multi_threaded)) {
		return false;
	}

	assert(mpCurvesContainer);	
	if(!mpCurvesContainer || mpCurvesContainer->empty()) {
		return false;
	}

	auto deformPoints = [this](bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
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

	PxrPointsLRUCacheShrinkLock cache_shrink_lock(pPointsLRUCache); // avoid cache shrinking during deformation stage
	if(cache_shrink_lock.isValid()) DLOG_TRC << "pPointsLRUCache locked";

	const PointsList* deformed_points_list_ptr = pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, curr_key) : getDeformedPoints(mpDeformedPointsList, multi_threaded, mpCurvesContainer.get(), time_code);

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	if(!curves.GetPointsAttr().Set(deformed_points_list_ptr->getVtArray(), time_code)) {
		return false;
	}

	if(mCalcMotionVectors) {
		const PxrPointsLRUCache::CompositeKey key_vel = {velocityKeyName(), time_code};
		const PointsList* veolcities_list_ptr = pPointsLRUCache ? pPointsLRUCache->get(key_vel) : nullptr;

		if(!veolcities_list_ptr) {
			PointsList* tmp_velicities_list_ptr = pPointsLRUCache ? pPointsLRUCache->put(key_vel, mpCurvesContainer->getTotalVertexCount()) : getTempVelocitiesList(mpCurvesContainer->getTotalVertexCount());

			const PxrPointsLRUCache::CompositeKey key_from = {uniqueName(), (mMotionBlurDirection != MotionBlurDirection::LEADING) ? (time_code.GetValue() - 1.0) : time_code};
			const PxrPointsLRUCache::CompositeKey key_to = {uniqueName(), (mMotionBlurDirection != MotionBlurDirection::TRAILING) ? (time_code.GetValue() + 1.0) : time_code};

			DLOG_DBG << "Motion blur: " <<  std::to_string(key_from.time.GetValue()) << " to " <<  std::to_string(key_to.time.GetValue());

			const PointsList* pPointsFrom = (mMotionBlurDirection == MotionBlurDirection::LEADING) ? deformed_points_list_ptr : 
				(pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, key_from) : getDeformedPoints(mpDeformedPointsListStep, multi_threaded, mpCurvesContainer.get(), key_from.time));
			
			const PointsList* pPointsTo = (mMotionBlurDirection == MotionBlurDirection::TRAILING) ? deformed_points_list_ptr : 
				(pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, mpCurvesContainer.get(), pPointsLRUCache, key_to) : getDeformedPoints(mpDeformedPointsListStep, multi_threaded, mpCurvesContainer.get(), key_to.time));

			assert(pPointsFrom && pPointsTo);

			const pxr::GfVec3f* p_pts_from_ptr = pPointsFrom->data();
			const pxr::GfVec3f* p_pts_to_ptr = pPointsTo->data();

			const float k = ((mMotionBlurDirection == MotionBlurDirection::CENTERED) ? .5f : 1.0f) * static_cast<float>(mDeformerGeoPrimHandle.getStageTimeCodesPerSecond());

			for(size_t i = 0; i < tmp_velicities_list_ptr->size(); ++i) {
				(*tmp_velicities_list_ptr)[i] = (p_pts_to_ptr[i] - p_pts_from_ptr[i]) * k;
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
	mStats.clear();
	mDirty = true;
	mDeformerDataWritten = false;

	clearLRUCaches();
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

} // namespace Piston

std::atomic_uint32_t Piston::BaseCurvesDeformer::current_id = 0;