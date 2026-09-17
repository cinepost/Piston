#include "global_config.h"
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

BaseDeformer::BaseDeformer(const BaseDeformer::Type t, const std::string& name): 
	mDirty(true), 
	mDeformerDataWritten(false), 
	mPool(std::max(2u, std::thread::hardware_concurrency()) - 1), 
	mType(t), 
	mName(name), 
	mID(current_id++),
	mRestTimeCode(pxr::UsdTimeCode::Default()) {
	
	DLOG_TRC << "BaseDeformer::BaseDeformer()";

	mUniqueName = toString() + mName + std::to_string(mID);
	mDeformerSubdivLevel = 0;
}

void BaseDeformer::setDataPrimPath(const std::string& path) {
	const pxr::SdfPath new_path(path);
	if(mDataPrimPath == new_path) return;
	if(!new_path.IsPrimPath()) {
		DLOG_ERR << "Unable to set data prim path to \"" << path << "\". Path is not a prim path !";
		return;
	}
	mDataPrimPath = new_path;
	makeDirty();
}

const pxr::SdfPath& BaseDeformer::getDataPrimPath() const { 
	if(mDataPrimPath.IsPrimPath()) {
		return mDataPrimPath; 
	}

	return GlobalConfig::getInstance().getDefaultDataPrimPath();
}

void BaseDeformer::setDeformerGeoPrim(const pxr::UsdPrim& prim) {
	if(!prim.IsValid() || mDeformerGeoPrimHandle == prim) return;
	
	if(!validateDeformerGeoPrim(prim)) {
		mDeformerGeoPrimHandle.clear();
		DLOG_ERR << "Invalid geometry prim " << prim << " type!";
		return;
	}

	auto new_handle = UsdPrimHandle(prim);
	new_handle.setSubdivLevel(mDeformerSubdivLevel);
	const bool same_topology = mDeformerGeoPrimHandle.isValid() ? isSameTopology(mDeformerGeoPrimHandle, new_handle, getRestTimeCode()) : false;

	mDeformerGeoPrimHandle = std::move(new_handle);
	if(!same_topology) {
		makeDirty();
	}

	DLOG_DBG << "Deformer geometry prim is set to: " << mDeformerGeoPrimHandle;
}

void BaseDeformer::setDeformerGeoPrim(const BaseDeformer::SharedPtr& pDeformer) {
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

	auto new_handle = UsdPrimHandle(pDeformer);
	const bool same_topology = mDeformerGeoPrimHandle.isValid() ? isSameTopology(mDeformerGeoPrimHandle, new_handle, getRestTimeCode()) : false;

	mDeformerGeoPrimHandle = std::move(new_handle);
	if(!same_topology) {
		makeDirty();
	}

	DLOG_DBG << "Deformer prim is set to " << pDeformer->getName();
}

const pxr::UsdPrim& BaseDeformer::getDeformerGeoPrim() const {
	return mDeformerGeoPrimHandle.getPrim();
}

void BaseDeformer::setDeformerRestAttrName(const std::string& name) {
	if(mDeformerGeoPrimHandle.getRestAttrName() == name) return;
	mDeformerGeoPrimHandle.setRestAttrName(name);
	makeDirty();

	DLOG_DBG << "Deformer geometry rest attribute name is set to: " <<  name;
}

void BaseDeformer::setReadJsonDataFromPrim(bool state) {
	if(mReadJsonDeformerData == state) return;
	mReadJsonDeformerData = state;
	makeDirty();
}

void BaseDeformer::setRestTimeCode(pxr::UsdTimeCode time_code) {
	if(getRestTimeCode() == time_code) return;
	mRestTimeCode = time_code;
	makeDirty();
}

pxr::UsdTimeCode BaseDeformer::getRestTimeCode() const {
	if(mRestTimeCode.IsDefault()) {
		static const auto& conf = GlobalConfig::getInstance();
		return conf.getDefaultRestTimeCode();
	}

	return mRestTimeCode;
}

bool BaseDeformer::deform_dbg(pxr::UsdTimeCode time_code) {	
	return deform(time_code, false);
}

bool BaseDeformer::deform(pxr::UsdTimeCode time_code, bool multi_threaded) {
	DLOG_TRC << "Deform at time code: " << time_code.GetValue();
		
	const pxr::UsdTimeCode bind_time_code = getRestTimeCode();
	if(!buildDeformerData(bind_time_code, multi_threaded)) {
		return false;
	}

	assert(mpDeformerMeshContainer);
	if(!mpDeformerMeshContainer || mpDeformerMeshContainer->getRestPositions().empty()) {
		return false;
	}

	const size_t points_count = getDeformedPointsCount();

	auto deformPoints = [this](bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
		if(!mpDeformerMeshContainer->update(mDeformerGeoPrimHandle, time_code, isDirty())) {
			return false;
		}
		
		if(multi_threaded) { return deformMtImpl(points, time_code); }

		return deformImpl(points, time_code);
	};

	auto getDeformedPoints = [this, points_count, &deformPoints](std::unique_ptr<PointsList>& points, bool multi_threaded, const PxrPointsLRUCache::CompositeKey& key) {
		DLOG_TRC << "Deforming curves at " << key.time;

		if(!points) {
			points = std::make_unique<PointsList>(points_count, deformerOutputsOrientations());
		} else {
			points->resize(points_count);
		}

		PointsList* points_list = points.get(); 

		if (deformPoints(multi_threaded, *points_list, key.time)) {
			return (const PointsList*)points_list;
		}

		return (const PointsList*)nullptr;
	};

	auto getDeformedPointsLRU = [this, points_count, &deformPoints](bool multi_threaded, PxrPointsLRUCache* pPointsLRUCache, const PxrPointsLRUCache::CompositeKey& key) {
		assert(pPointsLRUCache);
		DLOG_TRC << "Deforming curves (using cache) at " << key.time;

		static const PointsList* sNull = nullptr;

		const PointsList* p_points_list_ptr = pPointsLRUCache->get(key);
		if(p_points_list_ptr) {
			DLOG_TRC << "Cache has entry key " << to_string(key);
			return p_points_list_ptr;
		}

		PointsList* p_new_points_list = pPointsLRUCache->put(key, points_count, deformerOutputsOrientations());
		if (deformPoints(multi_threaded, *p_new_points_list, key.time)) {
			return (const PointsList*)p_new_points_list;
		}

		return sNull;
	};

	const PxrPointsLRUCache::CompositeKey curr_key = {uniqueName(), time_code};
	PxrPointsLRUCache* pPointsLRUCache = mUsePointsCache ? CurvesDeformerFactory::getInstance().getPxrPointsLRUCachePtr() : nullptr;

	PxrPointsLRUCacheShrinkLock cache_shrink_lock(pPointsLRUCache); // avoid cache shrinking during deformation stage
	if(cache_shrink_lock.isValid()) {
		DLOG_TRC << "pPointsLRUCache locked";
	}

	const PointsList* deformed_points_list_ptr = pPointsLRUCache ? getDeformedPointsLRU(multi_threaded, pPointsLRUCache, curr_key) : getDeformedPoints(mpDeformedPointsList, multi_threaded, curr_key);
	assert(deformed_points_list_ptr);
	if(!deformed_points_list_ptr) {
		DLOG_ERR << "Error getting deformed points list!";
		return false;
	}

	if(!outputDeformedPoints(deformed_points_list_ptr, time_code)) {
		DLOG_ERR << "Error setting deformerd points !";
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
		drawDebugSubdivDeformerGeometry(time_code);
		drawDebugGeometry(time_code, deformed_points_list_ptr);
	}

	return true;
}

bool BaseDeformer::buildDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!mDirty) return true;

	SimpleProfiler::clear();

	if(!mDeformerGeoPrimHandle) {
		DLOG_ERR << "No deformer UsdPrim is set !!!";
		return false;
	}

	if(!mpDeformerMeshContainer) {
		mpDeformerMeshContainer = MeshContainer::create(mDeformerGeoPrimHandle, rest_time_code);
		if(!mpDeformerMeshContainer) {
			DLOG_ERR << "Error creating deformer mesh container for prim " << mDeformerGeoPrimHandle << " !";
			return false;
		}
	} else {
		if(!mpDeformerMeshContainer->init(mDeformerGeoPrimHandle, rest_time_code)) {
			DLOG_ERR << "Error initializing deformer mesh container for prim " << mDeformerGeoPrimHandle << " !";
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

bool BaseDeformer::writeJsonDataToPrim(pxr::UsdTimeCode time_code) {
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

void BaseDeformer::setDeformerSubdivLevel(uint8_t level) {
	level = std::min(level, kMaxSubdivLevel);

	if(level == getDeformerSubdivLevel()) return;

	if(mDeformerGeoPrimHandle.isValid()) {
		mDeformerGeoPrimHandle.setSubdivLevel(level);
		mDeformerSubdivLevel = mDeformerGeoPrimHandle.getSubdivLevel();
	} else {
		mDeformerSubdivLevel = level;
	}

	makeDirty();
}


uint8_t BaseDeformer::getDeformerSubdivLevel() const {
	assert(mDeformerSubdivLevel == mDeformerGeoPrimHandle.getSubdivLevel());

	if(mDeformerGeoPrimHandle.isValid()) {
		return  mDeformerGeoPrimHandle.getSubdivLevel();
	}

	return mDeformerSubdivLevel;
}


void BaseDeformer::setPointsCacheUsageState(bool state) {
	if(mUsePointsCache == state) return;
	mUsePointsCache = state;

	if(!mUsePointsCache) {
		clearLRUCaches();
	}
}

void BaseDeformer::setInstancingState(bool state) {
	if(mInstancingEnabled == state) return;
	mInstancingEnabled = state;

	static auto const& conf = GlobalConfig::getInstance();
	if(mInstancingEnabled && !conf.getDataInstancingState()) {
		DLOG_INF << "Deformers data instancing is disabled!";
		return;
	}

	makeDirty();
}

bool BaseDeformer::getInstancingState() const { 
	static auto const& conf = GlobalConfig::getInstance();
	return mInstancingEnabled && conf.getDataInstancingState(); 
}

bool BaseDeformer::getPointsCacheUsageState() const {
	static auto const& conf = GlobalConfig::getInstance();
	return mUsePointsCache && conf.getPointsCacheUsageState();
}

void BaseDeformer::drawDebugSubdivDeformerGeometry(pxr::UsdTimeCode time_code) {
	if(!mDeformerGeoPrimHandle.isMeshGeoPrim()) return;

	auto* pRefiner = mDeformerGeoPrimHandle.getMeshRefiner(getRestTimeCode());
	if(!pRefiner || pRefiner->getMaxLevel() == 0) return;

	pRefiner->update(time_code);
	const pxr::UsdGeomMesh& subdMesh = pRefiner->getOutputMesh();

	if(!mpSubdivDebugGeo) {
		mpSubdivDebugGeo = DebugGeo::create(getName() + "_subdiv_mesh");
	} 
	
	mpSubdivDebugGeo->clear();
	
	pxr::VtArray<pxr::GfVec3f> subd_points;

	if(!subdMesh.GetPointsAttr().Get(&subd_points, time_code)) {
		LOG_ERR << "Error getting " << mDeformerGeoPrimHandle.getPath() << " subdivided surface points at " << time_code.GetValue();
		return;
	}

	LOG_TRC << "Subd points count " << subd_points.size();

	for(const auto& point: subd_points) {
		DebugGeo::Pt pt(point, {0.0, 1.0, 0.0}, 5.f * mDebugGeometryMult);
		mpSubdivDebugGeo->addPoint(pt);
	}

	mpSubdivDebugGeo->build("/debugSubdivMesh", mDeformerGeoPrimHandle.getStage());
}

void BaseDeformer::setSkinPrimAttrName(const std::string& name) {
	if(mSkinPrimAttrName == name) return;
	mSkinPrimAttrName = name;
	makeDirty();
	DLOG_DBG << "Skin prim ID attribute name is set to: " << mSkinPrimAttrName;
}

void BaseDeformer::makeDirty() {
	if(mDirty) return;

	DLOG_TRC << "BaseDeformer::makeDirty()";
	mStats.clear();
	mDirty = true;
	mDeformerDataWritten = false;

	clearLRUCaches();
	invalidateData(DeformerDataCache::getInstance());
	DLOG_TRC << "BaseDeformer::makeDirty() done";
}

void BaseDeformer::clearLRUCaches() {
	if(auto* pPointsLRUCache = CurvesDeformerFactory::getInstance().getPxrPointsLRUCachePtr()) {
		pPointsLRUCache->removeByName(uniqueName());
	}
}

void BaseDeformer::showDebugGeometry(bool state) {
	if(mShowDebugGeometry == state) return;
	mShowDebugGeometry = state;
}

const std::string& BaseDeformer::toString() const {
	static const std::string kBaseDeformerString = "BaseDeformer";
	return kBaseDeformerString;
}

std::string BaseDeformer::repr() const {
	std::stringstream ss;
    ss << toString() << "(name='" << getName() << "')";
    return ss.str();
}

} // namespace Piston

std::string to_string(const Piston::BaseDeformer::Type& mt) {
#define t2s(t_) case Piston::BaseDeformer::Type::t_: return #t_;
    switch (mt) {
        t2s(FAST);
        t2s(WRAP);
        t2s(GUIDES);
        t2s(POINT_INSTANCER);
        default:
            assert(false);
            return "UNKNOWN";
    }
#undef t2s
}

std::atomic_uint32_t Piston::BaseDeformer::current_id = 0;