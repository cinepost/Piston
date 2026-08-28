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

	DLOG_DBG << "Deformer gseometry rest attribute name is set to: " <<  name;
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
	if(mDeformerSubdivLevel == level && mDeformerGeoPrimHandle.getSubdivLevel() == level) return;
	mDeformerSubdivLevel = std::min(level, kMaxSubdivLevel);

	if(mDeformerGeoPrimHandle.isValid()) {
		mDeformerGeoPrimHandle.setSubdivLevel(mDeformerSubdivLevel);
	}

	makeDirty();
}


uint8_t BaseDeformer::getDeformerSubdivLevel() const {
	if(mDeformerGeoPrimHandle.isValid()) {
		assert(mDeformerSubdivLevel == mDeformerGeoPrimHandle.getSubdivLevel());
	}
	return mDeformerGeoPrimHandle.getSubdivLevel();
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

void BaseDeformer::setMotionBlurState(bool state) {
	if(mCalcMotionVectors == state) return;
	mCalcMotionVectors = state;
	makeDirty();
	DLOG_DBG << "Motion blur calculation " << (mCalcMotionVectors ? "enabled." : "disabled.");
}

void BaseDeformer::setVelocityAttrName(const std::string& name) {
	if(mVelocityAttrName == name) return;
	mVelocityAttrName = name;
	makeDirty();
	DLOG_DBG << "Velocity attribute name is set to: " << mVelocityAttrName;
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
		pPointsLRUCache->removeByName(velocityKeyName());
	}

	if(auto* pPointsLRUCache = CurvesDeformerFactory::getInstance().getPxrInstanceLRUCachePtr()) {
		pPointsLRUCache->removeByName(uniqueName());
		pPointsLRUCache->removeByName(velocityKeyName());
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