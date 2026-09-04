#include "instancer_container.h"
#include "logging.h"

#include <limits>
#include <cmath>


namespace Piston {

InstancerContainer::InstancerContainer(): mIsInitialized(false), mInstanceCount(0), mLastUpdateTimeCode(std::numeric_limits<double>::lowest()) {
}

InstancerContainer::UniquePtr InstancerContainer::create() {
	return InstancerContainer::UniquePtr(new InstancerContainer());
}

bool InstancerContainer::init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code, const pxr::VtArray<pxr::GfVec3f>* pRestPointsDataExt, const pxr::VtArray<pxr::GfVec3f>* pLivePointsDataExt) {
	mIsInitialized = false;

    if(!prim_handle.isPointInstancerGeoPrim()) {
		return false;
	}

	auto instancer = pxr::UsdGeomPointInstancer(prim_handle.getPrim());
	if(!instancer) {
		LOG_ERR << "Error getting instancer from " << prim_handle.getName() << " !";
		return false;
	}

	mInstanceCount = instancer.GetInstanceCount(rest_time_code);
	if(mInstanceCount == 0) {
		LOG_ERR << "No instances exist in primitive " << prim_handle.getName() << " !";
		return false;
	}

    // Get rest instance point positions
    if(prim_handle.getRestAttrName().empty() || !prim_handle.fetchAttributeValues<pxr::GfVec3f>(prim_handle.getRestAttrName(), mRestInstancePoints, rest_time_code)) {

    	if(!instancer.GetPositionsAttr().Get(&mRestInstancePoints, rest_time_code)) {
            LOG_ERR << "Error getting instance rest positions from " << prim_handle.getName() << " !";
            return false;
        }
    }

    // Try to get instance orientations
    instancer.GetOrientationsAttr().Get(&mRestOrientations, rest_time_code);
    if(mRestInstancePoints.size() != mRestInstancePoints.size()) {
    	LOG_WRN << prim_handle << " orientation attribute size mismatch!";
    	mRestOrientations.clear();
    }

	mLastUpdateTimeCode = rest_time_code;
	mIsInitialized = true;
    return true;
}

bool InstancerContainer::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force) {
	assert(prim_handle.isPointInstancerGeoPrim());
    assert(mIsInitialized);

    if(mIsInitialized) return false;

	if(!force) {
		if(mLastUpdateTimeCode == time_code) return true;
		//if(!prim_handle.hasPositionsTimeSamples(mLastUpdateTimeCode, time_code)) return true;
	}

	// Curve live point positions
	auto instancer = pxr::UsdGeomPointInstancer(prim_handle.getPrim());
	if(!instancer.GetPositionsAttr().Get(&mLiveInstancePoints, time_code)) {
		LOG_ERR << "Error getting instance live positions from " << prim_handle << " !";
		return false;
	}

	if(mLiveInstancePoints.size() != mRestInstancePoints.size()) {
		LOG_ERR << prim_handle.getPath() << " \"rest\" and \"live\" instance positions count (" << mRestInstancePoints.size() << " vs " << mLiveInstancePoints.size() << " ) mismatch !";
		return false;
	}

	mLastUpdateTimeCode = time_code;
	return true;
}

} // namespace Piston
