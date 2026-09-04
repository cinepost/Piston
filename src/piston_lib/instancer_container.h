#ifndef PISTON_LIB_INSTANCER_CONTAINER_H_
#define PISTON_LIB_INSTANCER_CONTAINER_H_

#include "framework.h"
#include "common.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/pointInstancer.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>

#include <memory>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

namespace Piston {

class InstancerContainer : public std::enable_shared_from_this<InstancerContainer> {
	public:
		using UniquePtr = std::unique_ptr<InstancerContainer>;

	public:
		static UniquePtr create();

		bool init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code, const pxr::VtArray<pxr::GfVec3f>* pRestPointsDataExt = nullptr, const pxr::VtArray<pxr::GfVec3f>* pLivePointsDataExt = nullptr);
		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force);

		size_t getInstanceCount() const { return mInstanceCount; }


		const pxr::VtArray<pxr::GfVec3f>& getRestInstancePoints() const { return mRestInstancePoints.AsConst(); }
		const pxr::VtArray<pxr::GfVec3f>& getLiveInstancePoints() const { return mLiveInstancePoints.AsConst(); }

		const pxr::VtArray<pxr::GfQuath>& getRestOrientations() const { return mRestOrientations.AsConst(); }


	protected:
		InstancerContainer();

	private:
		bool                                    mIsInitialized;
		size_t                                  mInstanceCount;

		pxr::VtArray<pxr::GfVec3f>              mRestInstancePoints;
		pxr::VtArray<pxr::GfVec3f>              mLiveInstancePoints;

		pxr::VtArray<pxr::GfQuath>              mRestOrientations;

		pxr::UsdTimeCode 						mLastUpdateTimeCode;
};

} // namespace Piston

#endif // PISTON_LIB_INSTANCER_CONTAINER_H_