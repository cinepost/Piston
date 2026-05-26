#ifndef PISTON_LIB_CURVES_CONTAINER_H_
#define PISTON_LIB_CURVES_CONTAINER_H_

#include "framework.h"
#include "common.h"
#include "kdtree.hpp"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>
#include <pxr/base/gf/matrix3f.h>

#include <limits>
#include <string>
#include <array>
#include <unordered_map>
#include <memory>
#include <utility>
#include <chrono>


namespace Piston {

class PxrCurvesContainer {
	public:

		enum class Space {
			UNKNOWN = 0,	// 
			LOCAL, 			// usd space
			DEFORMER   		// some space that is used by deformer
		};

		using UniquePtr = std::unique_ptr<PxrCurvesContainer>;
		using CurveDataPtr = std::pair<int, pxr::GfVec3f*>;  // curve <count, ptr> pair
		using CurveDataConstPtr = std::pair<int, const pxr::GfVec3f*>;  // curve <count, ptr> pair

		static UniquePtr create();
		static UniquePtr create(const UsdPrimHandle& prim_handle, const std::string& rest_attr_name, pxr::UsdTimeCode rest_time_code = pxr::UsdTimeCode::Default());

		bool init(const UsdPrimHandle& prim_handle, const std::string& rest_attr_name, pxr::UsdTimeCode reference_time_code);
		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force);

		bool empty() const { return mCurvesCount == 0; }

		size_t 		getCurvesCount() const { return mCurvesCount; }
		size_t 		getTotalVertexCount() const { return mCurveVectors.size(); }
		uint32_t	getCurveVertexOffset(size_t curve_idx) const { return mCurveOffsets[curve_idx]; }

		int 		getCurveVertexCount(size_t curve_idx) const { return mCurveVertexCounts[curve_idx]; }

		CurveDataPtr getCurveDataPtr(size_t curve_idx);
		CurveDataConstPtr getCurveDataPtr(size_t curve_idx) const;

		const pxr::GfVec3f& getCurveRootPoint(size_t curve_idx) const;

		const pxr::VtArray<pxr::GfVec3f>& getRestCurvePoints() const { return mRestCurvePoints.AsConst(); }

		bool  sUpdated() const { return mUpdated; }

		Space getSpace() const { return mSpace;}

		void  setSpace(const Space space) { mSpace = space; }

	private:
		PxrCurvesContainer();
		PxrCurvesContainer(PxrCurvesContainer& other);
	
	private:
		Space 									mSpace = Space::UNKNOWN;
		size_t                                  mCurvesCount;
		pxr::VtArray<int> 						mCurveVertexCounts;
		std::vector<uint32_t> 					mCurveOffsets;
		std::vector<pxr::GfVec3f>              	mCurveRootPositions;
		pxr::VtArray<pxr::GfVec3f>              mCurveVectors;

		pxr::VtArray<pxr::GfVec3f> 				mTempCurvePoints;
		pxr::VtArray<pxr::GfVec3f>              mRestCurvePoints;

		pxr::UsdTimeCode                        mLastUpdateTimeCode;
		bool                                    mUpdated;
};

inline std::string to_string(PxrCurvesContainer::Space space) {
	switch(space) {
		case PxrCurvesContainer::Space::LOCAL:
			return "LOCAL";
		case PxrCurvesContainer::Space::DEFORMER:
			return "DEFORMER";
		case PxrCurvesContainer::Space::UNKNOWN:
		default:
			return "UNKNOWN";
	}
}

} // namespace Piston

#endif // PISTON_LIB_CURVES_CONTAINER_H_