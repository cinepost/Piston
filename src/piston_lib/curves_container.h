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
		using UniquePtr = std::unique_ptr<PxrCurvesContainer>;
		using CurveDataPtr = std::pair<int, pxr::GfVec3f*>;  // curve <count, ptr> pair

		static UniquePtr create(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code = pxr::UsdTimeCode::Default());

		bool init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode reference_time_code);

		bool empty() const { return mCurvesCount == 0; }

		size_t 		getCurvesCount() const { return mCurvesCount; }
		size_t 		getTotalVertexCount() const { return mCurveRestVectors.size(); }
		uint32_t	getCurveVertexOffset(size_t curve_idx) const { return mCurveOffsets[curve_idx]; }

		int 		getCurveVertexCount(size_t curve_idx) const { return mCurveVertexCounts[curve_idx]; }

		std::vector<pxr::GfVec3f>& getPointsCache() { return mPointsCache; }
		std::vector<pxr::GfVec3f>* getPointsCachePtr() { return &mPointsCache; }

		const std::vector<pxr::GfVec3f>& getPointsCache() const { return mPointsCache; }
		const std::vector<pxr::GfVec3f>* getPointsCachePtr() const { return &mPointsCache; }

		pxr::VtArray<pxr::GfVec3f> getPointsCacheVtArray() const;

		CurveDataPtr getCurveDataPtr(size_t curve_idx);

		const pxr::GfVec3f& getCurveRootPoint(size_t curve_idx) const;

	private:
		PxrCurvesContainer();
	
	private:
		size_t                                  mCurvesCount;
		pxr::VtArray<int> 						mCurveVertexCounts;
		std::vector<uint32_t> 					mCurveOffsets;
		std::vector<pxr::GfVec3f>              	mCurveRestRootPositions;
		pxr::VtArray<pxr::GfVec3f>              mCurveRestVectors;

		std::vector<pxr::GfVec3f> 				mPointsCache;

		mutable pxr::Vt_ArrayForeignDataSource 	mForeignDataSource;
};

} // namespace Piston

#endif // PISTON_LIB_CURVES_CONTAINER_H_