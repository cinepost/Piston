#ifndef PISTON_LIB_GUIDE_CURVES_CONTAINER_H_
#define PISTON_LIB_GUIDE_CURVES_CONTAINER_H_

#include "framework.h"
#include "common.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <memory>
#include <vector>

namespace Piston {

class GuideCurvesContainer : public std::enable_shared_from_this<GuideCurvesContainer> {
	public:
		using UniquePtr = std::unique_ptr<GuideCurvesContainer>;

	public:
		static UniquePtr create();

		bool init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode reference_time_code, const pxr::VtArray<pxr::GfVec3f>* pRestPointsDataExt = nullptr, const pxr::VtArray<pxr::GfVec3f>* pLivePointsDataExt = nullptr);

		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code);

		size_t getCurvesCount() const { return mCurvesCount; }
		const pxr::VtArray<int>& getCurveVertexCounts() const { return mCurveVertexCounts.AsConst(); } 
		size_t getCurveVertexCount(size_t idx) const { assert(idx < mCurveVertexCounts.size()); return getCurveVertexCounts()[idx]; } 


		const pxr::VtArray<pxr::GfVec3f>& getRestCurvePoints() const { return mRestCurvePoints.AsConst(); }
		const pxr::VtArray<pxr::GfVec3f>& getLiveCurvePoints() const { return mLiveCurvePoints.AsConst(); }

		const std::vector<uint32_t>& getCurveOffsets() const { return mCurveOffsets; }
		size_t getCurveVertexOffset(size_t idx) const { assert(idx < mCurveOffsets.size()); return mCurveOffsets[idx]; }

		const pxr::GfVec3f& getGuideRestPoint(uint32_t guide_id, uint32_t vertex_id) const;
		const pxr::GfVec3f& getGuideLivePoint(uint32_t guide_id, uint32_t vertex_id) const;

	protected:
		GuideCurvesContainer();

	private:
		size_t                                  mCurvesCount;
		pxr::VtArray<int> 						mCurveVertexCounts;
		std::vector<uint32_t> 					mCurveOffsets;

		pxr::VtArray<pxr::GfVec3f>              mRestCurvePoints;
		pxr::VtArray<pxr::GfVec3f>              mLiveCurvePoints;
		bool                                    mExternalRestPointDataSource = false;
		bool                                    mExternalLivePointDataSource = false;
};

} // namespace Piston

#endif // PISTON_LIB_GUIDE_CURVES_CONTAINER_H_