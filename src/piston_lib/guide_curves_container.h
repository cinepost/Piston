#ifndef PISTON_LIB_GUIDE_CURVES_CONTAINER_H_
#define PISTON_LIB_GUIDE_CURVES_CONTAINER_H_

#include "framework.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <memory>
#include <vector>

namespace Piston {

class GuideCurvesContainer : public std::enable_shared_from_this<GuideCurvesContainer> {
	public:
		using SharedPtr = std::shared_ptr<GuideCurvesContainer>;

	public:
		~GuideCurvesContainer();

		static SharedPtr create();

		bool init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode reference_time_code);
		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code);

		size_t getCurvesCount() const { return mCurvesCount; }
		const pxr::VtArray<int>& getCurveVertexCounts() const { return mCurveVertexCounts; } 

		const pxr::VtArray<pxr::GfVec3f>& getRestCurvePoints() const { return mRestCurvePoints; }
		const pxr::VtArray<pxr::GfVec3f>& getLiveCurvePoints() const { return mLiveCurvePoints; }

		const std::vector<uint32_t>& getCurveOffsets() const { return mCurveOffsets; }

	protected:
		GuideCurvesContainer();

	private:
		size_t                                  mCurvesCount;
		pxr::VtArray<int> 						mCurveVertexCounts;
		std::vector<uint32_t> 					mCurveOffsets;

		pxr::VtArray<pxr::GfVec3f>              mRestCurvePoints;
		pxr::VtArray<pxr::GfVec3f>              mLiveCurvePoints;
};

} // namespace Piston

#endif // PISTON_LIB_GUIDE_CURVES_CONTAINER_H_