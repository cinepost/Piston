#include "curves_container.h"

namespace Piston {


PxrCurvesContainer::PxrCurvesContainer() {

}

bool PxrCurvesContainer::init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code) {
	auto geom_curves = pxr::UsdGeomCurves(prim_handle.getPrim());
	if(!geom_curves) {
		printf("Error getting hair geometry !\n");
		return false;
	}

	mCurvesCount = geom_curves.GetCurveCount(rest_time_code);
	if(mCurvesCount == 0) {
		printf("No curves exist in curves primitive !\n");
		return false;
	}

	// Curves. points
	pxr::VtArray<pxr::GfVec3f> curveRestPoints;

	if(!geom_curves.GetPointsAttr().Get(&curveRestPoints, rest_time_code)) {
		printf("Error getting curve points !\n");
		return false;
	}

	// Curves. Counts/offsets
	if(!geom_curves.GetCurveVertexCountsAttr().Get(&mCurveVertexCounts, rest_time_code)){
		printf("Error getting curve vertices counts !\n");
		return false;
	}

	// Calc offsets
	mCurveOffsets.resize(mCurvesCount);
	uint32_t total_vertex_count = 0u;
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveOffsets[i] = total_vertex_count;
		total_vertex_count += mCurveVertexCounts[i];

	}

	// Calc curves derivs
	mCurveRestVectors.resize(total_vertex_count);
	for(size_t i = 0; i < mCurvesCount; ++i) {
		auto curve_root_pt = curveRestPoints[mCurveOffsets[i]];
		mCurveRestVectors[mCurveOffsets[i]] = curve_root_pt;

		for(size_t j = 1; j < mCurveVertexCounts[i]; ++j) {
			mCurveRestVectors[mCurveOffsets[i] + j] = curveRestPoints[mCurveOffsets[i] + j] - curve_root_pt;
		}
	}

	assert(mCurveVertexCounts.size() == mCurveOffsets.size());
	return true;
}


PxrCurvesContainer::UniquePtr PxrCurvesContainer::create(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code) {
	PxrCurvesContainer::UniquePtr pResult = PxrCurvesContainer::UniquePtr(new PxrCurvesContainer());
	if(!pResult->init(prim_handle, rest_time_code)) return nullptr;

	return pResult;
}

PxrCurvesContainer::CurveDataPtr PxrCurvesContainer::getCurveDataPtr(size_t idx) {
	assert(idx < mCurveOffsets.size());

	return {mCurveVertexCounts[idx], &mCurveRestVectors[mCurveOffsets[idx]]};
}

const pxr::GfVec3f& PxrCurvesContainer::getCurveRootPoint(size_t idx) const {
	assert(idx < mCurveOffsets.size());

	return mCurveRestVectors[mCurveOffsets[idx]];
}
	

} // namespace Piston
