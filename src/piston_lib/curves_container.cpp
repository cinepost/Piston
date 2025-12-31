#include "curves_container.h"

namespace Piston {


PxrCurvesContainer::PxrCurvesContainer(): mCurvesCount(0) {

}

PxrCurvesContainer::PxrCurvesContainer(PxrCurvesContainer& other) {
	mCurvesCount = other.mCurvesCount;
	mCurveOffsets = other.mCurveOffsets;
	mCurveRootPositions = other.mCurveRootPositions;
	mCurveVectors = other.mCurveVectors;
}

bool PxrCurvesContainer::init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code) {
	auto geom_curves = pxr::UsdGeomCurves(prim_handle.getPrim());
	if(!geom_curves) {
		std::cerr << "Error getting curves geometry from " << prim_handle.getName() << " !" << std::endl;
		return false;
	}

	mCurvesCount = geom_curves.GetCurveCount(rest_time_code);
	if(mCurvesCount == 0) {
		std::cerr << "No curves exist in primitive " << prim_handle.getName() << " !" << std::endl;
		return false;
	}

	// Curves. points
	pxr::VtArray<pxr::GfVec3f> curvePoints;

	if(!geom_curves.GetPointsAttr().Get(&curvePoints, rest_time_code)) {
		std::cerr << "Error getting curves points from " << prim_handle.getName() << " !" << std::endl;
		return false;
	}

	// Curves. Counts/offsets
	if(!geom_curves.GetCurveVertexCountsAttr().Get(&mCurveVertexCounts, rest_time_code)){
		std::cerr << "Error getting curves vertices counts from " << prim_handle.getName() << "  !" << std::endl;
		return false;
	}

	assert(mCurveVertexCounts.size() == mCurvesCount);

	// Calc offsets
	mCurveOffsets.resize(mCurvesCount);
	uint32_t total_vertex_count = 0u;
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveOffsets[i] = total_vertex_count;
		total_vertex_count += mCurveVertexCounts[i];

	}

	// Calc curves derivs
	mCurveRootPositions.resize(mCurvesCount);
	mCurveVectors.resize(total_vertex_count);
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveRootPositions[i] = curvePoints[mCurveOffsets[i]];
		
		mCurveVectors[mCurveOffsets[i]] = {0.f, 0.f, 0.f};

		for(size_t j = 1; j < mCurveVertexCounts[i]; ++j) {
			mCurveVectors[mCurveOffsets[i] + j] = curvePoints[mCurveOffsets[i] + j] - mCurveRootPositions[i];
		}
	}

	assert(mCurveVertexCounts.size() == mCurveOffsets.size());

	return true;
}

PxrCurvesContainer::UniquePtr PxrCurvesContainer::create() {
	return PxrCurvesContainer::UniquePtr(new PxrCurvesContainer());
}

PxrCurvesContainer::UniquePtr PxrCurvesContainer::create(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code) {
	PxrCurvesContainer::UniquePtr pResult = PxrCurvesContainer::UniquePtr(new PxrCurvesContainer());
	if(!pResult->init(prim_handle, rest_time_code)) return nullptr;

	return pResult;
}

PxrCurvesContainer::CurveDataPtr PxrCurvesContainer::getCurveDataPtr(size_t curve_idx) {
	assert(curve_idx < mCurveOffsets.size());

	return {mCurveVertexCounts[curve_idx], &mCurveVectors[mCurveOffsets[curve_idx]]};
}

const pxr::GfVec3f& PxrCurvesContainer::getCurveRootPoint(size_t curve_idx) const {
	assert(curve_idx < mCurveRootPositions.size());

	return mCurveRootPositions[curve_idx];
}

} // namespace Piston
