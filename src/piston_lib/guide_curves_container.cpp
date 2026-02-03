#include "guide_curves_container.h"


namespace Piston {

GuideCurvesContainer::GuideCurvesContainer(): mCurvesCount(0), mExternalRestPointDataSource(false), mExternalLivePointDataSource(false) {
	mCurveOffsets.reserve(1024);
}

GuideCurvesContainer::UniquePtr GuideCurvesContainer::create() {
	return GuideCurvesContainer::UniquePtr(new GuideCurvesContainer());
}

bool GuideCurvesContainer::init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code, const pxr::VtArray<pxr::GfVec3f>* pRestPointsDataExt, const pxr::VtArray<pxr::GfVec3f>* pLivePointsDataExt) {
	if(!prim_handle.isBasisCurvesGeoPrim()) {
		return false;
	}

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

	// Curves. Counts/offsets
	if(!geom_curves.GetCurveVertexCountsAttr().Get(&mCurveVertexCounts, rest_time_code)){
		std::cerr << "Error getting curves vertices counts from " << prim_handle.getName() << "  !" << std::endl;
		return false;
	}
	assert(mCurveVertexCounts.size() == mCurvesCount);

	if(pRestPointsDataExt) {
		mRestCurvePoints= *pRestPointsDataExt;
		mExternalRestPointDataSource = true;
	} else {
		// Curve rest points
		if(!geom_curves.GetPointsAttr().Get(&mRestCurvePoints, rest_time_code)) {
			std::cerr << "Error getting curves points from " << prim_handle.getName() << " !" << std::endl;
			return false;
		}
	}

	if(pLivePointsDataExt) {
		mLiveCurvePoints = * pLivePointsDataExt;
		mExternalLivePointDataSource = true;
	}

	// Calc offsets
	mCurveOffsets.resize(mCurvesCount);
	uint32_t total_vertex_count = 0u;
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveOffsets[i] = total_vertex_count;
		total_vertex_count += mCurveVertexCounts[i];

	}

	assert(mCurveVertexCounts.size() == mCurveOffsets.size());

	return true;
}

bool GuideCurvesContainer::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) {
	assert(!mExternalLivePointDataSource);

	return true;
}

const pxr::GfVec3f& GuideCurvesContainer::getGuideRestPoint(uint32_t guide_id, uint32_t vertex_id) const {
	assert(guide_id < mCurveVertexCounts.size() && guide_id < mCurveOffsets.size());
	assert(vertex_id < mCurveVertexCounts[guide_id] && vertex_id < mCurveOffsets[guide_id]);

	const size_t global_vtx_id = mCurveOffsets[guide_id] + vertex_id;

	const auto& rest_points = mRestCurvePoints.AsConst();
	assert(global_vtx_id < rest_points.size());

	return rest_points[global_vtx_id];
}

const pxr::GfVec3f& GuideCurvesContainer::getGuideLivePoint(uint32_t guide_id, uint32_t vertex_id) const {
	assert(guide_id < mCurveVertexCounts.size() && guide_id < mCurveOffsets.size());
	assert(vertex_id < mCurveVertexCounts[guide_id] && vertex_id < mCurveOffsets[guide_id]);

	const size_t global_vtx_id = mCurveOffsets[guide_id] + vertex_id;

	const auto& live_points = mRestCurvePoints.AsConst();
	assert(global_vtx_id < live_points.size());

	return live_points[global_vtx_id];
}

} // namespace Piston
