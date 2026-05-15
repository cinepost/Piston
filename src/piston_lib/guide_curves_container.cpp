#include "guide_curves_container.h"
#include "logging.h"

#include <limits>


namespace Piston {

GuideCurvesContainer::GuideCurvesContainer(): mCurvesCount(0), mExternalRestPointDataSource(false), mExternalLivePointDataSource(false), mLastUpdateTimeCode(std::numeric_limits<double>::lowest()) {
	mCurveOffsets.reserve(1024);
}

GuideCurvesContainer::UniquePtr GuideCurvesContainer::create() {
	return GuideCurvesContainer::UniquePtr(new GuideCurvesContainer());
}

bool GuideCurvesContainer::init(const UsdPrimHandle& prim_handle, const std::string& rest_attr_name, pxr::UsdTimeCode rest_time_code, const pxr::VtArray<pxr::GfVec3f>* pRestPointsDataExt, const pxr::VtArray<pxr::GfVec3f>* pLivePointsDataExt) {
	if(!prim_handle.isBasisCurvesGeoPrim()) {
		return false;
	}

	auto geom_curves = pxr::UsdGeomCurves(prim_handle.getPrim());
	if(!geom_curves) {
		LOG_ERR << "Error getting curves geometry from " << prim_handle.getName() << " !";
		return false;
	}

	mCurvesCount = geom_curves.GetCurveCount(rest_time_code);
	if(mCurvesCount == 0) {
		LOG_ERR << "No curves exist in primitive " << prim_handle.getName() << " !";
		return false;
	}

	// Curves. Counts/offsets
	if(!geom_curves.GetCurveVertexCountsAttr().Get(&mCurveVertexCounts, rest_time_code)){
		LOG_ERR << "Error getting curves vertices counts from " << prim_handle.getName() << "  !";
		return false;
	}
	assert(mCurveVertexCounts.size() == mCurvesCount);

	if(pRestPointsDataExt) {
		mRestCurvePoints= *pRestPointsDataExt;
		mExternalRestPointDataSource = true;
	} else {
		// Curve rest points
		if(rest_attr_name.empty() || !prim_handle.fetchAttributeValues<pxr::GfVec3f>(rest_attr_name, mRestCurvePoints, rest_time_code)) {
			if(!prim_handle.getPoints(mRestCurvePoints, rest_time_code)) {
				LOG_ERR << "Error getting curves \"rest\" points from " << prim_handle.getName() << " !";
				return false;
			}
		}
	}

	if(pLivePointsDataExt) {
		mLiveCurvePoints = *pLivePointsDataExt;
		mExternalLivePointDataSource = true;
	} else {
		mLiveCurvePoints = mRestCurvePoints;
	}

	// Calc offsets
	mCurveOffsets.resize(mCurvesCount);
	uint32_t total_vertex_count = 0u;
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveOffsets[i] = total_vertex_count;
		total_vertex_count += mCurveVertexCounts[i];
	}

	assert(mCurveVertexCounts.size() == mCurveOffsets.size());
	mLastUpdateTimeCode = rest_time_code;
	return true;
}

bool GuideCurvesContainer::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force) {
	assert(!mExternalLivePointDataSource);
	assert(prim_handle.isBasisCurvesGeoPrim());

	if(!force) {
		if(mLastUpdateTimeCode == time_code) return true;
		
		pxr::UsdGeomPointBased mesh(prim_handle.getPrim());
		auto attr = mesh.GetPointsAttr();
		if (!attr.ValueMightBeTimeVarying()) return true;
	}

	// Curve live point positions
	if(!prim_handle.getPoints(mLiveCurvePoints, time_code)) {
		LOG_ERR << "Error getting curves point positions from " << prim_handle << " !";
		return false;
	}

	if(mLiveCurvePoints.size() != mRestCurvePoints.size()) {
		LOG_ERR << prim_handle.getPath() << " \"rest\" and \"live\" curves point positions count (" << mRestCurvePoints.size() << " vs " << mLiveCurvePoints.size() << " ) mismatch !";
		return false;
	}

	mLastUpdateTimeCode = time_code;
	return true;
}

const pxr::GfVec3f& GuideCurvesContainer::getGuideRestPoint(uint32_t guide_id, uint32_t vertex_id) const {
	assert(guide_id < mCurveVertexCounts.size() && guide_id < mCurveOffsets.size());
	assert(vertex_id < mCurveVertexCounts[guide_id]);

	const size_t global_vtx_id = mCurveOffsets[guide_id] + vertex_id;
	const auto& rest_points = mRestCurvePoints.AsConst();
	assert(global_vtx_id < rest_points.size());

	return rest_points[global_vtx_id];
}

const pxr::GfVec3f& GuideCurvesContainer::getGuideLivePoint(uint32_t guide_id, uint32_t vertex_id) const {
	assert(guide_id < mCurveVertexCounts.size() && guide_id < mCurveOffsets.size());
	assert(vertex_id < mCurveVertexCounts[guide_id]);

	const size_t global_vtx_id = mCurveOffsets[guide_id] + vertex_id;
	const auto& live_points = mRestCurvePoints.AsConst();
	assert(global_vtx_id < live_points.size());

	return live_points[global_vtx_id];
}

} // namespace Piston
