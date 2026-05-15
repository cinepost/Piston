#include "curves_container.h"
#include "logging.h"

#include <omp.h>

#include <limits>

namespace Piston {

PxrCurvesContainer::PxrCurvesContainer(): mCurvesCount(0), mLastUpdateTimeCode(std::numeric_limits<double>::lowest()) {

}

PxrCurvesContainer::PxrCurvesContainer(PxrCurvesContainer& other) {
	mCurvesCount = other.mCurvesCount;
	mCurveOffsets = other.mCurveOffsets;
	mCurveRootPositions = other.mCurveRootPositions;
	mCurveVectors = other.mCurveVectors;
	mLastUpdateTimeCode = other.mLastUpdateTimeCode;
}

bool PxrCurvesContainer::init(const UsdPrimHandle& prim_handle, const std::string& rest_attr_name, pxr::UsdTimeCode rest_time_code) {
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

	// Curves. points
	pxr::VtArray<pxr::GfVec3f> curve_points;

	if(rest_attr_name.empty() || !prim_handle.fetchAttributeValues<pxr::GfVec3f>(rest_attr_name, curve_points, rest_time_code)) {
		if(!prim_handle.getPoints(curve_points, rest_time_code)) {
			LOG_ERR << "Error getting curves points from " << prim_handle.getName() << " !";
			return false;
		}
	}

	// Curves. Counts/offsets
	if(!geom_curves.GetCurveVertexCountsAttr().Get(&mCurveVertexCounts, rest_time_code)){
		LOG_ERR << "Error getting curves vertices counts from " << prim_handle.getName() << "  !";
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

	#pragma omp parallel for num_threads(2) schedule(static)
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveRootPositions[i] = curve_points[mCurveOffsets[i]];
		
		mCurveVectors[mCurveOffsets[i]] = {0.f, 0.f, 0.f};

		for(size_t j = 1; j < mCurveVertexCounts[i]; ++j) {
			mCurveVectors[mCurveOffsets[i] + j] = curve_points[mCurveOffsets[i] + j] - mCurveRootPositions[i];
		}
	}

	assert(mCurveVertexCounts.size() == mCurveOffsets.size());

	mLastUpdateTimeCode = rest_time_code;

	return true;
}

bool PxrCurvesContainer::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force) {
	assert(prim_handle.isBasisCurvesGeoPrim());

	if(!force) {
		if(mLastUpdateTimeCode == time_code) return true;

		if (!pxr::UsdGeomPointBased(prim_handle.getPrim()).GetPointsAttr().ValueMightBeTimeVarying()) return true;
	}

	// Curve live point positions
	pxr::VtArray<pxr::GfVec3f> points;
	if(!prim_handle.getPoints(points, time_code)) {
		LOG_ERR << "Error getting curves point positions from " << prim_handle << " !";
		return false;
	}

	assert(points.size() == getTotalVertexCount());

	if(points.size() != getTotalVertexCount()) {
		LOG_ERR << prim_handle.getPath() << " curve point positions count ( old " << getTotalVertexCount() << ", new " << points.size() << " ) mismatch !";
		return false;
	}

	// Calc curves derivs
	#pragma omp parallel for num_threads(2) schedule(static)
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveRootPositions[i] = points[mCurveOffsets[i]];
		
		mCurveVectors[mCurveOffsets[i]] = {0.f, 0.f, 0.f};

		for(size_t j = 1; j < mCurveVertexCounts[i]; ++j) {
			mCurveVectors[mCurveOffsets[i] + j] = points[mCurveOffsets[i] + j] - mCurveRootPositions[i];
		}
	}

	mLastUpdateTimeCode = time_code;

	LOG_TRC << "PxrCurvesContainer updated at " << mLastUpdateTimeCode;

	return true;
}

PxrCurvesContainer::UniquePtr PxrCurvesContainer::create() {
	return PxrCurvesContainer::UniquePtr(new PxrCurvesContainer());
}

PxrCurvesContainer::UniquePtr PxrCurvesContainer::create(const UsdPrimHandle& prim_handle, const std::string& rest_attr_name, pxr::UsdTimeCode rest_time_code) {
	PxrCurvesContainer::UniquePtr pResult = PxrCurvesContainer::UniquePtr(new PxrCurvesContainer());
	if(!pResult->init(prim_handle, rest_attr_name, rest_time_code)) return nullptr;

	return pResult;
}

PxrCurvesContainer::CurveDataPtr PxrCurvesContainer::getCurveDataPtr(size_t curve_idx) {
	assert(curve_idx < mCurveOffsets.size());

	return {mCurveVertexCounts[curve_idx], &mCurveVectors[mCurveOffsets[curve_idx]]};
}

PxrCurvesContainer::CurveDataConstPtr PxrCurvesContainer::getCurveDataPtr(size_t curve_idx) const {
	assert(curve_idx < mCurveOffsets.size());

	return {mCurveVertexCounts[curve_idx], &mCurveVectors[mCurveOffsets[curve_idx]]};
}

const pxr::GfVec3f& PxrCurvesContainer::getCurveRootPoint(size_t curve_idx) const {
	assert(curve_idx < mCurveRootPositions.size());

	return mCurveRootPositions[curve_idx];
}

} // namespace Piston
