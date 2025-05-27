#include "fast_hair_deformer.h"
#include "kdtree.hpp"

#include <random>


namespace Piston {

FastHairDeformer::FastHairDeformer(): BaseHairDeformer(), mCurvesCount(0) {
	printf("FastHairDeformer::FastHairDeformer()\n");
}

FastHairDeformer::SharedPtr FastHairDeformer::create() {
	return SharedPtr(new FastHairDeformer());
}

const std::string& FastHairDeformer::toString() const {
	static const std::string kFastDeformerString = "FastHairDeformer";
	return kFastDeformerString;
}

bool FastHairDeformer::deformImpl(pxr::UsdTimeCode time_code) {
	printf("FastHairDeformer::deformImpl()\n");
	if(!mpPhantomTrimesh) return false;

	if(!mpPhantomTrimesh->update(mMeshGeoPrimHandle, time_code)) {
		return false;
	}

	pxr::UsdGeomCurves curves(mHairGeoPrimHandle.getPrim());
	pxr::VtArray<pxr::GfVec3f> points = mCurveRefPoints;

	assert(mCurveBinds.size() == mCurvesCount);

	for(uint32_t i = 0; i < mCurvesCount; ++i) {
		const auto& bind = mCurveBinds[i];
		if(bind.face_id == CurveBindData::kInvalidFaceID) continue;

		auto root_orig = points[mCurveOffsets[i]];

		// try to move curve root point
		auto root_pt_pos = mpPhantomTrimesh->getInterpolatedPosition(bind.face_id, bind.u, bind.v, bind.dist);
		points[mCurveOffsets[i]] = root_pt_pos;

		const pxr::GfVec3f translate = root_pt_pos - mCurveRefPoints[mCurveOffsets[i]];
		for(size_t j = 1; j < mCurveVertexCounts[i]; ++j) {
			points[mCurveOffsets[i] + j] += translate;
		}

		//points[mCurveOffsets[i]] = root_orig;

		//printf("Deformed curve %d\n", i);
		//printf("Deformed curve dist %4.4f\n", bind.dist);
	}

	if(!curves.GetPointsAttr().Set(points, time_code)) {
		return false;
	}

	return true;
}

bool FastHairDeformer::buildDeformerData(pxr::UsdTimeCode reference_time_code) {
	printf("FastHairDeformer::buildDeformerData()\n");

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = mMeshGeoPrimHandle.getPrimvarsAPI();

	pxr::UsdGeomMesh mesh(mMeshGeoPrimHandle.getPrim());

	// Create adjacency data
	mAdjacency = UsdGeomMeshFaceAdjacency::create(mesh);

	// Create phantom mesh
	mpPhantomTrimesh = PhantomTrimesh<PxrIndexType>::create(mMeshGeoPrimHandle, mRestPositionAttrName);
	if(!mpPhantomTrimesh) return false;

	auto geom_hair = pxr::UsdGeomCurves(mHairGeoPrimHandle.getPrim());
	if(!geom_hair) {
		printf("Error getting hair geometry !\n");
		return false;
	}

	// Curves information
	mCurvesCount = geom_hair.GetCurveCount(reference_time_code);
	mCurveBinds.resize(mCurvesCount);
	mCurveOffsets.resize(mCurvesCount);

	// Curves. points
	if(!geom_hair.GetPointsAttr().Get(&mCurveRefPoints, reference_time_code)) {
		printf("Error getting curve points !\n");
		return false;
	}

	// Curves. Counts/offsets
	if(!geom_hair.GetCurveVertexCountsAttr().Get(&mCurveVertexCounts, reference_time_code)){
		printf("Error getting curve vertices counts !\n");
		return false;
	}

	assert(mCurvesCount == mCurveVertexCounts.size());

	uint32_t offset = 0u;
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveOffsets[i] = offset;
		offset += mCurveVertexCounts[i];
	}

	return buildCurvesBindingData(reference_time_code);
}

bool FastHairDeformer::buildCurvesBindingData(pxr::UsdTimeCode reference_time_code) {
	if(!mpPhantomTrimesh) return false;

	// Build kdtree
	static const bool threaded_kdtree_creation = false;
	neighbour_search::KDTree<float, 3> kdtree(mpPhantomTrimesh->getRestPositions(), threaded_kdtree_creation);

	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> nearest_points;
	neighbour_search::KDTree<float, 3>::ReturnType nearest_point;

	for(uint32_t i = 0; i < mCurvesCount; ++i) {

		const pxr::GfVec3f curve_root_pt = mCurveRefPoints[mCurveOffsets[i]];

		kdtree.findKNearestNeighbours(curve_root_pt, 3, nearest_points);
		nearest_point = kdtree.findNearestNeighbour(curve_root_pt);

		if(nearest_points.size() == 3) {
			const uint32_t face_id = mpPhantomTrimesh->getOrCreate(
				static_cast<PxrIndexType>(nearest_points[0].first), static_cast<PxrIndexType>(nearest_points[1].first), static_cast<PxrIndexType>(nearest_points[2].first));
		
			auto& bind = mCurveBinds[i];

			bool projected = mpPhantomTrimesh->projectPoint(curve_root_pt, face_id, bind.u, bind.v, bind.dist);
			if(projected) {
				printf("projected\n");
				bind.face_id = face_id;
			} else {
				printf("skipped\n");
				bind.face_id = CurveBindData::kInvalidFaceID;
			}
		}
	}
	
	return true;
}

} // namespace Piston