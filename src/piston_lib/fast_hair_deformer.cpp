#include "fast_hair_deformer.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <random>


namespace Piston {

FastHairDeformer::FastHairDeformer(): BaseHairDeformer() {
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
	PxrCurvesContainer* pCurves = mpCurvesContainer.get();

	pxr::VtArray<pxr::GfVec3f> points(pCurves->getTotalVertexCount());

	assert(mCurveBinds.size() == pCurves->getCurvesCount());

	for(uint32_t i = 0; i < pCurves->getCurvesCount(); ++i) {
		const auto& bind = mCurveBinds[i];
		if(bind.face_id == CurveBindData::kInvalidFaceID) continue;

		const auto& face = mpPhantomTrimesh->getFace(bind.face_id);
		const pxr::GfMatrix3f rotate_mat = rotateAlign(face.getRestNormal(), face.getLiveNormal());

		// try to move curve root point
		auto root_pt_pos = mpPhantomTrimesh->getInterpolatedPosition(bind.face_id, bind.u, bind.v, bind.dist);
		uint32_t vertex_offset = pCurves->getCurveVertexOffset(i);
		points[vertex_offset++] = root_pt_pos;
		PxrCurvesContainer::CurveDataPtr curve_data_ptr = pCurves->getCurveDataPtr(i);

		for(size_t j = 1; j < curve_data_ptr.first; ++j) {
			//points[offset++] = root_pt_pos + (rotate_mat * (*(curve_data_ptr.second + j)));
			points[vertex_offset++] = root_pt_pos + (*(curve_data_ptr.second + j));
		}
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
	if(!mpPhantomTrimesh) {
		printf("Error creating phantom trimesh !\n");
		return false;
	}

	return buildCurvesBindingData(reference_time_code);
}

bool FastHairDeformer::buildCurvesBindingData(pxr::UsdTimeCode reference_time_code) {
	if(!mpPhantomTrimesh) return false;
	if(!mpCurvesContainer) return false;

	// Build kdtree
	static const bool threaded_kdtree_creation = false;
	neighbour_search::KDTree<float, 3> kdtree(mpPhantomTrimesh->getRestPositions(), threaded_kdtree_creation);

	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> nearest_points;
	neighbour_search::KDTree<float, 3>::ReturnType nearest_point;

	mCurveBinds.resize(mpCurvesContainer->getCurvesCount());

	for(size_t i = 0; i < mpCurvesContainer->getCurvesCount(); ++i) {

		const pxr::GfVec3f curve_root_pt = mpCurvesContainer->getCurveRootPoint(i);

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