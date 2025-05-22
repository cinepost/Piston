#include "fast_hair_deformer.h"
#include "kdtree.hpp"


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

	pxr::UsdGeomMesh mesh(mMeshGeoPrimHandle.getPrim());

	pxr::VtArray<pxr::GfVec3f> points; // TODO: make member

	if(!mesh.GetPointsAttr().Get(&points, time_code)) {
		return false;
	}

	return true;
}

bool FastHairDeformer::buildDeformerData() {
	printf("FastHairDeformer::buildDeformerData()\n");

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = mMeshGeoPrimHandle.getPrimvarsAPI();

	pxr::UsdGeomMesh mesh(mMeshGeoPrimHandle.getPrim());

	// Create adjacency data
	mAdjacency = UsdGeomMeshFaceAdjacency::create(mesh);

	// Create phantom mesh
	mpPhantomTrimesh = PhantomTrimesh<PxrIndexType>::create(mMeshGeoPrimHandle, mRestPositionAttrName);

	// Build kdtree
	static const bool threaded_kdtree_creation = false;
	neighbour_search::KDTree<float, 3> kdtree(mpPhantomTrimesh->getRestPositions(), threaded_kdtree_creation);

	// Hair to mesh binding data
	// Iterate hair curves and test/bind root points
	{
		std::vector<neighbour_search::KDTree<float, 3>::ReturnType> nearest_points;
		neighbour_search::KDTree<float, 3>::PointType curve_root_point;

		kdtree.findKNearestNeighbours(curve_root_point, 3, nearest_points);

		if(nearest_points.size() == 3) {
			auto tri_face = mpPhantomTrimesh->getOrCreate(
				static_cast<PxrIndexType>(nearest_points[0].first), static_cast<PxrIndexType>(nearest_points[1].first), static_cast<PxrIndexType>(nearest_points[2].first));
		}
	}

	return true;
}

bool FastHairDeformer::buildHairToMeshBindingData() {
	if(!mMeshGeoPrimHandle || !mHairGeoPrimHandle) {
		printf("No mesh or hair UsdPrim is set !\n");
		return false;
	}

	pxr::UsdGeomMesh mesh(mMeshGeoPrimHandle.getPrim());

	return true;
}

} // namespace Piston