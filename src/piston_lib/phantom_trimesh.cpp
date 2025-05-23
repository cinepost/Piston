#include "phantom_trimesh.h"
#include "base_hair_deformer.h"
#include "geometry_tools.h"

namespace Piston {

template<typename IndexType>
typename PhantomTrimesh<IndexType>::SharedPtr PhantomTrimesh<IndexType>::create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	assert(prim_handle.isMeshGeoPrim());

	typename PhantomTrimesh<IndexType>::SharedPtr pPhantomTrimesh = PhantomTrimesh<IndexType>::SharedPtr( new PhantomTrimesh<IndexType>());

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomMesh mesh(prim_handle.getPrim());

	// Keep rest position for further calculations
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(rest_p_name));
	if(!restPositionPrimVar) {
		printf("No valid primvar \"%s\" exists in mesh !\n", rest_p_name.c_str());
		return nullptr;
	}

	const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
	if(!restPosAttr.Get(&pPhantomTrimesh->mUsdMeshRestPositions, time_code)) {
		printf("Error getting mesh rest positions !\n");
		return nullptr;
	}

	pPhantomTrimesh->mValid = true;
	return pPhantomTrimesh;
}

template<typename IndexType>
size_t PhantomTrimesh<IndexType>::getOrCreate(IndexType a, IndexType b, IndexType c) {	
	auto it = mFaceMap.find({a, b, c});
	if(it != mFaceMap.end()) {
		return it->second;
	}

	const size_t idx = mFaces.size();

	mFaces.push_back({a, b, c});
	mFaces.back().restNormal = pxr::GfGetNormalized(pxr::GfCross(mUsdMeshRestPositions[b] - mUsdMeshRestPositions[a], mUsdMeshRestPositions[c] - mUsdMeshRestPositions[a]));

	return idx;
}

template<typename IndexType>
bool PhantomTrimesh<IndexType>::projectPoint(const pxr::GfVec3f& pt, size_t triface_id) const {
	const TriFace& face = mFaces[triface_id];
	float t, u, v;
	return pointTriangleProject(pt, face.getRestNormal(), mUsdMeshRestPositions[face[0]], mUsdMeshRestPositions[face[1]], mUsdMeshRestPositions[face[2]], t, u, v);
}

// Specialisation
template PhantomTrimesh<int>::SharedPtr PhantomTrimesh<int>::create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code); 

template size_t PhantomTrimesh<int>::getOrCreate(int a, int b, int c);

template bool PhantomTrimesh<int>::projectPoint(const pxr::GfVec3f& pt, size_t triface_id) const;

} // namespace Piston
