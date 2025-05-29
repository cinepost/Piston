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
uint32_t PhantomTrimesh<IndexType>::getOrCreate(IndexType a, IndexType b, IndexType c) {	
	auto it = mFaceMap.find({a, b, c});
	if(it != mFaceMap.end()) {
		return it->second;
	}

	const uint32_t idx = static_cast<uint32_t>(mFaces.size());

	mFaces.emplace_back(a, b, c);
	mFaces.back().restNormal = pxr::GfGetNormalized(pxr::GfCross(mUsdMeshRestPositions[b] - mUsdMeshRestPositions[a], mUsdMeshRestPositions[c] - mUsdMeshRestPositions[a]));

	return idx;
}

template<typename IndexType>
bool PhantomTrimesh<IndexType>::projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v, float& dist) const {
	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return pointTriangleProject(pt, face.getRestNormal(), mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], dist, u, v);
}

template<typename IndexType>
pxr::GfVec3f PhantomTrimesh<IndexType>::getInterpolatedPosition(uint32_t face_id, float u, float v, float dist) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	return (u * mUsdMeshLivePositions[face.indices[1]] + v * mUsdMeshLivePositions[face.indices[2]] + (1.f - u - v) * mUsdMeshLivePositions[face.indices[0]]) + face.getLiveNormal() * dist;// + face.getLiveNormal();
};

template<typename IndexType>
bool PhantomTrimesh<IndexType>::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) {
	assert(prim_handle.isMeshGeoPrim());

	pxr::UsdGeomMesh mesh(prim_handle.getPrim());

	if(!mesh.GetPointsAttr().Get(&mUsdMeshLivePositions, time_code)) {
		printf("Error getting point positions!\n");
		return false;
	}

	if(mUsdMeshLivePositions.size() != mUsdMeshRestPositions.size()) {
		printf("Rest and live mesh point positions count mismatch!\n");
		return false;
	}

	for(auto& face: mFaces) {
		face.liveNormal = pxr::GfGetNormalized(
			pxr::GfCross(mUsdMeshLivePositions[face.indices[1]] - mUsdMeshLivePositions[face.indices[0]], mUsdMeshLivePositions[face.indices[2]] - mUsdMeshLivePositions[face.indices[0]])
		);
	}

	return true;
}

// Specialisation
template PhantomTrimesh<int>::SharedPtr PhantomTrimesh<int>::create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code); 

template uint32_t PhantomTrimesh<int>::getOrCreate(int a, int b, int c);

template bool PhantomTrimesh<int>::projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v, float& dist) const;

template pxr::GfVec3f PhantomTrimesh<int>::getInterpolatedPosition(uint32_t face_id, float u, float v, float dist) const;

template bool PhantomTrimesh<int>::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code);
} // namespace Piston
