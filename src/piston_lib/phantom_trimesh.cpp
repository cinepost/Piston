#include "phantom_trimesh.h"
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
		std::cerr << "No valid primvar \"" << rest_p_name << "\" exists in mesh " << prim_handle.getPath() << " !" << std::endl;
		return nullptr;
	}

	const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
	if(!restPosAttr.Get(&pPhantomTrimesh->mUsdMeshRestPositions, time_code)) {
		std::cerr << "Error getting mesh " << prim_handle.getPath() << " \"rest\" positions !" << std::endl;
		return nullptr;
	}

	pPhantomTrimesh->mValid = true;
	return pPhantomTrimesh;
}

template<typename IndexType>
uint32_t PhantomTrimesh<IndexType>::getOrCreate(IndexType a, IndexType b, IndexType c) {	
	std::array<IndexType, 3> indices{a, b, c};
	std::sort(indices.begin(), indices.end());

	auto it = mFaceMap.find(indices);
	if(it != mFaceMap.end()) {
		return it->second;
	}

	const uint32_t idx = static_cast<uint32_t>(mFaces.size());

	mFaces.emplace_back(indices);
	mFaces.back().restNormal = pxr::GfGetNormalized(
		pxr::GfCross(mUsdMeshRestPositions[indices[1]] - mUsdMeshRestPositions[indices[0]], mUsdMeshRestPositions[indices[2]] - mUsdMeshRestPositions[indices[0]])
	);

	mFaceMap[indices] = idx;

	return idx;
}

template<typename IndexType>
bool PhantomTrimesh<IndexType>::projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;
	
	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return rayTriangleIntersect(pt, -face.getRestNormal(), mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], u, v);
}

template<typename IndexType>
bool PhantomTrimesh<IndexType>::projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v, float& dist) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;
	
	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return rayTriangleIntersect(pt, -face.getRestNormal(), mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], dist, u, v);
}

template<typename IndexType>
bool PhantomTrimesh<IndexType>::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, uint32_t face_id, float& u, float& v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;

	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return rayTriangleIntersect(orig, dir, mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], u, v);
}

template<typename IndexType>
bool PhantomTrimesh<IndexType>::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, uint32_t face_id, float& u, float& v, float& dist) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;

	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return rayTriangleIntersect(orig, dir, mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], dist, u, v);
}

template<typename IndexType>
pxr::GfVec3f PhantomTrimesh<IndexType>::getInterpolatedRestPosition(uint32_t face_id, float u, float v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	return u * mUsdMeshRestPositions[face.indices[1]] + v * mUsdMeshRestPositions[face.indices[2]] + (1.f - u - v) * mUsdMeshRestPositions[face.indices[0]];
};


template<typename IndexType>
pxr::GfVec3f PhantomTrimesh<IndexType>::getInterpolatedLivePosition(uint32_t face_id, float u, float v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	return u * mUsdMeshLivePositions[face.indices[1]] + v * mUsdMeshLivePositions[face.indices[2]] + (1.f - u - v) * mUsdMeshLivePositions[face.indices[0]];
};

template<typename IndexType>
pxr::GfVec3f PhantomTrimesh<IndexType>::getFaceRestCentroid(uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	static constexpr float kInvThree = 1.f / 3.f;
	return (mUsdMeshRestPositions[face.indices[0]] + mUsdMeshRestPositions[face.indices[1]] + mUsdMeshRestPositions[face.indices[2]]) * kInvThree;
}

template<typename IndexType>
bool PhantomTrimesh<IndexType>::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) {
	assert(prim_handle.isMeshGeoPrim());

	pxr::UsdGeomMesh mesh(prim_handle.getPrim());

	if(!mesh.GetPointsAttr().Get(&mUsdMeshLivePositions, time_code)) {
		std::cerr << "Error getting point positions from " << prim_handle.getPath() << " !" << std::endl;
		return false;
	}

	if(mUsdMeshLivePositions.size() != mUsdMeshRestPositions.size()) {
		std::cerr << prim_handle.getPath() << " \"rest\" and live mesh point positions count mismatch!" << std::endl;
		return false;
	}

	return true;
}

template<typename IndexType>
const pxr::GfVec3f& PhantomTrimesh<IndexType>::getFaceRestNormal(uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	return mFaces[face_id].getRestNormal();
}

template<typename IndexType>
pxr::GfVec3f PhantomTrimesh<IndexType>::getFaceLiveNormal(uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& indices = mFaces[face_id].indices;
	return pxr::GfGetNormalized(
		pxr::GfCross(mUsdMeshLivePositions[indices[1]] - mUsdMeshLivePositions[indices[0]], mUsdMeshLivePositions[indices[2]] - mUsdMeshLivePositions[indices[0]])
	);
}

// Specialisation
template PhantomTrimesh<int>::SharedPtr PhantomTrimesh<int>::create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code); 

template uint32_t PhantomTrimesh<int>::getOrCreate(int a, int b, int c);

template bool PhantomTrimesh<int>::projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v) const;
template bool PhantomTrimesh<int>::projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v, float& dist) const;

template bool PhantomTrimesh<int>::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& vec, uint32_t face_id, float& u, float& v) const;
template bool PhantomTrimesh<int>::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& vec, uint32_t face_id, float& u, float& v, float& dist) const;

template pxr::GfVec3f PhantomTrimesh<int>::getInterpolatedRestPosition(uint32_t face_id, float u, float v) const;
template pxr::GfVec3f PhantomTrimesh<int>::getInterpolatedLivePosition(uint32_t face_id, float u, float v) const;
template pxr::GfVec3f PhantomTrimesh<int>::getFaceRestCentroid(uint32_t face_id) const;

template const pxr::GfVec3f& PhantomTrimesh<int>::getFaceRestNormal(uint32_t face_id) const;
template pxr::GfVec3f PhantomTrimesh<int>::getFaceLiveNormal(uint32_t face_id) const;

template bool PhantomTrimesh<int>::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code);
} // namespace Piston
