#include "phantom_trimesh.h"
#include "geometry_tools.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kTrimeshDataVersion( 0u, 0u, 0u);

template<typename IndexType>
typename PhantomTrimesh<IndexType>::UniquePtr PhantomTrimesh<IndexType>::create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	assert(prim_handle.isMeshGeoPrim());

	if(!prim_handle.isMeshGeoPrim()) return nullptr;

	typename PhantomTrimesh<IndexType>::UniquePtr pPhantomTrimesh = PhantomTrimesh<IndexType>::UniquePtr( new PhantomTrimesh<IndexType>());

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

template<typename IndexType>
size_t PhantomTrimesh<IndexType>::calcHash() const {
	size_t hash = 0;

	for(const auto& pos: mUsdMeshRestPositions) {
		hash += size_t(pos[0]) + size_t(pos[1]*10.f) + size_t(pos[2]*100.f);
	}
	hash += mUsdMeshRestPositions.size();

	for(const auto& face: mFaces) {
		hash += face.calcHash();
	}
	hash += mFaces.size();

	for( const auto& [k, v]: mFaceMap) {
		hash += k[0] + k[1]*2 + k[2]*3 + v * 1000;
	}
	hash += mFaceMap.size();

	return hash;
}

// Specialisation
template PhantomTrimesh<int>::UniquePtr PhantomTrimesh<int>::create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code); 

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


static constexpr const char* kJUsdRestPositions = "rest_pos";
static constexpr const char* kJFaces = "faces";
static constexpr const char* kJaceMap = "face_map";
static constexpr const char* kJDataHash = "hash";

template<typename IndexType>
bool SerializablePhantomTrimesh<IndexType>::buildInPlace(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	clearData();

	mpTrimesh = PhantomTrimesh<IndexType>::create(prim_handle, rest_p_name);

	if(!mpTrimesh) {
		setPopulated(false);
		return false;
	}

	setPopulated(true);
	return true;
}

template<typename IndexType>
void SerializablePhantomTrimesh<IndexType>::clearData() {
	mpTrimesh = nullptr;
	setPopulated(false);
}

template<typename IndexType>
bool SerializablePhantomTrimesh<IndexType>::dumpToJSON(json& j) const {
	if(!mpTrimesh || !mpTrimesh->isValid()) return false;

	j[kJUsdRestPositions] = mpTrimesh->mUsdMeshRestPositions;
	j[kJFaces] = mpTrimesh->mFaces;
	j[kJaceMap] = mpTrimesh->mFaceMap;

	j[kJDataHash] = mpTrimesh->calcHash();

	return true;
}

template<typename IndexType>
bool SerializablePhantomTrimesh<IndexType>::readFromJSON(const json& j) {
	mpTrimesh = std::make_unique<PhantomTrimesh<IndexType>>();
	mpTrimesh->mValid = false;

	mpTrimesh->mUsdMeshRestPositions = j[kJUsdRestPositions];
	mpTrimesh->mFaces = j[kJFaces];
	mpTrimesh->mFaceMap = j[kJaceMap].template get<std::unordered_map<std::array<IndexType, 3>, size_t>>();

	const size_t json_trimesh_data_hash = j[kJDataHash];
	const size_t calc_trimesh_data_hash = mpTrimesh->calcHash();

	if(calc_trimesh_data_hash != json_trimesh_data_hash) {
		return false;
	}

	mpTrimesh->mValid = true;
	return true;
}

template<typename IndexType>
const PhantomTrimesh<IndexType>* SerializablePhantomTrimesh<IndexType>::getTrimesh() const {
	if(!isPopulated()) return nullptr;

	return mpTrimesh.get();
}

template<typename IndexType>
const std::string& SerializablePhantomTrimesh<IndexType>::typeName() const {
	static const std::string kTypeName = "SerializablePhantomTrimesh";
	return kTypeName;
}

template<typename IndexType>
const std::string& SerializablePhantomTrimesh<IndexType>::jsonDataKey() const {
	static const std::string kDataKey = "_piston_trimesh_data_";
	return kDataKey;
}

template<typename IndexType>
const SerializableDeformerDataBase::DataVersion& SerializablePhantomTrimesh<IndexType>::jsonDataVersion() const {
	return kTrimeshDataVersion;
}


} // namespace Piston
