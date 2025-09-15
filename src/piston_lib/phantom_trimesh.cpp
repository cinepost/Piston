#include "pxr_json.h"
#include "phantom_trimesh.h"
#include "geometry_tools.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kTrimeshDataVersion( 0u, 0u, 0u);

PhantomTrimesh::UniquePtr PhantomTrimesh::create() {
	return std::make_unique<PhantomTrimesh>();
}

bool PhantomTrimesh::init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	mValid = false;

	assert(prim_handle.isMeshGeoPrim());

	if(!prim_handle.isMeshGeoPrim()) return false;

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomMesh mesh(prim_handle.getPrim());

	// Keep rest position for further calculations
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(rest_p_name));
	if(!restPositionPrimVar) {
		std::cerr << "No valid primvar \"" << rest_p_name << "\" exists in mesh " << prim_handle.getPath() << " !" << std::endl;
		return false;
	}

	const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
	if(!restPosAttr.Get(&mUsdMeshRestPositions, time_code)) {
		std::cerr << "Error getting mesh " << prim_handle.getPath() << " \"rest\" positions !" << std::endl;
		return false;
	}

	mValid = true;
	return mValid;
}

void PhantomTrimesh::invalidate() {
	mValid = false;

	mUsdMeshRestPositions.clear();
	mUsdMeshLivePositions.clear();

	mFaceMap.clear();
	mFaces.clear();
}

uint32_t PhantomTrimesh::getOrCreate(PhantomTrimesh::PxrIndexType a, PhantomTrimesh::PxrIndexType b, PhantomTrimesh::PxrIndexType c) const {	
	std::array<PhantomTrimesh::PxrIndexType, 3> indices{a, b, c};
	std::sort(indices.begin(), indices.end());

	auto it = mFaceMap.find(indices);
	if(it != mFaceMap.end()) {
		return static_cast<uint32_t>(it->second);
	}

	const uint32_t idx = static_cast<uint32_t>(mFaces.size());

	mFaces.emplace_back(indices);
	mFaces.back().restNormal = pxr::GfGetNormalized(
		pxr::GfCross(mUsdMeshRestPositions[indices[1]] - mUsdMeshRestPositions[indices[0]], mUsdMeshRestPositions[indices[2]] - mUsdMeshRestPositions[indices[0]])
	);

	mFaceMap[indices] = idx;

	return idx;
}

bool PhantomTrimesh::projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;
	
	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return rayTriangleIntersect(pt, -face.getRestNormal(), mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], u, v);
}

bool PhantomTrimesh::projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v, float& dist) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;
	
	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return rayTriangleIntersect(pt, -face.getRestNormal(), mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], dist, u, v);
}

bool PhantomTrimesh::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, uint32_t face_id, float& u, float& v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;

	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return rayTriangleIntersect(orig, dir, mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], u, v);
}

bool PhantomTrimesh::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, uint32_t face_id, float& u, float& v, float& dist) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;

	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	return rayTriangleIntersect(orig, dir, mUsdMeshRestPositions[face.indices[0]], mUsdMeshRestPositions[face.indices[1]], mUsdMeshRestPositions[face.indices[2]], dist, u, v);
}

pxr::GfVec3f PhantomTrimesh::getInterpolatedRestPosition(uint32_t face_id, float u, float v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	return u * mUsdMeshRestPositions[face.indices[1]] + v * mUsdMeshRestPositions[face.indices[2]] + (1.f - u - v) * mUsdMeshRestPositions[face.indices[0]];
};


pxr::GfVec3f PhantomTrimesh::getInterpolatedLivePosition(uint32_t face_id, float u, float v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	return u * mUsdMeshLivePositions[face.indices[1]] + v * mUsdMeshLivePositions[face.indices[2]] + (1.f - u - v) * mUsdMeshLivePositions[face.indices[0]];
};

pxr::GfVec3f PhantomTrimesh::getFaceRestCentroid(uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	static constexpr float kInvThree = 1.f / 3.f;
	return (mUsdMeshRestPositions[face.indices[0]] + mUsdMeshRestPositions[face.indices[1]] + mUsdMeshRestPositions[face.indices[2]]) * kInvThree;
}

bool PhantomTrimesh::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) const {
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

const pxr::GfVec3f& PhantomTrimesh::getFaceRestNormal(uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	return mFaces[face_id].getRestNormal();
}

pxr::GfVec3f PhantomTrimesh::getFaceLiveNormal(uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& indices = mFaces[face_id].indices;
	return pxr::GfGetNormalized(
		pxr::GfCross(mUsdMeshLivePositions[indices[1]] - mUsdMeshLivePositions[indices[0]], mUsdMeshLivePositions[indices[2]] - mUsdMeshLivePositions[indices[0]])
	);
}

size_t PhantomTrimesh::calcHash() const {
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

void to_json(json& j, const PhantomTrimesh::TriFace& face) {
	j["indices"] = face.indices;
	to_json(j["normal"], face.restNormal);
}

void from_json(const json& j, PhantomTrimesh::TriFace& face) {
	j.at("indices").get_to(face.indices);
	from_json(j["normal"], face.restNormal);
}


static constexpr const char* kJUsdRestPositions = "rest_pos";
static constexpr const char* kJFaces = "faces";
static constexpr const char* kJaceMap = "face_map";
static constexpr const char* kJDataHash = "hash";

SerializablePhantomTrimesh::SerializablePhantomTrimesh() {
	mpTrimesh = PhantomTrimesh::create();
}

bool SerializablePhantomTrimesh::buildInPlace(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	assert(prim_handle.isMeshGeoPrim());
	clearData();

	if(!mpTrimesh) {
		mpTrimesh = PhantomTrimesh::create();
	}

	bool result = mpTrimesh->init(prim_handle, rest_p_name);

	if(!result) {
		mpTrimesh->invalidate();
	}

	setPopulated(result);
	return result;
}

void SerializablePhantomTrimesh::clearData() {
	setPopulated(false); 
	if(mpTrimesh) {
		mpTrimesh->invalidate();
	}
}

bool SerializablePhantomTrimesh::dumpToJSON(json& j) const {
	if(!mpTrimesh || !mpTrimesh->isValid()) return false;

	to_json(j[kJUsdRestPositions], mpTrimesh->mUsdMeshRestPositions);

	j[kJFaces] = mpTrimesh->mFaces;
	j[kJaceMap] = mpTrimesh->mFaceMap;

	j[kJDataHash] = mpTrimesh->calcHash();

	return true;
}

bool SerializablePhantomTrimesh::readFromJSON(const json& j) {
	mpTrimesh = std::make_unique<PhantomTrimesh>();
	mpTrimesh->mValid = false;

	from_json(j[kJUsdRestPositions], mpTrimesh->mUsdMeshRestPositions);
	mpTrimesh->mFaces = j[kJFaces].template get<std::vector<PhantomTrimesh::TriFace>>();
	mpTrimesh->mFaceMap = j[kJaceMap].template get<std::unordered_map<std::array<PhantomTrimesh::PxrIndexType, 3>, size_t, IndicesArrayHasher<PhantomTrimesh::PxrIndexType, 3>>>();

	const size_t json_trimesh_data_hash = j[kJDataHash];
	const size_t calc_trimesh_data_hash = mpTrimesh->calcHash();

	if(calc_trimesh_data_hash != json_trimesh_data_hash) {
		return false;
	}

	mpTrimesh->mValid = true;

	dbg_printf("PhantomTrimesh data read from json payload !\n");

	return true;
}

const PhantomTrimesh* SerializablePhantomTrimesh::getTrimesh() const {
	if(!isPopulated()) return nullptr;

	return mpTrimesh.get();
}

const std::string& SerializablePhantomTrimesh::typeName() const {
	static const std::string kTypeName = "SerializablePhantomTrimesh";
	return kTypeName;
}

const std::string& SerializablePhantomTrimesh::jsonDataKey() const {
	static const std::string kDataKey = "piston_trimesh_data";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& SerializablePhantomTrimesh::jsonDataVersion() const {
	return kTrimeshDataVersion;
}

} // namespace Piston
