#include "pxr_json.h"
#include "phantom_trimesh.h"
#include "geometry_tools.h"
#include "simple_profiler.h"
#include "math.h"

#ifdef USE_CGAL

#define CGAL_DO_NOT_USE_BOOST_MP
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Triangulation_3.h>

#endif // USE_CGAL

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kTrimeshDataVersion( 0u, 0u, 0u);

PhantomTrimesh::UniquePtr PhantomTrimesh::create() {
	return std::make_unique<PhantomTrimesh>();
}

PhantomTrimesh::PhantomTrimesh() : mValid(false) {
	static const size_t reserve_size = 1024;

	mFaceMap.reserve(reserve_size);
	mFaces.reserve(reserve_size);
	mFaceFlags.reserve(reserve_size);
}

bool PhantomTrimesh::init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	mValid = false;

	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(rest_p_name));
		
	if(!restPositionPrimVar) {
		std::cerr << "No valid primvar \"" << rest_p_name << "\" exists in prim " << prim_handle.getPath() << " !" << std::endl;
		return false;
	}

	const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
	if(!restPosAttr.Get(&mUsdMeshRestPositions, time_code)) {
		std::cerr << "Error getting prim " << prim_handle.getPath() << " \"rest\" positions !" << std::endl;
		return false;
	}

	std::cout << "Prim " << prim_handle.getPath() << " has " << mUsdMeshRestPositions.size() << " rest positions" << std::endl;

	mValid = true;
	return mValid;
}

#ifdef USE_CGAL

bool PhantomTrimesh::buildTetrahedrons() {
	using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Vb =  CGAL::Triangulation_vertex_base_with_info_3<uint32_t, Kernel>;
	using Tds = CGAL::Triangulation_data_structure_3<Vb>;
	using Triangulation = CGAL::Triangulation_3<Kernel, Tds>;
	
	using CGAL_Point = Triangulation::Point;

	using CGAL_CellHandle = Triangulation::Cell_handle;
	using CGAL_VtxHandle = Triangulation::Vertex_handle;
	using CGAL_LocateType = Triangulation::Locate_type;

	const pxr::VtArray<pxr::GfVec3f>& rest_positions = getRestPositions();
	const size_t points_count = rest_positions.size();

	if(points_count < 4) {
		std::cerr << "Mesh has less than 4 vertices !" << std::endl;
		return false;
	}

	std::vector< std::pair<CGAL_Point, uint32_t> > points;
	for(size_t i = 0; i < points_count; ++i) {
		const auto& pxr_pt = rest_positions[i];
  		points.push_back( std::make_pair( CGAL_Point(pxr_pt[0], pxr_pt[1], pxr_pt[2]), (uint32_t)i ));
  	}

	Triangulation T(points.begin(), points.end());

	dbg_printf("T number of cells: %zu\n", (size_t)T.number_of_cells());
	dbg_printf("T number of finite cells: %zu\n", (size_t)T.number_of_finite_cells());

	mTetrahedrons.resize(T.number_of_finite_cells());

	size_t i = 0;
	for(const CGAL_CellHandle& cell_handle: T.finite_cell_handles()) {
		dbg_printf("cell %zu vertices: %d %d %d %d\n", i, cell_handle->vertex(0)->info(), cell_handle->vertex(1)->info(), cell_handle->vertex(2)->info(), cell_handle->vertex(3)->info());
		mTetrahedrons[i].indices[0] = cell_handle->vertex(0)->info();
		mTetrahedrons[i].indices[1] = cell_handle->vertex(1)->info();
		mTetrahedrons[i].indices[2] = cell_handle->vertex(2)->info();
		mTetrahedrons[i].indices[3] = cell_handle->vertex(3)->info();
		i++;
	}

	mTetrahedronCounts.resize(points_count);
	std::fill(mTetrahedronCounts.begin(), mTetrahedronCounts.end(), 0);
	mTetrahedronOffsets.resize(points_count);
	
	for(const Tetrahedron& tetra: mTetrahedrons) {
		mTetrahedronCounts[tetra.indices[0]]++;
		mTetrahedronCounts[tetra.indices[1]]++;
		mTetrahedronCounts[tetra.indices[2]]++;
		mTetrahedronCounts[tetra.indices[3]]++;
	}

	uint32_t offset = 0;
	for(size_t i = 0; i < mTetrahedronCounts.size(); ++i) {
		mTetrahedronOffsets[i] = offset;
		offset += mTetrahedronCounts[i];
	}

	mTetrahedronIndices.resize(mTetrahedrons.size() * 4);
	std::fill(mTetrahedronIndices.begin(), mTetrahedronIndices.end(), Tetrahedron::kInvalidVertexID);

	std::fill(mTetrahedronCounts.begin(), mTetrahedronCounts.end(), 0);
	for(size_t i = 0; i < mTetrahedrons.size(); ++i) {
		const auto& tindices = mTetrahedrons[i].indices;
		mTetrahedronIndices[mTetrahedronOffsets[tindices[0]] + mTetrahedronCounts[tindices[0]]++] = i;
		mTetrahedronIndices[mTetrahedronOffsets[tindices[1]] + mTetrahedronCounts[tindices[1]]++] = i;
		mTetrahedronIndices[mTetrahedronOffsets[tindices[2]] + mTetrahedronCounts[tindices[2]]++] = i;
		mTetrahedronIndices[mTetrahedronOffsets[tindices[3]] + mTetrahedronCounts[tindices[3]]++] = i;
	}

	for(size_t i = 0; i < mTetrahedronCounts.size(); ++i) {
		dbg_printf("pt %zu tetras count %zu\n", i, (size_t)mTetrahedronCounts[i]);
	}

	return true;
}

#else  // no USE_CGAL

bool PhantomTrimesh::buildTetrahedrons() {
	std::cerr << "PhantomTrimesh::buildTetrahedrons() NOT IMPLEMENTED !!!" << std::endl;
	return false;
}

#endif  // USE_CGAL

pxr::GfVec3f PhantomTrimesh::getTetrahedronRestCentroid(const Tetrahedron& t) const {
	assert(t.isValid());

	const pxr::VtArray<pxr::GfVec3f>& rest_positions = getRestPositions();
	return (rest_positions[t.indices[0]] + rest_positions[t.indices[1]] + rest_positions[t.indices[2]] + rest_positions[t.indices[3]]) / 4.0f;
}

pxr::GfVec3f PhantomTrimesh::getTetrahedronRestCentroid(size_t idx) const {
	assert(idx < mTetrahedrons.size());
	return getTetrahedronRestCentroid(mTetrahedrons[idx]);
}

void PhantomTrimesh::barycentricTetrahedronRestCoords(const Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w, float& x) const {
	assert(t.isValid());

	const pxr::VtArray<pxr::GfVec3f>& rest_positions = getRestPositions();

	const pxr::GfVec3f& a = rest_positions[t.indices[0]];
	const pxr::GfVec3f& b = rest_positions[t.indices[1]];
	const pxr::GfVec3f& c = rest_positions[t.indices[2]];
	const pxr::GfVec3f& d = rest_positions[t.indices[3]];

    pxr::GfVec3f vap = p - a;
    pxr::GfVec3f vbp = p - b;

    pxr::GfVec3f vab = b - a;
    pxr::GfVec3f vac = c - a;
    pxr::GfVec3f vad = d - a;

    pxr::GfVec3f vbc = c - b;
    pxr::GfVec3f vbd = d - b;

    // ScTP computes the scalar triple product
    auto ScTP = [](const pxr::GfVec3f &a, const pxr::GfVec3f &b, const pxr::GfVec3f &c) {
    	// computes scalar triple product
    	return pxr::GfDot(a, pxr::GfCross(b, c));
	};

    float va6 = ScTP(vbp, vbd, vbc);
    float vb6 = ScTP(vap, vac, vad);
    float vc6 = ScTP(vap, vad, vab);
    float vd6 = ScTP(vap, vab, vac);
    float v6 = 1.f / ScTP(vab, vac, vad);

    u = va6*v6;
    v = vb6*v6;
    w = vc6*v6;
    x = vd6*v6;
/*
    float _x = 1.0f - (u + v + w);

    if(!floatsAreEqualRelative(x, _x)) {
    	std::cout << "x " << x << " _x " << _x << std::endl;
    }
*/
}

void PhantomTrimesh::barycentricTetrahedronRestCoords(size_t idx, const pxr::GfVec3f& p, float& u, float& v, float& w, float& x) const {
	assert(idx < mTetrahedrons.size());
	barycentricTetrahedronRestCoords(mTetrahedrons[idx], p, u, v, w, x);
}

void PhantomTrimesh::barycentricTetrahedronRestCoords(const Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w) const {
	float _temp_x;
	barycentricTetrahedronRestCoords(t, p, u, v, w, _temp_x);
}

void PhantomTrimesh::barycentricTetrahedronRestCoords(size_t idx, const pxr::GfVec3f& p, float& u, float& v, float& w) const {
	assert(idx < mTetrahedrons.size());
	barycentricTetrahedronRestCoords(mTetrahedrons[idx], p, u, v, w);
}

pxr::GfVec3f PhantomTrimesh::getPointPositionFromBarycentricTetrahedronLiveCoords(const Tetrahedron& t, float u, float v, float w, float x) const {
	assert(t.isValid());

	const pxr::VtArray<pxr::GfVec3f>& live_positions = getLivePositions();

	assert(t.indices[0] < live_positions.size());
	assert(t.indices[1] < live_positions.size());
	assert(t.indices[2] < live_positions.size());
	assert(t.indices[3] < live_positions.size());

	const pxr::GfVec3f& a = live_positions[t.indices[0]];
	const pxr::GfVec3f& b = live_positions[t.indices[1]];
	const pxr::GfVec3f& c = live_positions[t.indices[2]];
	const pxr::GfVec3f& d = live_positions[t.indices[3]];

    return {u * a[0] + v * b[0] + w * c[0] + x * d[0], u * a[1] + v * b[1] + w * c[1] + x * d[1], u * a[2] + v * b[2] + w * c[2] + x * d[2]};
}

pxr::GfVec3f PhantomTrimesh::getPointPositionFromBarycentricTetrahedronLiveCoords(size_t idx, float u, float v, float w, float x) const {
	assert(idx < mTetrahedrons.size());
	return getPointPositionFromBarycentricTetrahedronLiveCoords(mTetrahedrons[idx], u, v, w, x);
}

uint32_t PhantomTrimesh::getPointConnectedTetrahedronIndex(size_t pt_index, size_t tetra_local_index) const { 
	assert(pt_index < mTetrahedronCounts.size()); 
	assert(tetra_local_index < mTetrahedronCounts[pt_index]);
	return mTetrahedronIndices[mTetrahedronOffsets[pt_index] + tetra_local_index];
}

void PhantomTrimesh::invalidate() {
	mValid = false;

	mUsdMeshRestPositions.clear();
	mUsdMeshLivePositions.clear();

	// Tetrahedrons
	mTetrahedronCounts.clear();
	mTetrahedronOffsets.clear();
	mTetrahedronIndices.clear();
	mTetrahedrons.clear();

	// Trifaces
	mFaceMap.clear();
	mFaces.clear();
	mFaceFlags.clear();

	mVertices.clear();
	mTmpVertices.clear();
}

uint32_t PhantomTrimesh::getFaceIDByIndices(PxrIndexType a, PxrIndexType b, PxrIndexType c) const {
	std::array<PhantomTrimesh::PxrIndexType, 3> indices{a, b, c};
	std::sort(indices.begin(), indices.end());

	auto it = mFaceMap.find(indices);
	if(it != mFaceMap.end()) {
		return static_cast<uint32_t>(it->second);
	}

	return kInvalidTriFaceID;
}

uint32_t PhantomTrimesh::getOrCreateFaceID(PhantomTrimesh::PxrIndexType a, PhantomTrimesh::PxrIndexType b, PhantomTrimesh::PxrIndexType c) {
	assert((a != b) && (a != c) && (b != c));	
	std::array<PhantomTrimesh::PxrIndexType, 3> indices{a, b, c};
	std::sort(indices.begin(), indices.end());

	auto it = mFaceMap.find(indices);
	if(it != mFaceMap.end()) {
		return static_cast<uint32_t>(it->second);
	}

	const uint32_t idx = static_cast<uint32_t>(mFaces.size());

	mFaces.emplace_back(indices);
	mFaceFlags.emplace_back(TriFace::Flags::None);

	mTmpVertices.insert(a);

	if(mTmpVertices.insert(a).second == true) mVertices.push_back(a);
	if(mTmpVertices.insert(b).second == true) mVertices.push_back(b);
	if(mTmpVertices.insert(c).second == true) mVertices.push_back(c);

	auto const& usdMeshRestPositions = mUsdMeshRestPositions.AsConst();

	mFaces.back().restNormal = pxr::GfGetNormalized(
		pxr::GfCross(usdMeshRestPositions[indices[1]] - usdMeshRestPositions[indices[0]], usdMeshRestPositions[indices[2]] - usdMeshRestPositions[indices[0]])
	);

	mFaceMap[indices] = idx;

	return idx;
}

uint32_t PhantomTrimesh::getOrCreateFaceID(const std::array<PxrIndexType, 3>& a) {
	return getOrCreateFaceID(a[0], a[1], a[2]);
}

bool PhantomTrimesh::projectPoint(const pxr::GfVec3f& pt, const uint32_t face_id, float& u, float& v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;
	
	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	auto const& usdMeshRestPositions = mUsdMeshRestPositions.AsConst();
	return rayTriangleIntersect(pt, -face.getRestNormal(), usdMeshRestPositions[face.indices[0]], usdMeshRestPositions[face.indices[1]], usdMeshRestPositions[face.indices[2]], u, v);
}

bool PhantomTrimesh::projectPoint(const pxr::GfVec3f& pt, const uint32_t face_id, float& u, float& v, float& dist) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;
	
	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	auto const& usdMeshRestPositions = mUsdMeshRestPositions.AsConst();
	return rayTriangleIntersect(pt, -face.getRestNormal(), usdMeshRestPositions[face.indices[0]], usdMeshRestPositions[face.indices[1]], usdMeshRestPositions[face.indices[2]], dist, u, v);
}

bool PhantomTrimesh::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const uint32_t face_id, float& u, float& v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;

	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	auto const& usdMeshRestPositions = mUsdMeshRestPositions.AsConst();
	return rayTriangleIntersect(orig, dir, usdMeshRestPositions[face.indices[0]], usdMeshRestPositions[face.indices[1]], usdMeshRestPositions[face.indices[2]], u, v);
}

bool PhantomTrimesh::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const uint32_t face_id, float& u, float& v, float& dist) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());
	if(face_id == kInvalidTriFaceID) return false;

	const TriFace& face = mFaces[static_cast<size_t>(face_id)];
	auto const& usdMeshRestPositions = mUsdMeshRestPositions.AsConst();
	return rayTriangleIntersect(orig, dir, usdMeshRestPositions[face.indices[0]], usdMeshRestPositions[face.indices[1]], usdMeshRestPositions[face.indices[2]], dist, u, v);
}

pxr::GfVec3f PhantomTrimesh::getInterpolatedRestPosition(const uint32_t face_id, const float u, const float v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	auto const& usdMeshRestPositions = mUsdMeshRestPositions.AsConst();

	return u * usdMeshRestPositions[face.indices[1]] + v * usdMeshRestPositions[face.indices[2]] + (1.f - u - v) * usdMeshRestPositions[face.indices[0]];
};


pxr::GfVec3f PhantomTrimesh::getInterpolatedLivePosition(const uint32_t face_id, const float u, const float v) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& face = mFaces[face_id];
	auto const& usdMeshLivePositions = mUsdMeshLivePositions.AsConst();

	return u * usdMeshLivePositions[face.indices[1]] + v * usdMeshLivePositions[face.indices[2]] + (1.f - u - v) * usdMeshLivePositions[face.indices[0]];
};

pxr::GfVec3f PhantomTrimesh::getFaceRestCentroid(const uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	static constexpr float kInvThree = 1.f / 3.f;
	auto const& face = mFaces[face_id];
	auto const& usdMeshRestPositions = mUsdMeshRestPositions.AsConst();

	return (usdMeshRestPositions[face.indices[0]] + usdMeshRestPositions[face.indices[1]] + usdMeshRestPositions[face.indices[2]]) * kInvThree;
}

bool PhantomTrimesh::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) const {
	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());

	pxr::UsdGeomPointBased mesh(prim_handle.getPrim());

	if(!mesh.GetPointsAttr().Get(&mUsdMeshLivePositions, time_code)) {
		std::cerr << "Error getting point positions from " << prim_handle.getPath() << " !" << std::endl;
		return false;
	}

	if(mUsdMeshLivePositions.size() != mUsdMeshRestPositions.size()) {
		std::cerr << prim_handle.getPath() << " \"rest\" and \"live\" mesh point positions count (" << mUsdMeshRestPositions.size() << " vs " << mUsdMeshLivePositions.size() << " ) mismatch !" << std::endl;
		return false;
	}

	return true;
}

const pxr::GfVec3f& PhantomTrimesh::getFaceRestNormal(const uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	return mFaces[face_id].getRestNormal();
}

pxr::GfVec3f PhantomTrimesh::getFaceLiveNormal(const uint32_t face_id) const {
	assert(static_cast<size_t>(face_id) < mFaces.size());

	auto const& indices = mFaces[face_id].indices;
	auto const& usdMeshLivePositions = mUsdMeshLivePositions.AsConst();

	return pxr::GfGetNormalized(
		pxr::GfCross(usdMeshLivePositions[indices[1]] - usdMeshLivePositions[indices[0]], usdMeshLivePositions[indices[2]] - usdMeshLivePositions[indices[0]])
	);
}

size_t PhantomTrimesh::calcHash() const {
	size_t hash = 0;

	for(const auto& pos: mUsdMeshRestPositions) {
		hash += static_cast<uint32_t>(pos[0]) + (static_cast<uint32_t>(pos[1]) << 1) + (static_cast<uint32_t>(pos[2]) << 2);
	}
	hash += mUsdMeshRestPositions.size() << 3;

	for(const auto& face: mFaces) {
		hash += face.calcHash();
	}
	hash += mFaces.size() << 6;

	for(const TriFace::Flags flag: mFaceFlags) {
		hash += static_cast<size_t>(flag);
	}
	hash += mFaceFlags.size() << 8;

	for(const auto& [k, v]: mFaceMap) {
		hash += k[0] + (k[1] << 1) + (k[2] << 2) + (v << 7);
	}
	hash += mFaceMap.size() << 10;

	size_t verices_hash = 0;
	for(const PxrIndexType& i: mVertices) {
		verices_hash += static_cast<uint32_t>(i); 
	}
	hash += verices_hash << 16;

	return hash;
}

//void to_json(json& j, const PhantomTrimesh::TriFace& face) {
//	j["indices"] = face.indices;
//	to_json(j["normal"], face.restNormal);
//}

//void from_json(const json& j, PhantomTrimesh::TriFace& face) {
//	j.at("indices").get_to(face.indices);
//	from_json(j["normal"], face.restNormal);
//}

void to_json(json& j, const std::vector<PhantomTrimesh::TriFace>& trifaces) {
	for(const auto& f: trifaces) {
    	j.push_back({
    		f.indices[0], f.indices[1], f.indices[2],
    		f.restNormal[0], f.restNormal[1], f.restNormal[2],
    	});
    }
}

void from_json(const json& j, std::vector<PhantomTrimesh::TriFace>& trifaces) {
	trifaces.clear();
	for (const auto& e : j) {
 		trifaces.emplace_back(
 			e.at(0).template get<PhantomTrimesh::PxrIndexType>(),
 			e.at(1).template get<PhantomTrimesh::PxrIndexType>(),
 			e.at(2).template get<PhantomTrimesh::PxrIndexType>(),
 			e.at(3).template get<float>(),
 			e.at(4).template get<float>(),
 			e.at(5).template get<float>()
 		);
	}
}



static constexpr const char* kJUsdRestPositions = "rest_pos";
static constexpr const char* kJFaces = "faces";
static constexpr const char* kJVertices = "vertices";
static constexpr const char* kJFaceMap = "face_map";
static constexpr const char* kJFaceFlags = "face_flags";
static constexpr const char* kJDataHash = "hash";

SerializablePhantomTrimesh::SerializablePhantomTrimesh() {
	mpTrimesh = PhantomTrimesh::create();
}

bool SerializablePhantomTrimesh::buildInPlace(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());
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
	j[kJFaceMap] = mpTrimesh->mFaceMap;
	j[kJFaceFlags] = mpTrimesh->mFaceFlags;
	j[kJVertices] = mpTrimesh->mVertices;

	j[kJDataHash] = mpTrimesh->calcHash();

	return true;
}

bool SerializablePhantomTrimesh::readFromJSON(const json& j) {
	mpTrimesh = std::make_unique<PhantomTrimesh>();
	mpTrimesh->mValid = false;

	from_json(j[kJUsdRestPositions], mpTrimesh->mUsdMeshRestPositions);
	mpTrimesh->mFaces = j[kJFaces].template get<std::vector<PhantomTrimesh::TriFace>>();
	mpTrimesh->mFaceMap = j[kJFaceMap].template get<std::unordered_map<std::array<PhantomTrimesh::PxrIndexType, 3>, size_t, IndicesArrayHasher<PhantomTrimesh::PxrIndexType, 3>>>();
	mpTrimesh->mFaceFlags = j[kJFaceFlags].template get<std::vector<PhantomTrimesh::TriFace::Flags>>();
	mpTrimesh->mVertices = j[kJVertices].template get<std::vector<PhantomTrimesh::PxrIndexType>>();

	const size_t json_trimesh_data_hash = j[kJDataHash];
	const size_t calc_trimesh_data_hash = mpTrimesh->calcHash();

	if(calc_trimesh_data_hash != json_trimesh_data_hash) {
		return false;
	}

	if(mpTrimesh->mFaces.size() != mpTrimesh->mFaceFlags.size()) {
		std::cerr << "Trimesh face and face_flags array sizes mismatch!" << std::endl;
		return false;
	}

	mpTrimesh->mTmpVertices.clear();
	for(const auto& vtx: mpTrimesh->mVertices) {
		mpTrimesh->mTmpVertices.insert(vtx);
	}

	mpTrimesh->mValid = true;

	dbg_printf("PhantomTrimesh data read from json payload !\n");

	return true;
}

PhantomTrimesh* SerializablePhantomTrimesh::getTrimesh() {
	return isPopulated() ? mpTrimesh.get() : nullptr;
}

const PhantomTrimesh* SerializablePhantomTrimesh::getTrimesh() const {
	return isPopulated() ? mpTrimesh.get() : nullptr;
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
