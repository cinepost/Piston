#include "pxr_json.h"
#include "phantom_trimesh.h"
#include "geometry_tools.h"
#include "simple_profiler.h"
#include "piston_math.h"
#include "logging.h"

#ifdef USE_CGAL

#define CGAL_DO_NOT_USE_BOOST_MP
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Triangulation_3.h>

#endif // USE_CGAL

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kTrimeshDataVersion( 0u, 0u, 1u);

PhantomTrimesh::UniquePtr PhantomTrimesh::create() {
	return std::make_unique<PhantomTrimesh>();
}

PhantomTrimesh::PhantomTrimesh() : mPointsCount(0u), mIsValid(false) {
	static const size_t reserve_size = 1024;

	mFaceMap.reserve(reserve_size);
	mFaces.reserve(reserve_size);
	mFaceFlags.reserve(reserve_size);
}

bool PhantomTrimesh::init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	mIsValid = false;

	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(rest_p_name));
	pxr::VtArray<pxr::GfVec3f> points;

	if(!restPositionPrimVar) {
		LOG_WRN << "No valid primvar \"" << rest_p_name << "\" exists in prim " << prim_handle.getPath() << "! Using positions at time code 0.0 !";

		pxr::UsdGeomPointBased mesh(prim_handle.getPrim());

		static const pxr::UsdTimeCode s_zero_time_code(0.0);

		if(!mesh.GetPointsAttr().Get(&points, s_zero_time_code)) {
			LOG_ERR << "Error getting " << prim_handle.getPath() << " point positions at time code " << s_zero_time_code.GetValue();
			return false;
		}
	} else {
		const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
		if(!restPosAttr.Get(&points, time_code)) {
			LOG_ERR << "Error getting prim " << prim_handle.getPath() << " \"rest\" positions !";
			return false;
		}
		LOG_DBG << "Prim " << prim_handle.getPath() << " has " << points.size() << " points.";
	}

	mPointsCount = points.size();
	return true;
}

#ifdef USE_CGAL

bool PhantomTrimesh::buildTetrahedrons(const pxr::VtArray<pxr::GfVec3f>& positions) {
	using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Vb =  CGAL::Triangulation_vertex_base_with_info_3<uint32_t, Kernel>;
	using Tds = CGAL::Triangulation_data_structure_3<Vb>;
	using Triangulation = CGAL::Triangulation_3<Kernel, Tds>;
	
	using CGAL_Point = Triangulation::Point;

	using CGAL_CellHandle = Triangulation::Cell_handle;
	using CGAL_VtxHandle = Triangulation::Vertex_handle;
	using CGAL_LocateType = Triangulation::Locate_type;

	const size_t points_count = positions.size();

	if(points_count < 4) {
		LOG_ERR << "Mesh has less than 4 vertices !";
		return false;
	}
	LOG_DBG << "PhantomTrimesh::buildTetrahedrons(...) for " << positions.size() << " points.";

	std::vector< std::pair<CGAL_Point, uint32_t> > points;
	for(size_t i = 0; i < points_count; ++i) {
		const auto& pxr_pt = positions[i];
  		points.push_back( std::make_pair( CGAL_Point(pxr_pt[0], pxr_pt[1], pxr_pt[2]), (uint32_t)i ));
  	}

	Triangulation T(points.begin(), points.end());

	LOG_TRC << "T number of cells: " << (size_t)T.number_of_cells();
	LOG_TRC << "T number of finite cells: " << (size_t)T.number_of_finite_cells();

	mTetrahedrons.resize(T.number_of_finite_cells());

	size_t i = 0;
	for(const CGAL_CellHandle& cell_handle: T.finite_cell_handles()) {
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
	for(uint32_t i = 0; i < mTetrahedronCounts.size(); ++i) {
		mTetrahedronOffsets[i] = offset;
		offset += mTetrahedronCounts[i];
	}

	mTetrahedronIndices.resize(mTetrahedrons.size() * 4);
	std::fill(mTetrahedronIndices.begin(), mTetrahedronIndices.end(), Tetrahedron::kInvalidVertexID);

	std::fill(mTetrahedronCounts.begin(), mTetrahedronCounts.end(), 0);
	for(uint32_t i = 0; i < mTetrahedrons.size(); ++i) {
		const auto& tindices = mTetrahedrons[i].indices;
		mTetrahedronIndices[mTetrahedronOffsets[tindices[0]] + mTetrahedronCounts[tindices[0]]++] = i;
		mTetrahedronIndices[mTetrahedronOffsets[tindices[1]] + mTetrahedronCounts[tindices[1]]++] = i;
		mTetrahedronIndices[mTetrahedronOffsets[tindices[2]] + mTetrahedronCounts[tindices[2]]++] = i;
		mTetrahedronIndices[mTetrahedronOffsets[tindices[3]] + mTetrahedronCounts[tindices[3]]++] = i;
	}

	return true;
}

#else  // no USE_CGAL

bool PhantomTrimesh::buildTetrahedrons() {
	LOG_ERR << "PhantomTrimesh::buildTetrahedrons() NOT IMPLEMENTED !!!";
	return false;
}

#endif  // USE_CGAL

uint32_t PhantomTrimesh::getPointConnectedTetrahedronIndex(size_t pt_index, size_t tetra_local_index) const { 
	assert(pt_index < mTetrahedronCounts.size()); 
	assert(tetra_local_index < mTetrahedronCounts[pt_index]);
	return mTetrahedronIndices[mTetrahedronOffsets[pt_index] + tetra_local_index];
}

void PhantomTrimesh::invalidate() {
	mIsValid = false;

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

bool PhantomTrimesh::isValid() const { 
	return mIsValid && !mFaces.empty(); 
}

uint32_t PhantomTrimesh::getFaceIDByIndices(PxrIndexType a, PxrIndexType b, PxrIndexType c) const {
	std::array<PhantomTrimesh::PxrIndexType, 3> indices{a, b, c};
	std::sort(indices.begin(), indices.end());

	{
		std::scoped_lock lock(mFaceMapMutex);

		auto it = mFaceMap.find(indices);
		if(it != mFaceMap.end()) {
			return static_cast<uint32_t>(it->second);
		}
	}

	return kInvalidTriFaceID;
}

uint32_t PhantomTrimesh::getOrCreateFaceID(PhantomTrimesh::PxrIndexType a, PhantomTrimesh::PxrIndexType b, PhantomTrimesh::PxrIndexType c) {
	assert((a != b) && (a != c) && (b != c));	
	std::array<PhantomTrimesh::PxrIndexType, 3> indices{a, b, c};
	std::sort(indices.begin(), indices.end());

	std::scoped_lock lock(mFaceMapMutex);

	auto it = mFaceMap.find(indices);
	if(it != mFaceMap.end()) {
		return static_cast<uint32_t>(it->second);
	}

	const uint32_t idx = static_cast<uint32_t>(mFaces.size());

	mFaces.emplace_back(indices);
	mFaceFlags.emplace_back(TriFace::Flags::None);

	if(mTmpVertices.insert(a).second == true) mVertices.push_back(a);
	if(mTmpVertices.insert(b).second == true) mVertices.push_back(b);
	if(mTmpVertices.insert(c).second == true) mVertices.push_back(c);

	mFaceMap[indices] = idx;

	return idx;
}

uint32_t PhantomTrimesh::getOrCreateFaceID(const std::array<PxrIndexType, 3>& a) {
	return getOrCreateFaceID(a[0], a[1], a[2]);
}

size_t PhantomTrimesh::calcHash() const {
	size_t hash = 0;

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

void to_json(json& j, const std::vector<PhantomTrimesh::TriFace>& trifaces) {
	for(const auto& f: trifaces) {
    	j.push_back({
    		f.indices[0], f.indices[1], f.indices[2]
    	});
    }
}

void from_json(const json& j, std::vector<PhantomTrimesh::TriFace>& trifaces) {
	trifaces.clear();
	for (const auto& e : j) {
 		trifaces.emplace_back(
 			e.at(0).template get<PhantomTrimesh::PxrIndexType>(),
 			e.at(1).template get<PhantomTrimesh::PxrIndexType>(),
 			e.at(2).template get<PhantomTrimesh::PxrIndexType>()
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

	if(isValid()) {
		// Trimesh data is still valid. No rebuild.
		return true;
	}

	clearData();

	const std::lock_guard<std::mutex> lock(mMutex);

	if(!mpTrimesh) {
		mpTrimesh = PhantomTrimesh::create();
	}

	bool result = mpTrimesh->init(prim_handle, rest_p_name);

	if(!result) {
		mpTrimesh->invalidate();
	}

	return result;
}

void SerializablePhantomTrimesh::clearData() {
	const std::lock_guard<std::mutex> lock(mMutex);
	if(mpTrimesh) {
		mpTrimesh->invalidate();
	}
}

bool SerializablePhantomTrimesh::dumpToJSON(json& j) const {
	if(!mpTrimesh || !mpTrimesh->isValid()) return false;

	const std::lock_guard<std::mutex> lock(mMutex);
	j[kJFaces] = mpTrimesh->mFaces;
	j[kJFaceMap] = mpTrimesh->mFaceMap;
	j[kJFaceFlags] = mpTrimesh->mFaceFlags;
	j[kJVertices] = mpTrimesh->mVertices;
	j[kJDataHash] = mpTrimesh->calcHash();

	return true;
}

bool SerializablePhantomTrimesh::readFromJSON(const json& j) {
	const std::lock_guard<std::mutex> lock(mMutex);

	mpTrimesh = std::make_unique<PhantomTrimesh>();
	mpTrimesh->mIsValid = false;

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
		LOG_ERR << "Trimesh face and face_flags array sizes mismatch!";
		return false;
	}

	mpTrimesh->mTmpVertices.clear();
	for(const auto& vtx: mpTrimesh->mVertices) {
		mpTrimesh->mTmpVertices.insert(vtx);
	}

	mpTrimesh->mIsValid = true;

	LOG_DBG << "PhantomTrimesh data read from json payload.";

	return true;
}

void SerializablePhantomTrimesh::setValid(bool state) {
	if(!mpTrimesh) return;

	const std::lock_guard<std::mutex> lock(mMutex); 
	const std::lock_guard<std::mutex> trimesh_facemap_lock(mpTrimesh->mFaceMapMutex);

	if(mpTrimesh->mIsValid != state) {
		// TODO: check if we need to update other states... Anyways this is kinda ugly
		mpTrimesh->mIsValid = state;
	}
}

PhantomTrimesh* SerializablePhantomTrimesh::getTrimesh() {
	return mpTrimesh ? mpTrimesh.get() : nullptr;
}

const PhantomTrimesh* SerializablePhantomTrimesh::getTrimesh() const {
	return mpTrimesh ? mpTrimesh.get() : nullptr;
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
