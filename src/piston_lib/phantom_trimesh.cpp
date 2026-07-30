#include "pxr_json.h"
#include "phantom_trimesh.h"
#include "guide_curves_container.h"
#include "geometry_tools.h"
#include "simple_profiler.h"
#include "piston_math.h"
#include "kdtree.hpp"
#include "logging.h"

#ifdef USE_CGAL

#define CGAL_DO_NOT_USE_BOOST_MP
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Regular_triangulation_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_3.h>

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

bool PhantomTrimesh::init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code) {
	mIsValid = false;

	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(prim_handle.getRestAttrName()));
	pxr::VtArray<pxr::GfVec3f> points;

	if(!restPositionPrimVar) {
		LOG_WRN << "No valid primvar \"" << prim_handle.getRestAttrName() << "\" exists in prim " << prim_handle.getPath() << "! Using positions at time code 0.0 !";

		pxr::UsdGeomPointBased mesh(prim_handle.getPrim());

		static const pxr::UsdTimeCode s_zero_time_code(0.0);

		if(!mesh.GetPointsAttr().Get(&points, s_zero_time_code)) {
			LOG_ERR << "Error getting " << prim_handle.getPath() << " point positions at time code " << s_zero_time_code.GetValue();
			return false;
		}
	} else {
		const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
		if(!restPosAttr.Get(&points, rest_time_code)) {
			LOG_ERR << "Error getting prim " << prim_handle.getPath() << " \"rest\" positions !";
			return false;
		}
		LOG_DBG << "Prim " << prim_handle.getPath() << " has " << points.size() << " points.";
	}

	mPointsCount = points.size();
	return true;
}

#ifdef USE_CGAL

template <typename T>
bool PhantomTrimesh::buildTetrahedrons(const T& positions, const GuideCurvesContainer* pCurvesContainer) {
	PROFILE("PhantomTrimesh::buildTetrahedrons");
	static_assert(std::is_same_v<T, std::vector<pxr::GfVec3f>> || std::is_same_v<T, pxr::VtArray<pxr::GfVec3f>>, "Only std::vector<pxr::GfVec3f> and pxr::VtArray<pxr::GfVec3f> types are permitted!"); 

	using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point_3 = Kernel::Point_3;
	using Weighted_point = Kernel::Weighted_point_3;

	using Rt_Vb = CGAL::Regular_triangulation_vertex_base_3<Kernel>;
	using Rt_Alpha_Vb = CGAL::Alpha_shape_vertex_base_3<Kernel, Rt_Vb>;
	using Vb_with_info = CGAL::Triangulation_vertex_base_with_info_3<unsigned int, Kernel, Rt_Alpha_Vb>;

	using Rt_Cb = CGAL::Regular_triangulation_cell_base_3<Kernel>;
	using Cb_with_alpha = CGAL::Alpha_shape_cell_base_3<Kernel, Rt_Cb>;

	// 4. Mesh Assembly Topology Types
	using Tds = CGAL::Triangulation_data_structure_3<Vb_with_info, Cb_with_alpha>;
	using Rt = CGAL::Regular_triangulation_3<Kernel, Tds>;
	using Weighted_Alpha_shape = CGAL::Alpha_shape_3<Rt>;  

	//using CGAL_CellHandle = Triangulation::Cell_handle;
	//using CGAL_VtxHandle = Triangulation::Vertex_handle;
	//using CGAL_LocateType = Triangulation::Locate_type;

	const size_t points_count = positions.size();

	if(points_count < 4) {
		LOG_ERR << "Mesh has less than 4 vertices !";
		return false;
	}
	LOG_DBG << "PhantomTrimesh::buildTetrahedrons(...) for " << positions.size() << " points.";

	// Mark root and tip vertices
	std::vector<bool> tipVerticesFlags;
	std::vector<bool> rootVerticesFlags;

	if(pCurvesContainer) {
		if(points_count != pCurvesContainer->getTotalVertexCount())	{
			LOG_ERR << "PhantomTrimesh::buildTetrahedrons positions and pCurvesContainer points count mismatch!! Filtering disabled!";
			return false;
		}
			
		tipVerticesFlags.resize(points_count);
		rootVerticesFlags.resize(points_count);

		std::fill(tipVerticesFlags.begin(), tipVerticesFlags.begin(), false);
		std::fill(rootVerticesFlags.begin(), rootVerticesFlags.begin(), false);

		const auto& curve_vertex_counts = pCurvesContainer->getCurveVertexCounts();
		//const auto& curves_offsets = pCurvesContainer->getCurveOffsets();

		size_t curr_idx = 0;
		for(auto count: curve_vertex_counts) {
			if(count == 0) continue;

			rootVerticesFlags[curr_idx] = true;
			tipVerticesFlags[curr_idx + count - 1] = true;
			curr_idx += count;
		}
	}
/*
	std::vector<std::pair<CGAL_Point, uint32_t>> points;
	for(size_t i = 0; i < points_count; ++i) {
		const auto& pxr_pt = positions[i];
  		points.push_back( std::make_pair( CGAL_Point(pxr_pt[0], pxr_pt[1], pxr_pt[2]), (uint32_t)i ));
  	}
*/

	// Calc point weights
	neighbour_search::KDTree<float, 3> deformer_restpoints_kdtree = neighbour_search::KDTree<float, 3>(positions, true /* multi threaded */);

	std::vector<std::tuple<Weighted_point, unsigned int>> input_data;
     
	static const auto kClosestNeighborsCount = 4;

	for(size_t i = 0; i < points_count; ++i) {
		const auto& pxr_pt = positions[i];
  		std::vector<neighbour_search::KDTree<float, 3>::ReturnType> closest_points(kClosestNeighborsCount);
  		deformer_restpoints_kdtree.findKNearestNeighbours(pxr_pt, kClosestNeighborsCount, closest_points);

  		double total_distance = 0.0;
  		uint32_t count = 0;
  		for(uint32_t j = 0; j < kClosestNeighborsCount; ++j) {
  			auto dist = distance(pxr_pt, positions[closest_points[j].first]);
  			if(dist > 0.0001) {
  				total_distance += dist;
  				count++;
  			}
  		}

  		double avg_neighbor_distance = (count > 0) ? (total_distance / count) : 1.0;
  		double calculated_radius = avg_neighbor_distance * 0.5;
  		double target_weight = calculated_radius * calculated_radius;

  		input_data.push_back({Weighted_point(Point_3(pxr_pt[0], pxr_pt[1], pxr_pt[2]), target_weight), (unsigned int)i});
  	}

  	Rt rt;

  	// Insert points one-by-one to explicitly couple the vertex info
    for (const auto& item : input_data) {
        Weighted_point wp = std::get<0>(item);
        uint32_t source_idx   = std::get<1>(item);

        Rt::Vertex_handle vh = rt.insert(wp);
        
        // Handle overlapping points or geometric degeneracies gracefully
        if (vh != Rt::Vertex_handle()) {
            vh->info() = source_idx; // Assign the original index to the CGAL vertex
        }
    }

	//Triangulation tr(points.begin(), points.end());
	//Delaunay tr(points.begin(), points.end());
	double base_alpha = 0.0; 
	Weighted_Alpha_shape tr(rt, Kernel::FT(base_alpha), Weighted_Alpha_shape::REGULARIZED);

	// Alternate strategy: If 0.0 is too aggressive, let CGAL find the optimal 
    // baseline where your weights begin to matter dynamically:
    Weighted_Alpha_shape::Alpha_iterator opt_alpha_it = tr.find_optimal_alpha(1);
    double calculated_best_alpha = *opt_alpha_it;
    tr.set_alpha(calculated_best_alpha); // Finds the sharpest crisp boundary threshold
    LOG_DBG << "Dynamic alpha threshold derived from weightd: " << calculated_best_alpha;

	LOG_DBG << "T number of cells: " << (size_t)tr.number_of_cells();
	LOG_DBG << "T number of finite cells: " << (size_t)tr.number_of_finite_cells();

	// Check that tetra is not connecting all tips or all root vertices of a cureves 
	auto checkIsValid = [&tipVerticesFlags, &rootVerticesFlags](uint32_t i0, uint32_t i1, uint32_t i2, uint32_t i3) {
		if(tipVerticesFlags[i0] && tipVerticesFlags[i1] && tipVerticesFlags[i2] && tipVerticesFlags[i3]) return false;
		if(rootVerticesFlags[i0] && rootVerticesFlags[i1] && rootVerticesFlags[i2] && rootVerticesFlags[i3]) return false;

		if((rootVerticesFlags[i0] || tipVerticesFlags[i0]) &&
		   (rootVerticesFlags[i1] || tipVerticesFlags[i1]) &&
		   (rootVerticesFlags[i2] || tipVerticesFlags[i2]) &&
		   (rootVerticesFlags[i3] || tipVerticesFlags[i3])) return false;

		return true;
	};

	for(auto cit = tr.finite_cells_begin(); cit != tr.finite_cells_end(); ++cit) {
		if(tr.classify(cit) != Weighted_Alpha_shape::CELL) continue;

        // Access vertices via cit->vertex(0), cit->vertex(1), etc.
        if(checkIsValid(cit->vertex(0)->info() ,cit->vertex(1)->info(), cit->vertex(2)->info() ,cit->vertex(3)->info())) {
        	mTetrahedrons.emplace_back(
        		cit->vertex(0)->info()
        		,cit->vertex(1)->info()
        		,cit->vertex(2)->info()
        		,cit->vertex(3)->info()
    		);
    	}
    }

    LOG_DBG << "Filtered tetrahedrons count: " << mTetrahedrons.size();

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


template <typename T>
bool PhantomTrimesh::buildTetrahedrons(const T& positions) {
	LOG_ERR << "PhantomTrimesh::buildTetrahedrons() NOT IMPLEMENTED !!!";
	return false;
}

#endif  // USE_CGAL

bool PhantomTrimesh::hasTetrahedrons() const { 
	LOG_DBG << "PhantomTrimesh::hasTetrahedrons()";
	LOG_DBG << "mTetrahedronCounts count " << mTetrahedronCounts.size();
	LOG_DBG << "mTetrahedronOffsets count " << mTetrahedronOffsets.size();
	LOG_DBG << "mPointsCount " << mPointsCount;

	return (!mTetrahedrons.empty()) && (mTetrahedronCounts.size() == mTetrahedronOffsets.size()); 
}

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

bool SerializablePhantomTrimesh::buildInPlace(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) {
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

	bool result = mpTrimesh->init(prim_handle);

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

	if(mpTrimesh->mIsValid == state) return;

	// TODO: check if we need to update other states... Anyways this is kinda ugly
	mpTrimesh->mIsValid = state;
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

template bool PhantomTrimesh::buildTetrahedrons(const std::vector<pxr::GfVec3f>& positions, const GuideCurvesContainer* pCurvesContainer);
template bool PhantomTrimesh::buildTetrahedrons(const pxr::VtArray<pxr::GfVec3f>& positions, const GuideCurvesContainer* pCurvesContainer);

} // namespace Piston
