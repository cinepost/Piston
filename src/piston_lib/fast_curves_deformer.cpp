#include "simple_profiler.h"
#include "fast_curves_deformer.h"

#include "common.h"
#include "kdtree.hpp"
#include "geometry_tools.h"
//#include "simple_profiler.h"

#include <pxr/base/gf/matrix4f.h>

#include <random>
#include <algorithm>

static inline float distanceSquared(const pxr::GfVec3f &p1, const pxr::GfVec3f &p2) {
	const auto dx = p1[0] - p2[0];
	const auto dy = p1[1] - p2[1];
	const auto dz = p1[2] - p2[2];
	return (dx * dx) + (dy * dy) + (dz * dz);
}

namespace Piston {

FastCurvesDeformer::FastCurvesDeformer(): BaseCurvesDeformer() {
	dbg_printf("FastCurvesDeformer::FastCurvesDeformer()\n");
}

FastCurvesDeformer::SharedPtr FastCurvesDeformer::create() {
	return SharedPtr(new FastCurvesDeformer());
}

const std::string& FastCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "FastCurvesDeformer";
	return kFastDeformerString;
}

bool FastCurvesDeformer::deformImpl(pxr::UsdTimeCode time_code) {
	PROFILE("FastCurvesDeformer::deformImpl");
	dbg_printf("FastCurvesDeformer::deformImpl()\n");
	if(!mpPhantomTrimesh) return false;

	if(!mpPhantomTrimesh->update(mMeshGeoPrimHandle, time_code)) {
		return false;
	}

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	PxrCurvesContainer* pCurves = mpCurvesContainer.get();

	if(!pCurves || pCurves->empty()) {
		return false;
	}

	const bool build_live = true; // build using live data
	if(!calcPerBindNormals(build_live)) {
		std::cerr << "Error building interpolated live normals !" << std::endl;
		return false;
	}

	calcPerBindTangentsAndBiNormals(build_live);

	std::vector<pxr::GfVec3f>& points = pCurves->getPointsCache();

	assert(mCurveBinds.size() == pCurves->getCurvesCount());

	for(uint32_t i = 0; i < pCurves->getCurvesCount(); ++i) {
		const auto& bind = mCurveBinds[i];
		if(bind.face_id == CurveBindData::kInvalidFaceID) {
			// invalid face_id
			continue;
		}

		const pxr::GfVec3f& N = mPerBindLiveNormals[i];
		const pxr::GfVec3f& T = mPerBindLiveTBs[i].first;
		const pxr::GfVec3f& B = mPerBindLiveTBs[i].second;

		const pxr::GfMatrix3f m = {
			N[0], T[0], B[0], 
			N[1], T[1], B[1],
			N[2], T[2], B[2]
		};

		auto curve_bind_pos = mpPhantomTrimesh->getInterpolatedLivePosition(bind.face_id, bind.u, bind.v);
		uint32_t vertex_offset = pCurves->getCurveVertexOffset(i);
		PxrCurvesContainer::CurveDataPtr curve_data_ptr = pCurves->getCurveDataPtr(i);

		for(size_t j = 0; j < curve_data_ptr.first; ++j) {
			points[vertex_offset++] = curve_bind_pos + m * (*(curve_data_ptr.second + j));
		}
	}

	if(!curves.GetPointsAttr().Set(pCurves->getPointsCacheVtArray(), time_code)) {
		return false;
	}

	return true;
}

bool FastCurvesDeformer::buildDeformerData(pxr::UsdTimeCode rest_time_code) {
	PROFILE("FastCurvesDeformer::buildDeformerData");
	dbg_printf("FastCurvesDeformer::buildDeformerData()\n");

	pxr::UsdGeomMesh mesh(mMeshGeoPrimHandle.getPrim());

	// Create adjacency data
	mpAdjacency = UsdGeomMeshFaceAdjacency::create(mesh);
	if(!mpAdjacency) {
		return false;
	}

	assert(mpAdjacency->getMaxFaceVertexCount() > 0);

	// Create phantom mesh
	mpPhantomTrimesh = PhantomTrimesh<PxrIndexType>::create(mMeshGeoPrimHandle, mMeshRestPositionAttrName);
	if(!mpPhantomTrimesh) {
		std::cerr << "Error creating phantom trimesh for " << mMeshGeoPrimHandle.getPath() << " !" << std::endl;
		return false;
	}

	if(!buildCurvesBindingData(rest_time_code)) {
		return false;
	}

	// Additional per-bind rest normals if needed
	const bool build_live_per_bind_data = false; // build using rest data
	if(!calcPerBindNormals(build_live_per_bind_data)) {
		std::cerr << "Error building per curve " << (build_live_per_bind_data ? "\"live\"" : "\"rest\"") << " normals !" << std::endl;
		return false;
	}

	if(!calcPerBindTangentsAndBiNormals(build_live_per_bind_data)) {
		std::cerr << "Error building per curve " << (build_live_per_bind_data ? "\"live\"" : "\"rest\"") << " tangents and binormals !" << std::endl;
		return false;
	}

	// transform curves to NTB spaces
	transformCurvesToNTB();

	return true; 
}

bool FastCurvesDeformer::calcPerBindNormals(bool build_live) {
	assert(mpPhantomTrimesh);
	assert(mpAdjacency);

	if(!mpAdjacency || !mpPhantomTrimesh) return false;

	const pxr::VtArray<pxr::GfVec3f>& pt_positions = build_live ? mpPhantomTrimesh->getLivePositions() : mpPhantomTrimesh->getRestPositions();

	std::vector<pxr::GfVec3f>& vertex_normals = build_live ? mLiveVertexNormals : mRestVertexNormals;

	buildVertexNormals(mpAdjacency.get(), mpPhantomTrimesh.get(), vertex_normals, build_live);

	// Build per bind normals
	auto& perBindNormals = build_live ? mPerBindLiveNormals : mPerBindRestNormals;
	perBindNormals.resize(mCurveBinds.size());
	for(size_t i = 0; i < mCurveBinds.size(); ++i) {
		const auto& bind = mCurveBinds[i];
		
		if(bind.face_id == CurveBindData::kInvalidFaceID) continue;

		const auto& face = mpPhantomTrimesh->getFace(bind.face_id);
		perBindNormals[i] = pxr::GfGetNormalized(bind.u * vertex_normals[face.indices[1]] + bind.v * vertex_normals[face.indices[2]] + (1.f - bind.u - bind.v) * vertex_normals[face.indices[0]]);
	}

	return true;
}

bool FastCurvesDeformer::calcPerBindTangentsAndBiNormals(bool build_live) {
	static constexpr float kF = 1.f / 3.f;
	auto& perBindNormals = build_live ? mPerBindLiveNormals : mPerBindRestNormals;

	assert(perBindNormals.size() == mCurveBinds.size());

	auto& mPerBindTBs = build_live ? mPerBindLiveTBs : mPerBindRestTBs;
	mPerBindTBs.resize(mCurveBinds.size());

	const pxr::VtArray<pxr::GfVec3f>& pt_positions = build_live ? mpPhantomTrimesh->getLivePositions() : mpPhantomTrimesh->getRestPositions();
	const std::vector<PhantomTrimesh<int>::TriFace>& faces = mpPhantomTrimesh->getFaces(); 

	pxr::VtArray<pxr::GfVec3f> face_center_points(faces.size());

	for(size_t i = 0; i < faces.size(); ++i) {
		const auto& face = faces[i];
		face_center_points[i] = (pt_positions[face.indices[0]] + pt_positions[face.indices[1]] + pt_positions[face.indices[2]]) * kF;
	}

	for(size_t i = 0; i < mCurveBinds.size(); ++i) {
		const auto& bind = mCurveBinds[i];
		
		if(bind.face_id == CurveBindData::kInvalidFaceID) continue;

		const auto& face = faces[bind.face_id];
		const pxr::GfVec3f root_proj_pos = bind.u * pt_positions[face.indices[0]] + bind.v * pt_positions[face.indices[2]] + (1.f - bind.u - bind.v) * pt_positions[face.indices[1]];
		const pxr::GfVec3f tmp_binormal = face_center_points[bind.face_id] - root_proj_pos;
		mPerBindTBs[i].first = pxr::GfGetNormalized(pxr::GfCross(perBindNormals[i], tmp_binormal)); // tangent
		mPerBindTBs[i].second = pxr::GfGetNormalized(pxr::GfCross(perBindNormals[i], mPerBindTBs[i].first)); //binormal
	}

	return true;
}

void FastCurvesDeformer::transformCurvesToNTB() {
	assert(mpCurvesContainer);
	assert(mpPhantomTrimesh);
	assert(mCurveBinds.size() == mpCurvesContainer->getCurvesCount());

	for(uint32_t curve_index = 0; curve_index < mpCurvesContainer->getCurvesCount(); ++curve_index) {
		PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_index);
		const auto& bind = mCurveBinds[curve_index];
		if(bind.face_id == CurveBindData::kInvalidFaceID) continue;

		const auto root_pos_offset = mpCurvesContainer->getCurveRootPoint(curve_index) - mpPhantomTrimesh->getInterpolatedRestPosition(bind.face_id, bind.u, bind.v);
		const pxr::GfVec3f& N = mPerBindRestNormals[curve_index];
		const pxr::GfVec3f& T = mPerBindRestTBs[curve_index].first;
		const pxr::GfVec3f& B = mPerBindRestTBs[curve_index].second;
		const pxr::GfMatrix3f m = pxr::GfMatrix3f(
			N[0], T[0], B[0],
			N[1], T[1], B[1],
			N[2], T[2], B[2]
		).GetInverse();

		for(size_t j = 0; j < curve_data_ptr.first; ++j) {
			*(curve_data_ptr.second + j) = m * (*(curve_data_ptr.second + j) + root_pos_offset);
			//*(curve_data_ptr.second + j) = m.Transform(*(curve_data_ptr.second + j) + root_pos_offset);
		}
	}
}

bool FastCurvesDeformer::bindCurveToTriface(uint32_t curve_index, uint32_t face_id, CurveBindData& bind) {
	PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_index);
	// first we try to itersect one of the curves segments
	if(curve_data_ptr.first < 2 || !curve_data_ptr.second) return false; // invalid curve

	const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_index);

	for(uint32_t ptr_offset = 0; ptr_offset < static_cast<uint32_t>(curve_data_ptr.first - 1); ++ptr_offset) {
		const pxr::GfVec3f orig = curve_root_pt + *(curve_data_ptr.second + ptr_offset);
		const pxr::GfVec3f dir  = *(curve_data_ptr.second + ptr_offset + 1) - *(curve_data_ptr.second + ptr_offset); 
		if(mpPhantomTrimesh->intersectRay(orig, dir, face_id, bind.u, bind.v)) {
			bind.face_id = face_id;
			return true;
		}
	}

	// if we failed to intersect one of curves segments, then try to project one of its points starting from root
	for(uint32_t ptr_offset = 0; ptr_offset < static_cast<uint32_t>(curve_data_ptr.first); ++ptr_offset) {
		if(mpPhantomTrimesh->projectPoint(curve_root_pt, face_id, bind.u, bind.v)) {
			bind.face_id = face_id;
			return true;
		}
	}


	return false;
}

bool FastCurvesDeformer::buildCurvesBindingData(pxr::UsdTimeCode rest_time_code) {
	if(!mpCurvesContainer) return false;

	pxr::UsdGeomPrimvarsAPI curvesPrimvarsApi = mCurvesGeoPrimHandle.getPrimvarsAPI();

	const size_t total_curves_count = mpCurvesContainer->getCurvesCount();

	mCurveBinds.resize(total_curves_count);
	size_t skin_bound_curves_count = 0;
	size_t kdtree_bound_curves_count = 0;
	size_t bforce_bound_curves_count = 0;


	// Clear all possible previous binds
	for(auto& bind: mCurveBinds) {
		bind.face_id = CurveBindData::kInvalidFaceID;
	}

	std::vector<float> tmp_squared_distances(mpAdjacency->getMaxFaceVertexCount());

	auto bindCurveToPrim = [&] (uint32_t curve_index, CurveBindData& bind, uint32_t prim_id) {
		bool isBound = false;
		const uint32_t prim_vertex_count = mpAdjacency->getFaceVertexCount(prim_id);
		const uint32_t prim_vertex_offset = mpAdjacency->getFaceVertexOffset(prim_id);

		// try to bind face corner "ear"

		for(size_t j = 0; j < prim_vertex_count; ++j) {
			tmp_squared_distances[j] = distanceSquared(mpCurvesContainer->getCurveRootPoint(curve_index), mpPhantomTrimesh->getRestPositions()[mpAdjacency->getFaceVertex(prim_vertex_offset + j)]);
		}

		std::vector<float>::iterator it = std::min_element(tmp_squared_distances.begin(), tmp_squared_distances.begin() + prim_vertex_count);
		uint32_t local_index = std::distance(std::begin(tmp_squared_distances), it);

		const uint32_t face_id = mpPhantomTrimesh->getOrCreate(
			mpAdjacency->getFaceVertex(prim_id, (local_index + prim_vertex_count - 1) % prim_vertex_count),
			mpAdjacency->getFaceVertex(prim_id, local_index), 
			mpAdjacency->getFaceVertex(prim_id, (local_index + 1) % prim_vertex_count)
		);

		if(bindCurveToTriface(curve_index, face_id, bind)) {
			isBound = true;
		} else if ( prim_vertex_count > 3u){
			// Try to bind using face "fan" triangulation
			for(uint32_t i = 1; i < (prim_vertex_count - 1); ++i) {
				const uint32_t face_id = mpPhantomTrimesh->getOrCreate(
					mpAdjacency->getFaceVertex(prim_id, local_index), 
					mpAdjacency->getFaceVertex(prim_id, (local_index + i) % prim_vertex_count),
					mpAdjacency->getFaceVertex(prim_id, (local_index + i + 1) % prim_vertex_count)
				);

				if(bindCurveToTriface(curve_index, face_id, bind)) {
					isBound = true;
					break;
				} 
			}
		}

		return isBound;
	};

	dbg_printf("Total curves count to bind: %zu\n", total_curves_count);

	// Strategy: 1
	// First we try to bind curves using skin prim ids
	if(!mСurvesSkinPrimAttrName.empty()) {
		pxr::UsdGeomPrimvar skinPrimVar = curvesPrimvarsApi.GetPrimvar(pxr::TfToken(mСurvesSkinPrimAttrName));
		if(!skinPrimVar) {
			std::cerr << "Curves " << mCurvesGeoPrimHandle.getPath() << " has no valid primvar \"" << mСurvesSkinPrimAttrName << "\" !" << std::endl;
		} else {
			pxr::VtArray<int> skinPrimIndices;
			if(skinPrimVar.GetAttr().Get(&skinPrimIndices, rest_time_code)) {
				assert(skinPrimIndices.size() == total_curves_count);
				std::vector<float> squared_distances(mpAdjacency->getMaxFaceVertexCount());

				for(uint32_t curve_index = 0; curve_index < total_curves_count; ++curve_index) {
					if(skinPrimIndices[curve_index] < 0) continue; // pixar uses negative indices as invalid

					auto& bind = mCurveBinds[curve_index];
					const uint32_t skin_prim_id = static_cast<uint32_t>(skinPrimIndices[curve_index]);
					
					if(bindCurveToPrim(curve_index, bind, skin_prim_id)) {
						skin_bound_curves_count++;
						//dbg_printf("skin bound\n");
					}
				}
			}
		}
	}

	dbg_printf("Skin bound curves count: %zu\n", skin_bound_curves_count);

	if(total_curves_count == skin_bound_curves_count) {
		// all curves were bound using skinPrimVar
		return true;
	}

	// Strategy: 2
	// Bind ramaining curves using nearest point

	// Build kdtree
	static const bool threaded_kdtree_creation = false;
	neighbour_search::KDTree<float, 3> kdtree(mpPhantomTrimesh->getRestPositions(), threaded_kdtree_creation);

	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> nearest_points;

	for(uint32_t curve_index = 0; curve_index < total_curves_count; ++curve_index) {
		auto& bind = mCurveBinds[curve_index];
		if(bind.face_id != CurveBindData::kInvalidFaceID) continue; // bound already

		const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_index);
		const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = kdtree.findNearestNeighbour(curve_root_pt);

		const auto nearest_vtx = nearest_pt.first;
		const uint32_t neighbors_count = mpAdjacency->getNeighborsCount(nearest_vtx);
		if(neighbors_count > 0) {
			const uint32_t neighbors_offset = mpAdjacency->getNeighborsOffset(nearest_vtx);
			for(uint32_t prim_offset = neighbors_offset; prim_offset < (neighbors_offset + neighbors_count); ++prim_offset){
				const auto skin_prim_id = mpAdjacency->getNeighborPrim(prim_offset);
				if(bindCurveToPrim(curve_index, bind, skin_prim_id)) {
					//dbg_printf("kdtree bound\n");
					kdtree_bound_curves_count++;
					break;
				}
			}
		}
	}

	dbg_printf("KDtree bound curves count: %zu\n", kdtree_bound_curves_count);

	if(total_curves_count == (skin_bound_curves_count + kdtree_bound_curves_count)) {
		// all remaining curves were bound using kdtree
		return true;
	}

	// Strategy: 3
	// Brute force binding
	for(uint32_t curve_index = 0; curve_index < total_curves_count; ++curve_index) {
		auto& bind = mCurveBinds[curve_index];
		if(bind.face_id != CurveBindData::kInvalidFaceID) continue; // bound already

		for(const auto& skin_prim_id: mpAdjacency->getPrimData()) {
			if(bindCurveToPrim(curve_index, bind, skin_prim_id)) {
				//dbg_printf("bforce bound\n");
				bforce_bound_curves_count++;
				break;
			}
		}

	}

	dbg_printf("Brute force bounf curves count: %zu\n", bforce_bound_curves_count);

	return true;
}

FastCurvesDeformer::~FastCurvesDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston