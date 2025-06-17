#include "fast_curves_deformer.h"

#include "common.h"
#include "kdtree.hpp"
#include "geometry_tools.h"
#include "simple_profiler.h"

#include <random>
#include <algorithm>

static inline float distanceSquared(const pxr::GfVec3f &p1, const pxr::GfVec3f &p2) {
	const auto dx = p1[0] - p2[0];
	const auto dy = p1[1] - p2[1];
	const auto dz = p1[2] - p2[2];
	return (dx * dx) + (dy * dy) + (dz * dz);
}

namespace Piston {

FastCurvesDeformer::FastCurvesDeformer(): BaseCurvesDeformer(), mDeformMode(DeformMode::FACET) {
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

	if(!pCurves || pCurves->getCurvesCount() == 0) {
		return false;
	}

	if(mDeformMode != DeformMode::FACET) {
		const bool build_live = true; // build using live data
		if(!calcPerBindNormals(build_live)) {
			std::cerr << "Error building interpolated live normals !" << std::endl;
			return false;
		}

		if(mDeformMode == DeformMode::ACCURATE) {
			calcPerBindTangentsAndBiNormals(build_live);
		}
	}

	pxr::VtArray<pxr::GfVec3f> points(pCurves->getTotalVertexCount());

	assert(mCurveBinds.size() == pCurves->getCurvesCount());

	for(uint32_t i = 0; i < pCurves->getCurvesCount(); ++i) {
		const auto& bind = mCurveBinds[i];
		if(bind.face_id == CurveBindData::kInvalidFaceID) {
			//dbg_printf("invalid face_id\n");
			continue;
		}

		const auto& face = mpPhantomTrimesh->getFace(bind.face_id);
		pxr::GfMatrix3f rotate_mat;

		if(mDeformMode == DeformMode::ACCURATE) {
		const pxr::GfVec3f& N = mPerBindLiveNormals[i];
		const pxr::GfVec3f& T = mPerBindLiveTBs[i].first;
		const pxr::GfVec3f& B = mPerBindLiveTBs[i].second;

		const pxr::GfMatrix3f m = {
			N[0], T[0], B[0], 
			N[1], T[1], B[1],
			N[2], T[2], B[2]
		};

			rotate_mat = m;//.GetInverse();
		} else {
			rotate_mat = mDeformMode == DeformMode::FACET ? rotateAlign(face.getRestNormal(), face.getLiveNormal()) : rotateAlign(mPerBindRestNormals[i], mPerBindLiveNormals[i]);
		}


		// try to move curve root point
		//std::cout << bind.dist << std::endl;

		auto root_pt_pos = mpPhantomTrimesh->getInterpolatedPosition(bind.face_id, bind.u, bind.v, bind.dist);
		uint32_t vertex_offset = pCurves->getCurveVertexOffset(i);
		points[vertex_offset++] = root_pt_pos;
		PxrCurvesContainer::CurveDataPtr curve_data_ptr = pCurves->getCurveDataPtr(i);

		for(size_t j = 1; j < curve_data_ptr.first; ++j) {
			points[vertex_offset++] = root_pt_pos + rotate_mat * (*(curve_data_ptr.second + j));
		}
	}

	if(!curves.GetPointsAttr().Set(points, time_code)) {
		return false;
	}

	return true;
}

void FastCurvesDeformer::setDeformMode(FastCurvesDeformer::DeformMode mode) {
	if(mDeformMode == mode) return;
	mDeformMode = mode;
	mDirty = true;
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

	std::cout << mpAdjacency->toString() << std::endl;

	// Create phantom mesh
	mpPhantomTrimesh = PhantomTrimesh<PxrIndexType>::create(mMeshGeoPrimHandle, mMeshRestPositionAttrName);
	if(!mpPhantomTrimesh) {
		std::cerr << "Error creating phantom trimesh for " << mMeshGeoPrimHandle.getPath() << " !" << std::endl;
		return false;
	}

	if(!buildCurvesBindingData(rest_time_code)) {
		return false;
	}

	// We've done for simplest FACET mode
	if(mDeformMode == DeformMode::FACET) {
		return true;
	}

	// Additional per-bind rest normals if needed
	const bool build_live_per_bind_data = false; // build using rest data
	if(!calcPerBindNormals(build_live_per_bind_data)) {
		std::cerr << "Error building per curve " << (build_live_per_bind_data ? "\"live\"" : "\"rest\"") << " normals !" << std::endl;
		return false;
	}

	if(mDeformMode == DeformMode::ACCURATE) {
		if(!calcPerBindTangentsAndBiNormals(build_live_per_bind_data)) {
			std::cerr << "Error building per curve " << (build_live_per_bind_data ? "\"live\"" : "\"rest\"") << " tangents and binormals !" << std::endl;
			return false;
		}
		transformCurvesToNTB();
	}

	return true; 
}

bool FastCurvesDeformer::calcPerBindNormals(bool build_live) {
	assert(mpPhantomTrimesh);
	assert(mpAdjacency);

	if(!mpAdjacency || !mpPhantomTrimesh) return false;

	const pxr::VtArray<pxr::GfVec3f>& pt_positions = build_live ? mpPhantomTrimesh->getLivePositions() : mpPhantomTrimesh->getRestPositions();

	pxr::VtArray<pxr::GfVec3f> vertex_normals(mpAdjacency->getVertexCount());

	const auto& faces = mpPhantomTrimesh->getFaces();
	std::set<PxrIndexType> vertices;
	for(const auto& face: faces) {
		vertices.insert(face.indices[0]);
		vertices.insert(face.indices[1]);
		vertices.insert(face.indices[2]);
	}

	for(PxrIndexType vtx: vertices) {
		pxr::GfVec3f vn = {0.f, 0.f, 0.f};

		const uint32_t edges_count = mpAdjacency->getNeighborsCount(vtx);
		const uint32_t vtx_offset = mpAdjacency->getNeighborsOffset(vtx);
		
		for(uint32_t i = 0; i < edges_count; ++i) {
			const auto& vtx_pair = mpAdjacency->getCornerVertexPair(vtx_offset + i);
			vn += pxr::GfGetNormalized(pxr::GfCross(pt_positions[vtx_pair.first] - pt_positions[vtx], pt_positions[vtx_pair.second] - pt_positions[vtx]));
		}

		vertex_normals[vtx] = pxr::GfGetNormalized(vn);
	}

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
	PxrCurvesContainer* pCurves = mpCurvesContainer.get();

	for(size_t i = 0; i < mCurveBinds.size(); ++i) {
		PxrCurvesContainer::CurveDataPtr curve_data_ptr = pCurves->getCurveDataPtr(i);

		const pxr::GfVec3f& N = mPerBindRestNormals[i];
		const pxr::GfVec3f& T = mPerBindRestTBs[i].first;
		const pxr::GfVec3f& B = mPerBindRestTBs[i].second;

		const pxr::GfMatrix3f m = pxr::GfMatrix3f(
			N[0], T[0], B[0], 
			N[1], T[1], B[1],
			N[2], T[2], B[2]).GetInverse();

		for(size_t j = 0; j < curve_data_ptr.first; ++j) {
			*(curve_data_ptr.second + j) = m * (*(curve_data_ptr.second + j));
		}
	}
}

bool FastCurvesDeformer::buildCurvesBindingData(pxr::UsdTimeCode rest_time_code) {
	if(!mpCurvesContainer) return false;

	pxr::UsdGeomPrimvarsAPI curvesPrimvarsApi = mCurvesGeoPrimHandle.getPrimvarsAPI();

	mCurveBinds.resize(mpCurvesContainer->getCurvesCount());
	size_t bound_curves_count = 0;
	size_t skin_bound_curves_count = 0;


	// Clear all possible previous binds
	for(auto& bind: mCurveBinds) {
		bind.face_id = CurveBindData::kInvalidFaceID;
	}

	std::vector<float> tmp_squared_distances(mpAdjacency->getMaxFaceVertexCount());

	auto bindToPrimIndex = [&] (CurveBindData& bind, const pxr::GfVec3f curve_root_pt, uint32_t prim_id) {
		bool isBound = false;
		const uint32_t prim_vertex_count = mpAdjacency->getFaceVertexCount(prim_id);
		const uint32_t prim_vertex_offset = mpAdjacency->getFaceVertexOffset(prim_id);

		// try to bind face corner "ear"

		for(size_t j = 0; j < prim_vertex_count; ++j) {
			tmp_squared_distances[j] = distanceSquared(curve_root_pt, mpPhantomTrimesh->getRestPositions()[mpAdjacency->getFaceVertex(prim_vertex_offset + j)]);
		}

		std::vector<float>::iterator it = std::min_element(tmp_squared_distances.begin(), tmp_squared_distances.begin() + prim_vertex_count);
		uint32_t local_index = std::distance(std::begin(tmp_squared_distances), it);

		const uint32_t face_id = mpPhantomTrimesh->getOrCreate(
			mpAdjacency->getFaceVertex(prim_id, (local_index + prim_vertex_count - 1) % prim_vertex_count),
			mpAdjacency->getFaceVertex(prim_id, local_index), 
			mpAdjacency->getFaceVertex(prim_id, (local_index + 1) % prim_vertex_count)
		);

		if(mpPhantomTrimesh->projectPoint(curve_root_pt, face_id, bind.u, bind.v, bind.dist)) {
			bind.face_id = face_id;
			isBound = true;
		} else if ( prim_vertex_count > 3u){
			// Try to bind using face "fan" triangulation
			for(uint32_t i = 1; i < (prim_vertex_count - 2); ++i) {
				const uint32_t face_id = mpPhantomTrimesh->getOrCreate(
					mpAdjacency->getFaceVertex(prim_id, local_index), 
					mpAdjacency->getFaceVertex(prim_id, (local_index + i) % prim_vertex_count),
					mpAdjacency->getFaceVertex(prim_id, (local_index + i + 1) % prim_vertex_count)
				);

				if(mpPhantomTrimesh->projectPoint(curve_root_pt, face_id, bind.u, bind.v, bind.dist)) {
					bind.face_id = face_id;
					isBound = true;
					break;
				} 
			}
		}
	
		return isBound;
	};

	// Strategy: 1

	// First we try to bind curves using skin prim ids
	if(!mСurvesSkinPrimAttrName.empty()) {
		pxr::UsdGeomPrimvar skinPrimVar = curvesPrimvarsApi.GetPrimvar(pxr::TfToken(mСurvesSkinPrimAttrName));
		if(!skinPrimVar) {
			std::cerr << "Curves " << mCurvesGeoPrimHandle.getPath() << " has no valid primvar \"" << mСurvesSkinPrimAttrName << "\" !" << std::endl;
		} else {
			pxr::VtArray<int> skinPrimIndices;
			if(skinPrimVar.GetAttr().Get(&skinPrimIndices, rest_time_code)) {
				assert(skinPrimIndices.size() == mpCurvesContainer->getCurvesCount());
				std::vector<float> squared_distances(mpAdjacency->getMaxFaceVertexCount());

				for(size_t i = 0; i < mpCurvesContainer->getCurvesCount(); ++i) {
					if(skinPrimIndices[i] < 0) continue; // pixar uses negative indices as invalid

					auto& bind = mCurveBinds[i];
					const pxr::GfVec3f curve_root_pt = mpCurvesContainer->getCurveRootPoint(i);
					const uint32_t skin_prim_id = static_cast<uint32_t>(skinPrimIndices[i]);
					
					if(bindToPrimIndex(bind, curve_root_pt, skin_prim_id)) {
						skin_bound_curves_count += 1;
						printf("skin bound\n");
					}
				}
			}
		}
	}

	// Build kdtree
	static const bool threaded_kdtree_creation = false;
	neighbour_search::KDTree<float, 3> kdtree(mpPhantomTrimesh->getRestPositions(), threaded_kdtree_creation);

	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> nearest_points;

	for(size_t i = 0; i < mpCurvesContainer->getCurvesCount(); ++i) {

		const pxr::GfVec3f curve_root_pt = mpCurvesContainer->getCurveRootPoint(i);

		nearest_points.clear();
		kdtree.findKNearestNeighbours(curve_root_pt, 3, nearest_points);
		if(nearest_points.size() < 3) {
			continue;
		}

		const uint32_t face_id = mpPhantomTrimesh->getOrCreate(
			static_cast<PxrIndexType>(nearest_points[0].first), static_cast<PxrIndexType>(nearest_points[1].first), static_cast<PxrIndexType>(nearest_points[2].first));
	
		auto& bind = mCurveBinds[i];
		if(bind.face_id != CurveBindData::kInvalidFaceID) continue;

		if(mpPhantomTrimesh->projectPoint(curve_root_pt, face_id, bind.u, bind.v, bind.dist)) {
			dbg_printf("projected\n");
			bound_curves_count++;
			bind.face_id = face_id;
		} else {
			auto closest_vtx = nearest_points[0].first;
			const uint32_t neighbors_count = mpAdjacency->getNeighborsCount(closest_vtx);
			if(neighbors_count > 0) {
				const uint32_t neighbors_offset = mpAdjacency->getNeighborsOffset(closest_vtx);
				for(uint32_t i = neighbors_offset; i < (neighbors_offset + neighbors_count); ++i){
					const auto& index_pair = mpAdjacency->getCornerVertexPair(i);
					const uint32_t face_id = mpPhantomTrimesh->getOrCreate(
						static_cast<PxrIndexType>(closest_vtx), static_cast<PxrIndexType>(index_pair.first), static_cast<PxrIndexType>(index_pair.second));
					if(mpPhantomTrimesh->projectPoint(curve_root_pt, face_id, bind.u, bind.v, bind.dist)) {
						dbg_printf("re-projected\n");
						bind.face_id = face_id;
						bound_curves_count++;
						break;
					}
				}
			} else {
				dbg_printf("skipped\n");
			}
		}
	}

	dbg_printf("Total curves count to bind: %zu\n", mpCurvesContainer->getCurvesCount());
	dbg_printf("Skin bound curves count: %zu\n", skin_bound_curves_count);
	dbg_printf("Bound curves count: %zu\n", bound_curves_count);
	
	return true;
}

FastCurvesDeformer::~FastCurvesDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston