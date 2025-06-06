#include "fast_curves_deformer.h"

#include "common.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

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
	dbg_printf("FastCurvesDeformer::deformImpl()\n");
	if(!mpPhantomTrimesh) return false;

	if(!mpPhantomTrimesh->update(mMeshGeoPrimHandle, time_code)) {
		return false;
	}

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	PxrCurvesContainer* pCurves = mpCurvesContainer.get();

	pxr::VtArray<pxr::GfVec3f> points(pCurves->getTotalVertexCount());

	assert(mCurveBinds.size() == pCurves->getCurvesCount());

	for(uint32_t i = 0; i < pCurves->getCurvesCount(); ++i) {
		const auto& bind = mCurveBinds[i];
		if(bind.face_id == CurveBindData::kInvalidFaceID) {
			dbg_printf("invalid face_id\n");
			continue;
		}

		const auto& face = mpPhantomTrimesh->getFace(bind.face_id);
		const pxr::GfMatrix3f rotate_mat = rotateAlign(face.getRestNormal(), face.getLiveNormal());

		// try to move curve root point
		//std::cout << bind.dist << std::endl;

		auto root_pt_pos = mpPhantomTrimesh->getInterpolatedPosition(bind.face_id, bind.u, bind.v, bind.dist);
		uint32_t vertex_offset = pCurves->getCurveVertexOffset(i);
		points[vertex_offset++] = root_pt_pos;
		PxrCurvesContainer::CurveDataPtr curve_data_ptr = pCurves->getCurveDataPtr(i);

		for(size_t j = 1; j < curve_data_ptr.first; ++j) {
			points[vertex_offset++] = root_pt_pos + rotate_mat * (*(curve_data_ptr.second + j));
			//points[vertex_offset++] = root_pt_pos + (*(curve_data_ptr.second + j));
		}
	}

	if(!curves.GetPointsAttr().Set(points, time_code)) {
		return false;
	}

	return true;
}

bool FastCurvesDeformer::buildDeformerData(pxr::UsdTimeCode rest_time_code) {
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

	return buildCurvesBindingData(rest_time_code);
}

bool FastCurvesDeformer::buildCurvesBindingData(pxr::UsdTimeCode rest_time_code) {
	if(!mpCurvesContainer) return false;

	pxr::UsdGeomPrimvarsAPI curvesPrimvarsApi = mCurvesGeoPrimHandle.getPrimvarsAPI();

	mCurveBinds.resize(mpCurvesContainer->getCurvesCount());
	size_t bound_curves_count = 0;

	// Clear all possible previous binds
	for(auto& bind: mCurveBinds) {
		bind.face_id = CurveBindData::kInvalidFaceID;
	}

	// Strategy: 1

	// First we try to bind curves using skin prim ids
	if(!mСurvesSkinPrimAttrName.empty()) {
		pxr::UsdGeomPrimvar skinPrimVar = curvesPrimvarsApi.GetPrimvar(pxr::TfToken(mСurvesSkinPrimAttrName));
		if(!skinPrimVar) {
			std::cerr << "Curves " << mCurvesGeoPrimHandle.getPath() << " has no valid primvar \"" << mСurvesSkinPrimAttrName << "\" !" << std::endl;
		} else {
			pxr::VtArray<int> skinPrimIndices;
			if(!skinPrimVar.GetAttr().Get(&skinPrimIndices, rest_time_code)) {
				std::cerr << "Error getting skin primitive indices for attr \"" << mСurvesSkinPrimAttrName << "\" !" << std::endl;
			} else {
				assert(skinPrimIndices.size() == mpCurvesContainer->getCurvesCount());
				std::vector<float> squared_distances(mpAdjacency->getMaxFaceVertexCount());

				for(size_t i = 0; i < mpCurvesContainer->getCurvesCount(); ++i) {
					auto& bind = mCurveBinds[i];
					const pxr::GfVec3f curve_root_pt = mpCurvesContainer->getCurveRootPoint(i);
					const uint32_t skin_prim_id = static_cast<uint32_t>(skinPrimIndices[i]);
					const uint32_t prim_vertex_count = mpAdjacency->getFaceVertexCount(skin_prim_id);
					const uint32_t prim_vertex_offset = mpAdjacency->getFaceVertexOffset(skin_prim_id);

					for(size_t j = 0; j < prim_vertex_count; ++j) {
						squared_distances[j] = distanceSquared(curve_root_pt, mpPhantomTrimesh->getRestPositions()[mpAdjacency->getFaceVertex(prim_vertex_offset + j)]);
					}

					std::vector<float>::iterator it = std::min_element(squared_distances.begin(), squared_distances.begin() + prim_vertex_count);
					uint32_t local_index = std::distance(std::begin(squared_distances), it);
					dbg_printf("curve %zu face %u closest vtx %u\n", i, skin_prim_id, mpAdjacency->getFaceVertex(prim_vertex_offset + local_index));

					
					const uint32_t face_id = mpPhantomTrimesh->getOrCreate(
						mpAdjacency->getFaceVertex(skin_prim_id, (static_cast<int>(local_index) + prim_vertex_count - 1) % prim_vertex_count),
						mpAdjacency->getFaceVertex(skin_prim_id, local_index), 
						mpAdjacency->getFaceVertex(skin_prim_id, (static_cast<int>(local_index) + 1) % prim_vertex_count)
					);

					if(mpPhantomTrimesh->projectPoint(curve_root_pt, face_id, bind.u, bind.v, bind.dist)) {
						dbg_printf("skin projected\n");
						bind.face_id = face_id;
						bound_curves_count++;
					}
					
					/*
					dbg_printf("[%u: %d %d %d]\n", 
						prim_vertex_count,
						mpAdjacency->getFaceVertex(skin_prim_id, (static_cast<int>(local_index) + prim_vertex_count - 1) % prim_vertex_count),
						mpAdjacency->getFaceVertex(skin_prim_id, local_index), 
						mpAdjacency->getFaceVertex(skin_prim_id, (static_cast<int>(local_index) + 1) % prim_vertex_count)
					);
					*/
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
	dbg_printf("Bound curves count: %zu\n", bound_curves_count);
	
	return true;
}

} // namespace Piston