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
	mpFastCurvesDeformerData = std::make_unique<FastCurvesDeformerData>();
}

FastCurvesDeformer::SharedPtr FastCurvesDeformer::create() {
	return SharedPtr(new FastCurvesDeformer());
}

const std::string& FastCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "FastCurvesDeformer";
	return kFastDeformerString;
}

bool FastCurvesDeformer::deformImpl(pxr::UsdTimeCode time_code) {
	return __deform__(false, time_code);
}

bool FastCurvesDeformer::deformMtImpl(pxr::UsdTimeCode time_code) {
	return __deform__(true, time_code);
}

bool FastCurvesDeformer::__deform__(bool multi_threaded, pxr::UsdTimeCode time_code) {
	PROFILE("FastCurvesDeformer::__deform__");

	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();

	if(!pPhantomTrimesh || !pPhantomTrimesh->update(mMeshGeoPrimHandle, time_code)) {
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

	const auto& curveBinds = mpFastCurvesDeformerData->mCurveBinds;

	assert(curveBinds.size() == pCurves->getCurvesCount());

	auto func = [&](const std::size_t start, const std::size_t end) {
		for(size_t i = start; i < end; ++i) {
			const auto& bind = curveBinds[i];
			if(bind.face_id == CurveBindData::kInvalidFaceID) continue;

			const pxr::GfVec3f& N = mPerBindLiveNormals[i];
			const pxr::GfVec3f& T = mPerBindLiveTBs[i].first;
			const pxr::GfVec3f& B = mPerBindLiveTBs[i].second;

			const pxr::GfMatrix3f m = {
				N[0], T[0], B[0], 
				N[1], T[1], B[1],
				N[2], T[2], B[2]
			};

			auto curve_bind_pos = pPhantomTrimesh->getInterpolatedLivePosition(bind.face_id, bind.u, bind.v);
			uint32_t vertex_offset = pCurves->getCurveVertexOffset(i);
			PxrCurvesContainer::CurveDataPtr curve_data_ptr = pCurves->getCurveDataPtr(i);

			for(size_t j = 0; j < curve_data_ptr.first; ++j) {
				points[vertex_offset++] = curve_bind_pos + m * (*(curve_data_ptr.second + j));
			}
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, curveBinds.size(), func);
		mPool.wait();
	} else {
		func(0u, curveBinds.size());
	}

	if(!curves.GetPointsAttr().Set(pCurves->getPointsCacheVtArray(), time_code)) {
		return false;
	}

	return true;
}

bool FastCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(!mCurvesGeoPrimHandle.writeDataToBson(mpFastCurvesDeformerData.get())) {
		std::cerr << "Error writing " << mpFastCurvesDeformerData->typeName() << " deformer data to json !";	
		return false;
	}
	return true;
}

bool FastCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code) {
	PROFILE("FastCurvesDeformer::buildDeformerDataImpl");
	dbg_printf("FastCurvesDeformer::buildDeformerDataImpl()\n");

	assert(mpAdjacencyData);
	const auto* pAdjacency = mpAdjacencyData->getAdjacency();
	if(!pAdjacency->isValid()) {
		std::cerr << "No valid mesh adjacency data!" << std::endl;
		return false;
	}

	assert(pAdjacency->getMaxFaceVertexCount() > 0);


	if(!mpFastCurvesDeformerData) {
		mpFastCurvesDeformerData = std::make_unique<FastCurvesDeformerData>();
	}
	mpFastCurvesDeformerData->clear();

	if(!getReadJsonDataState() || !mCurvesGeoPrimHandle.getDataFromBson(mpFastCurvesDeformerData.get())) {
		// Build deformer data in place if no json data present or not needed
		
		if(!buildCurvesBindingData(rest_time_code)) {
			return false;
		}

		const size_t binds_count = mpFastCurvesDeformerData->getCurveBinds().size();

		mpFastCurvesDeformerData->mRestVertexNormals.resize(binds_count);
		mpFastCurvesDeformerData->mPerBindRestNormals.resize(binds_count);
		mpFastCurvesDeformerData->mPerBindRestTBs.resize(binds_count);

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
	
		mpFastCurvesDeformerData->setPopulated(true);
	}

	mLiveVertexNormals.resize(mpFastCurvesDeformerData->getRestVertexNormals().size());
	mPerBindLiveNormals.resize(mpFastCurvesDeformerData->getPerBindRestNormals().size());
	mPerBindLiveTBs.resize(mpFastCurvesDeformerData->getPerBindRestTBs().size());

	// transform curves to NTB spaces
	transformCurvesToNTB();

	return true; 
}

bool FastCurvesDeformer::calcPerBindNormals(bool build_live) {
	assert(mpAdjacencyData);
	const auto* pAdjacency = mpAdjacencyData->getAdjacency();
	assert(pAdjacency);

	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	if(!pAdjacency || !pPhantomTrimesh) return false;

	std::vector<pxr::GfVec3f>& vertex_normals = build_live ? mLiveVertexNormals : mpFastCurvesDeformerData->mRestVertexNormals;

	buildVertexNormals(pAdjacency, pPhantomTrimesh, vertex_normals, build_live);

	// Build per bind normals
	auto& perBindNormals = build_live ? mPerBindLiveNormals : mpFastCurvesDeformerData->mPerBindRestNormals;
	const auto& curveBinds = mpFastCurvesDeformerData->getCurveBinds();
	assert(perBindNormals.size() == curveBinds.size());

	for(size_t i = 0; i < curveBinds.size(); ++i) {
		const auto& bind = curveBinds[i];
		
		if(bind.face_id == CurveBindData::kInvalidFaceID) continue;

		const auto& face = pPhantomTrimesh->getFace(bind.face_id);
		perBindNormals[i] = pxr::GfGetNormalized(bind.u * vertex_normals[face.indices[1]] + bind.v * vertex_normals[face.indices[2]] + (1.f - bind.u - bind.v) * vertex_normals[face.indices[0]]);
	}

	return true;
}

bool FastCurvesDeformer::calcPerBindTangentsAndBiNormals(bool build_live) {
	static constexpr float kF = 1.f / 3.f;

	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	auto& perBindNormals = build_live ? mPerBindLiveNormals : mpFastCurvesDeformerData->mPerBindRestNormals;

	const auto& curveBinds = mpFastCurvesDeformerData->getCurveBinds();
	assert(perBindNormals.size() == curveBinds.size());

	auto& mPerBindTBs = build_live ? mPerBindLiveTBs : mpFastCurvesDeformerData->mPerBindRestTBs;
	assert(mPerBindTBs.size() == curveBinds.size());

	const std::vector<pxr::GfVec3f>& pt_positions = build_live ? pPhantomTrimesh->getLivePositions() : pPhantomTrimesh->getRestPositions();
	const std::vector<PhantomTrimesh::TriFace>& faces = pPhantomTrimesh->getFaces(); 

	pxr::VtArray<pxr::GfVec3f> face_center_points(faces.size());

	for(size_t i = 0; i < faces.size(); ++i) {
		const auto& face = faces[i];
		face_center_points[i] = (pt_positions[face.indices[0]] + pt_positions[face.indices[1]] + pt_positions[face.indices[2]]) * kF;
	}

	for(size_t i = 0; i < curveBinds.size(); ++i) {
		const auto& bind = curveBinds[i];
		
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

	const auto& curveBinds = mpFastCurvesDeformerData->getCurveBinds();
	assert(curveBinds.size() == mpCurvesContainer->getCurvesCount());

	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	const auto& perBindRestNormals = mpFastCurvesDeformerData->getPerBindRestNormals();
	const auto& perBindRestTBs = mpFastCurvesDeformerData->getPerBindRestTBs();

	for(uint32_t curve_index = 0; curve_index < mpCurvesContainer->getCurvesCount(); ++curve_index) {
		PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_index);
		const auto& bind = curveBinds[curve_index];
		if(bind.face_id == CurveBindData::kInvalidFaceID) continue;

		const auto root_pos_offset = mpCurvesContainer->getCurveRootPoint(curve_index) - pPhantomTrimesh->getInterpolatedRestPosition(bind.face_id, bind.u, bind.v);
		const pxr::GfVec3f& N = perBindRestNormals[curve_index];
		const pxr::GfVec3f& T = perBindRestTBs[curve_index].first;
		const pxr::GfVec3f& B = perBindRestTBs[curve_index].second;
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
	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_index);
	// first we try to itersect one of the curves segments
	if(curve_data_ptr.first < 2 || !curve_data_ptr.second) return false; // invalid curve

	const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_index);

	for(uint32_t ptr_offset = 0; ptr_offset < static_cast<uint32_t>(curve_data_ptr.first - 1); ++ptr_offset) {
		const pxr::GfVec3f orig = curve_root_pt + *(curve_data_ptr.second + ptr_offset);
		const pxr::GfVec3f dir  = *(curve_data_ptr.second + ptr_offset + 1) - *(curve_data_ptr.second + ptr_offset); 
		float isect_dist;
		if(pPhantomTrimesh->intersectRay(orig, dir, face_id, bind.u, bind.v, isect_dist)) {
			// check we've intersected within curve segment
			if((isect_dist*isect_dist) <= lengthSquared(dir)) {
				bind.face_id = face_id;
				return true;
			}
		}
	}

	// if we failed to intersect one of curves segments, then try to project one of its points starting from root
	for(uint32_t ptr_offset = 0; ptr_offset < static_cast<uint32_t>(curve_data_ptr.first); ++ptr_offset) {
		if(pPhantomTrimesh->projectPoint(curve_root_pt, face_id, bind.u, bind.v)) {
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

	auto& curveBinds = mpFastCurvesDeformerData->mCurveBinds;

	curveBinds.resize(total_curves_count);
	size_t skin_bound_curves_count = 0;
	size_t kdtree_bound_curves_count = 0;
	size_t bforce_bound_curves_count = 0;


	// Clear all possible previous binds
	for(auto& bind: curveBinds) {
		bind.face_id = CurveBindData::kInvalidFaceID;
	}

	assert(mpAdjacencyData);
	const auto* pAdjacency = mpAdjacencyData->getAdjacency();
	assert(pAdjacency);

	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	std::vector<float> tmp_squared_distances(pAdjacency->getMaxFaceVertexCount());

	auto bindCurveToPrim = [&] (uint32_t curve_index, CurveBindData& bind, uint32_t prim_id) {
		bool isBound = false;
		const uint32_t prim_vertex_count = pAdjacency->getFaceVertexCount(prim_id);
		const uint32_t prim_vertex_offset = pAdjacency->getFaceVertexOffset(prim_id);

		// try to bind face corner "ear"

		for(size_t j = 0; j < prim_vertex_count; ++j) {
			tmp_squared_distances[j] = distanceSquared(mpCurvesContainer->getCurveRootPoint(curve_index), pPhantomTrimesh->getRestPositions()[pAdjacency->getFaceVertex(prim_vertex_offset + j)]);
		}

		std::vector<float>::iterator it = std::min_element(tmp_squared_distances.begin(), tmp_squared_distances.begin() + prim_vertex_count);
		uint32_t local_index = std::distance(std::begin(tmp_squared_distances), it);

		const uint32_t face_id = pPhantomTrimesh->getOrCreate(
			pAdjacency->getFaceVertex(prim_id, (local_index + prim_vertex_count - 1) % prim_vertex_count),
			pAdjacency->getFaceVertex(prim_id, local_index), 
			pAdjacency->getFaceVertex(prim_id, (local_index + 1) % prim_vertex_count)
		);

		if(bindCurveToTriface(curve_index, face_id, bind)) {
			isBound = true;
		} else if ( prim_vertex_count > 3u){
			// Try to bind using face "fan" triangulation
			for(uint32_t i = 1; i < (prim_vertex_count - 1); ++i) {
				const uint32_t face_id = pPhantomTrimesh->getOrCreate(
					pAdjacency->getFaceVertex(prim_id, local_index), 
					pAdjacency->getFaceVertex(prim_id, (local_index + i) % prim_vertex_count),
					pAdjacency->getFaceVertex(prim_id, (local_index + i + 1) % prim_vertex_count)
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
				std::vector<float> squared_distances(pAdjacency->getMaxFaceVertexCount());

				for(uint32_t curve_index = 0; curve_index < total_curves_count; ++curve_index) {
					if(skinPrimIndices[curve_index] < 0) continue; // pixar uses negative indices as invalid

					auto& bind = curveBinds[curve_index];
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
	neighbour_search::KDTree<float, 3> kdtree(pPhantomTrimesh->getRestPositions(), threaded_kdtree_creation);

	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> nearest_points;

	for(uint32_t curve_index = 0; curve_index < total_curves_count; ++curve_index) {
		auto& bind = curveBinds[curve_index];
		if(bind.face_id != CurveBindData::kInvalidFaceID) continue; // bound already

		const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_index);
		const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = kdtree.findNearestNeighbour(curve_root_pt);

		const auto nearest_vtx = nearest_pt.first;
		const uint32_t neighbors_count = pAdjacency->getNeighborsCount(nearest_vtx);
		if(neighbors_count > 0) {
			const uint32_t neighbors_offset = pAdjacency->getNeighborsOffset(nearest_vtx);
			for(uint32_t prim_offset = neighbors_offset; prim_offset < (neighbors_offset + neighbors_count); ++prim_offset){
				const auto skin_prim_id = pAdjacency->getNeighborPrim(prim_offset);
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
		auto& bind = curveBinds[curve_index];
		if(bind.face_id != CurveBindData::kInvalidFaceID) continue; // bound already

		for(const auto& skin_prim_id: pAdjacency->getPrimData()) {
			if(bindCurveToPrim(curve_index, bind, skin_prim_id)) {
				//dbg_printf("bforce bound\n");
				bforce_bound_curves_count++;
				break;
			}
		}

	}

	dbg_printf("Brute force bound curves count: %zu\n", bforce_bound_curves_count);

	return true;
}

FastCurvesDeformer::~FastCurvesDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston