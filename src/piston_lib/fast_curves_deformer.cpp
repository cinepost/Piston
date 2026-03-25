#include "simple_profiler.h"
#include "fast_curves_deformer.h"

#include "common.h"
#include "logging.h"
#include "kdtree.hpp"
#include "geometry_tools.h"
//#include "simple_profiler.h"

#include <pxr/base/gf/matrix4f.h>

#include <random>
#include <algorithm>
#include <cmath>


static inline float distanceSquared(const pxr::GfVec3f &p1, const pxr::GfVec3f &p2) {
	const auto dx = p1[0] - p2[0];
	const auto dy = p1[1] - p2[1];
	const auto dz = p1[2] - p2[2];
	return (dx * dx) + (dy * dy) + (dz * dz);
}

namespace Piston {

FastCurvesDeformer::FastCurvesDeformer(const std::string& name): BaseMeshCurvesDeformer(BaseCurvesDeformer::Type::FAST, name) {
	LOG_TRC << "FastCurvesDeformer::FastCurvesDeformer(" << getName() << ")";
	mpFastCurvesDeformerData = std::make_unique<FastCurvesDeformerData>();

	mpDebugGeo = DebugGeo::create(name);
}

FastCurvesDeformer::SharedPtr FastCurvesDeformer::create(const std::string& name) {
	return SharedPtr(new FastCurvesDeformer(name));
}

const std::string& FastCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "FastCurvesDeformer";
	return kFastDeformerString;
}

bool FastCurvesDeformer::deformImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, false, time_code);
}

bool FastCurvesDeformer::deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, true, time_code);
}

bool FastCurvesDeformer::__deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code) {
	PROFILE("FastCurvesDeformer::__deform__");

	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();

	if(!pPhantomTrimesh || !pPhantomTrimesh->update(mDeformerGeoPrimHandle, time_code)) {
		return false;
	}

	assert(mpAdjacencyData);
	const auto* pAdjacency = mpAdjacencyData->getAdjacency();
	assert(pAdjacency);

	if(!pAdjacency) {
		return false;
	}

	const bool build_live = true; // build using live data
	std::vector<pxr::GfVec3f>& vertex_normals = build_live ? mLiveVertexNormals : mpFastCurvesDeformerData->mRestVertexNormals;

	buildVertexNormals(pAdjacency, pPhantomTrimesh, vertex_normals, build_live, (multi_threaded ? &mPool : nullptr));
	calcPerBindNormals(pAdjacency, pPhantomTrimesh, vertex_normals, build_live, (multi_threaded ? &mPool : nullptr));
	calcPerBindTangentsAndBiNormals(pPhantomTrimesh, build_live, (multi_threaded ? &mPool : nullptr));

	const auto& curveBinds = mpFastCurvesDeformerData->mCurveBinds;

	assert(curveBinds.size() == mpCurvesContainer->getCurvesCount());

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
			uint32_t vertex_offset = mpCurvesContainer->getCurveVertexOffset(i);
			PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(i);

			for(size_t j = 0; j < curve_data_ptr.first; ++j) {
				points[vertex_offset++] = curve_bind_pos + m * (*(curve_data_ptr.second + j));
			}
		}
	};

	DLOG_TRC << "FastCurvesDeformer::__deform__ " << (multi_threaded ? "multi_threaded" : "single thread");

	if(multi_threaded) {
		mPool.detach_blocks(0u, curveBinds.size(), func);
		mPool.wait();
	} else {
		func(0u, curveBinds.size());
	}
	return true;
}

bool FastCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(!BaseMeshCurvesDeformer::writeJsonDataToPrimImpl()) {
		return false;
	}

	if(!mCurvesGeoPrimHandle.writeDataToBson(mpFastCurvesDeformerData.get())) {
		DLOG_ERR << "Error writing " << mpFastCurvesDeformerData->typeName() << " deformer data to json !";	
		return false;
	}
	return true;
}

void FastCurvesDeformer::drawDebugGeometry(pxr::UsdTimeCode time_code) {
	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	if(mpDebugGeo) {
		mpDebugGeo->clear();

		const pxr::VtArray<pxr::GfVec3f>& positions = pPhantomTrimesh->getLivePositions();

		for(const auto face: pPhantomTrimesh->getFaces()) {
			DebugGeo::Line lA(positions[face.indices[0]], positions[face.indices[0]] + mLiveVertexNormals[face.indices[0]]);
			lA.setColor({1.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
			lA.setWidth(0.05);
			DebugGeo::Line lB(positions[face.indices[1]], positions[face.indices[1]] + mLiveVertexNormals[face.indices[1]]);
			lB.setColor({1.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
			lB.setWidth(0.05);
			DebugGeo::Line lC(positions[face.indices[2]], positions[face.indices[2]] + mLiveVertexNormals[face.indices[2]]);
			lC.setColor({1.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
			lC.setWidth(0.05);

			mpDebugGeo->addLine(lA);
			mpDebugGeo->addLine(lB);
			mpDebugGeo->addLine(lC);
		}

		const auto& curveBinds = mpFastCurvesDeformerData->mCurveBinds;
		for(size_t i = 0; i < curveBinds.size(); ++i) {
			
			const pxr::GfVec3f& N = mPerBindLiveNormals[i];
			const pxr::GfVec3f& T = mPerBindLiveTBs[i].first;
			const pxr::GfVec3f& B = mPerBindLiveTBs[i].second;

			const auto& bind = curveBinds[i];
			auto curve_bind_pos = pPhantomTrimesh->getInterpolatedLivePosition(bind.face_id, bind.u, bind.v);

			DebugGeo::Line lN(curve_bind_pos, curve_bind_pos + N*0.1f);
			lN.setColor({0.0, 1.0, 0.0});
			DebugGeo::Line lT(curve_bind_pos, curve_bind_pos + T*0.1f);
			lT.setColor({1.0, 0.0, 0.0});
			DebugGeo::Line lB(curve_bind_pos, curve_bind_pos + B*0.1f);
			lB.setColor({0.0, 0.0, 1.0});
			
			mpDebugGeo->addLine(lN);
			mpDebugGeo->addLine(lT);
			mpDebugGeo->addLine(lB);
		}

		mpDebugGeo->build("/debugNormals", mCurvesGeoPrimHandle.getStage());
	}
}

bool FastCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	PROFILE("FastCurvesDeformer::buildDeformerDataImpl");

	if(!BaseMeshCurvesDeformer::buildDeformerDataImpl(rest_time_code, multi_threaded)) {
		return false;
	}

	if(!mpFastCurvesDeformerData) {
		mpFastCurvesDeformerData = std::make_unique<FastCurvesDeformerData>();
	}
	mpFastCurvesDeformerData->clear();

	if(!getReadJsonDataState() || !mCurvesGeoPrimHandle.getDataFromBson(mpFastCurvesDeformerData.get())) {
		// Build deformer data in place if no json data present or not needed
		
		assert(mpAdjacencyData);
		const auto* pAdjacency = mpAdjacencyData->getAdjacency();
		assert(pAdjacency);
		if(!pAdjacency->isValid()) {
			DLOG_ERR << "No valid mesh adjacency data!";
			return false;
		}

		assert(pAdjacency->getMaxFaceVertexCount() > 0);

		assert(mpPhantomTrimeshData);
		const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
		assert(pPhantomTrimesh);
		if(!pPhantomTrimesh->isValid()) {
			DLOG_ERR << "No valid phantom trimesh data!";
			return false;
		}

		if(!buildCurvesBindingData(rest_time_code, multi_threaded)) {
			return false;
		}

		const size_t binds_count = mpFastCurvesDeformerData->getCurveBinds().size();

		mpFastCurvesDeformerData->mRestVertexNormals.resize(binds_count);
		mpFastCurvesDeformerData->mPerBindRestNormals.resize(binds_count);
		mpFastCurvesDeformerData->mPerBindRestTBs.resize(binds_count);

		// Additional per-bind rest normals if needed
		const bool build_live = false; // build using rest data
		std::vector<pxr::GfVec3f>& vertex_normals = build_live ? mLiveVertexNormals : mpFastCurvesDeformerData->mRestVertexNormals;
		buildVertexNormals(pAdjacency, pPhantomTrimesh, vertex_normals, build_live, (multi_threaded ? &mPool : nullptr));
		calcPerBindNormals(pAdjacency, pPhantomTrimesh, vertex_normals, build_live, (multi_threaded ? &mPool : nullptr));
		calcPerBindTangentsAndBiNormals(pPhantomTrimesh, build_live, (multi_threaded ? &mPool : nullptr));
		
		mpFastCurvesDeformerData->setPopulated(true);
	}

	mLiveVertexNormals.resize(mpFastCurvesDeformerData->getRestVertexNormals().size());
	mPerBindLiveNormals.resize(mpFastCurvesDeformerData->getPerBindRestNormals().size());
	mPerBindLiveTBs.resize(mpFastCurvesDeformerData->getPerBindRestTBs().size());

	// transform curves to NTB spaces
	transformCurvesToNTB();

	return true; 
}

void FastCurvesDeformer::calcPerBindNormals(const UsdGeomMeshFaceAdjacency* pAdjacency, const PhantomTrimesh* pPhantomTrimesh, const std::vector<pxr::GfVec3f>& vertex_normals, bool build_live, BS::thread_pool<BS::tp::none>* pThreadPool) {
	assert(pAdjacency);
	assert(pPhantomTrimesh);

	// Build per bind normals
	auto& perBindNormals = build_live ? mPerBindLiveNormals : mpFastCurvesDeformerData->mPerBindRestNormals;
	const auto& curveBinds = mpFastCurvesDeformerData->getCurveBinds();
	assert(perBindNormals.size() == curveBinds.size());

	auto func = [&](const std::size_t i) {
        const auto& bind = curveBinds[i];
		
		if(bind.face_id != CurveBindData::kInvalidFaceID) {
			const auto& face = pPhantomTrimesh->getFace(bind.face_id);

			float u = bind.u;
			float v = bind.v;
			float w = 1.f - u - v;
			barycentrics_clamp_to_triangle(u, v, w);
			perBindNormals[i] = pxr::GfGetNormalized(u * vertex_normals[face.indices[1]] + v * vertex_normals[face.indices[2]] + w * vertex_normals[face.indices[0]]);
    	}
    };

	if(pThreadPool) {
        BS::multi_future<void> loop = pThreadPool->submit_loop(0u, curveBinds.size(), func);
        loop.wait();
    } else {
        for(size_t i = 0; i < curveBinds.size(); ++i) {
            func(i);
        }
    }
}

void FastCurvesDeformer::calcPerBindTangentsAndBiNormals(const PhantomTrimesh* pPhantomTrimesh, bool build_live, BS::thread_pool<BS::tp::none>* pThreadPool) {
	static constexpr float kF = 1.f / 3.f;

	assert(pPhantomTrimesh);
	auto& perBindNormals = build_live ? mPerBindLiveNormals : mpFastCurvesDeformerData->mPerBindRestNormals;

	const auto& curveBinds = mpFastCurvesDeformerData->getCurveBinds();
	assert(perBindNormals.size() == curveBinds.size());

	auto& mPerBindTBs = build_live ? mPerBindLiveTBs : mpFastCurvesDeformerData->mPerBindRestTBs;
	assert(mPerBindTBs.size() == curveBinds.size());

	const auto& pt_positions = build_live ? pPhantomTrimesh->getLivePositions() : pPhantomTrimesh->getRestPositions();
	const std::vector<PhantomTrimesh::TriFace>& faces = pPhantomTrimesh->getFaces(); 

	pxr::VtArray<pxr::GfVec3f> face_center_points(faces.size());

	for(size_t i = 0; i < faces.size(); ++i) {
		const auto& face = faces[i];
		face_center_points[i] = (pt_positions[face.indices[0]] + pt_positions[face.indices[1]] + pt_positions[face.indices[2]]) * kF;
	}

	auto func = [&](const std::size_t i) {
		const auto& bind = curveBinds[i];
		
		if(bind.face_id != CurveBindData::kInvalidFaceID) {
			const auto& face = faces[bind.face_id];
			const pxr::GfVec3f root_proj_pos = bind.u * pt_positions[face.indices[0]] + bind.v * pt_positions[face.indices[2]] + (1.f - bind.u - bind.v) * pt_positions[face.indices[1]];			
			const pxr::GfVec3f tmp_binormal = face_center_points[bind.face_id] - root_proj_pos;

			mPerBindTBs[i].first = pxr::GfGetNormalized(pxr::GfCross(perBindNormals[i], tmp_binormal)); // tangent
			mPerBindTBs[i].second = pxr::GfGetNormalized(pxr::GfCross(perBindNormals[i], mPerBindTBs[i].first)); //binormal
    	}
    };

	if(pThreadPool) {
        BS::multi_future<void> loop = pThreadPool->submit_loop(0u, curveBinds.size(), func);
        loop.wait();
    } else {
        for(size_t i = 0; i < curveBinds.size(); ++i) {
            func(i);
        }
    }
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
		}
	}
}

bool FastCurvesDeformer::bindCurveToTriface(uint32_t curve_index, uint32_t face_id, CurveBindData& bind, bool ignore_face_boundaries) {
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

	// if failed but allowed to ignore face boundaries we bind curve by finding closest to face plane
	if(ignore_face_boundaries) {
		assert(false && "ignore_face_boundaries unimplemented!");
		return true;
	}


	return false;
}

bool FastCurvesDeformer::buildCurvesBindingData(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!mpCurvesContainer) return false;

	pxr::UsdGeomPrimvarsAPI curvesPrimvarsApi = mCurvesGeoPrimHandle.getPrimvarsAPI();

	const size_t total_curves_count = mpCurvesContainer->getCurvesCount();

	auto& curveBinds = mpFastCurvesDeformerData->mCurveBinds;

	curveBinds.resize(total_curves_count);
	std::atomic<uint32_t> skin_bound_curves_count = 0;
	std::atomic<uint32_t> kdtree_bound_curves_count = 0;
	std::atomic<uint32_t> bforce_bound_curves_count = 0;


	// Clear all possible previous binds
	for(auto& bind: curveBinds) {
		bind.face_id = CurveBindData::kInvalidFaceID;
	}

	assert(mpAdjacencyData);
	const auto* pAdjacency = mpAdjacencyData->getAdjacency();
	assert(pAdjacency);

	assert(mpPhantomTrimeshData);
	auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	auto bindCurveToPrim = [&] (uint32_t curve_index, CurveBindData& bind, uint32_t prim_id, std::vector<float>& _tmp_sq_distances, bool ignore_face_boundaries) {
		bool isBound = false;
		const uint32_t prim_vertex_count = pAdjacency->getFaceVertexCount(prim_id);
		const uint32_t prim_vertex_offset = pAdjacency->getFaceVertexOffset(prim_id);

		if ( prim_vertex_count > 3u){
			if(_tmp_sq_distances.size() < prim_vertex_count) _tmp_sq_distances.resize(prim_vertex_count);
			
			for(size_t j = 0; j < prim_vertex_count; ++j) {
				_tmp_sq_distances[j] = distanceSquared(mpCurvesContainer->getCurveRootPoint(curve_index), pPhantomTrimesh->getRestPositions()[pAdjacency->getFaceVertex(prim_vertex_offset + j)]);
			}

			std::vector<float>::iterator it = std::min_element(_tmp_sq_distances.begin(), _tmp_sq_distances.begin() + prim_vertex_count);
			uint32_t local_index = std::distance(std::begin(_tmp_sq_distances), it);

			for(uint32_t i = 1; i < (prim_vertex_count - 1); ++i) {
				const uint32_t face_id = pPhantomTrimesh->getOrCreateFaceID(
					pAdjacency->getFaceVertex(prim_id, local_index), 
					pAdjacency->getFaceVertex(prim_id, (local_index + i) % prim_vertex_count),
					pAdjacency->getFaceVertex(prim_id, (local_index + i + 1) % prim_vertex_count)
				);

				if(bindCurveToTriface(curve_index, face_id, bind, ignore_face_boundaries)) {
					isBound = true;
					break;
				} 
			}
		} else {
			const uint32_t face_id = pPhantomTrimesh->getOrCreateFaceID(
				pAdjacency->getFaceVertex(prim_id, 0), 
				pAdjacency->getFaceVertex(prim_id, 1),
				pAdjacency->getFaceVertex(prim_id, 2)
			);
			isBound = bindCurveToTriface(curve_index, face_id, bind, ignore_face_boundaries);
		}

		return isBound;
	};

	pxr::VtArray<int> skin_prim_indices;
	const bool has_skin_prim_attr = mCurvesGeoPrimHandle.fetchAttributeValues(mSkinPrimAttrName, skin_prim_indices, rest_time_code) && (skin_prim_indices.size() == total_curves_count);

	std::mutex kdtree_mutex;  // protects kdree initialisation
	std::unique_ptr<neighbour_search::KDTree<float, 3>> pKDTree;

	auto func = [&](const std::size_t start, const std::size_t end) {
		std::vector<float> tmp_squared_distances(pAdjacency->getMaxFaceVertexCount());
		std::vector<std::pair<float, uint32_t>> tmp_indexed_squared_distances(pAdjacency->getMaxFaceVertexCount());

		for(uint32_t curve_index = static_cast<uint32_t>(start); curve_index < static_cast<uint32_t>(end); ++curve_index) {

			auto& bind = curveBinds[curve_index];

			// Strategy: 1. First we try to bind curves using skin prim ids
			if(has_skin_prim_attr) {
				if(skin_prim_indices[curve_index] < 0) continue; // pixar uses negative indices as invalid

				const uint32_t prim_id = static_cast<uint32_t>(skin_prim_indices[curve_index]);
				if(!bindCurveToPrim(curve_index, bind, prim_id, tmp_squared_distances, false /* respect face boundaries */)) {
					const auto& pt = mpCurvesContainer->getCurveRootPoint(curve_index);				
					const uint32_t prim_vertex_count = pAdjacency->getFaceVertexCount(prim_id);

					if(prim_vertex_count == 3) {
						bind.face_id = pPhantomTrimesh->getOrCreateFaceID(
							pAdjacency->getFaceVertex(prim_id, 0), 
							pAdjacency->getFaceVertex(prim_id, 1),
							pAdjacency->getFaceVertex(prim_id, 2)
						);
					} else {

						for(size_t j = 0; j < prim_vertex_count; ++j) {
							const auto vtx = pAdjacency->getFaceVertex(prim_id, j);
							tmp_indexed_squared_distances[j] = { distanceSquared(pt, pPhantomTrimesh->getRestPositions()[vtx]), vtx };
						}
						
						std::sort(tmp_indexed_squared_distances.begin(), tmp_indexed_squared_distances.begin() + prim_vertex_count);
						
						bind.face_id = pPhantomTrimesh->getOrCreateFaceID(
							tmp_indexed_squared_distances[0].second,
							tmp_indexed_squared_distances[1].second,
							tmp_indexed_squared_distances[2].second
						);
					}

					assert(bind.face_id != PhantomTrimesh::kInvalidTriFaceID);
					pPhantomTrimesh->projectPoint(pt, bind.face_id, bind.u, bind.v);
				}

				skin_bound_curves_count++;
			}

			if(bind.face_id != CurveBindData::kInvalidFaceID) continue; // bound already
			// Strategy: 2. Bind ramaining curves using nearest point

			// Build guide kdtree if needed
			{
        		const std::lock_guard<std::mutex> lock(kdtree_mutex);
        		if(!pKDTree) {
        			pKDTree = std::make_unique<neighbour_search::KDTree<float, 3>>(pPhantomTrimesh->getRestPositions(), false /* no threads */);
        		}
        	}

			std::vector<neighbour_search::KDTree<float, 3>::ReturnType> nearest_points;

			const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_index);
			const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = pKDTree->findNearestNeighbour(curve_root_pt);

			const auto nearest_vtx = nearest_pt.first;
			const uint32_t neighbors_count = pAdjacency->getNeighborsCount(nearest_vtx);
			if(neighbors_count > 0) {
				const uint32_t neighbors_offset = pAdjacency->getNeighborsOffset(nearest_vtx);
				for(uint32_t prim_offset = neighbors_offset; prim_offset < (neighbors_offset + neighbors_count); ++prim_offset){
					const auto prim_id = pAdjacency->getNeighborPrim(prim_offset);
					if(bindCurveToPrim(curve_index, bind, prim_id, tmp_squared_distances, false /* respect face boundaries */)) {
						kdtree_bound_curves_count++;
						break;
					}
				}
			}

			if(bind.face_id != CurveBindData::kInvalidFaceID) continue; // bound already
			// Strategy: 3. Brute force binding

			for(const auto& prim_id: pAdjacency->getPrimData()) {
				if(bindCurveToPrim(curve_index, bind, prim_id, tmp_squared_distances, false /* respect face boundaries */)) {
					bforce_bound_curves_count++;
					break;
				}
			}
		}
	};

	DLOG_INF << "Binding curves to mesh.";

	if(multi_threaded) {
		mPool.detach_blocks(0u, total_curves_count, func);
		mPool.wait();
	} else {
		func(0u, total_curves_count);
	}

	DLOG_DBG << "Total curves count to bind: " << size_t(total_curves_count);
	DLOG_DBG << "Skin bound curves count: " << size_t(skin_bound_curves_count.load());
	DLOG_DBG << "KDtree bound curves count: " << size_t(kdtree_bound_curves_count.load());
	DLOG_DBG << "Brute force bound curves count: " << size_t(bforce_bound_curves_count.load());

	return true;
}

FastCurvesDeformer::~FastCurvesDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston