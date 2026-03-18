#include "simple_profiler.h"
#include "guide_curves_deformer.h"
#include "deformer_data_cache.h"

#include "common.h"
#include "logging.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <pxr/base/gf/matrix4f.h>

#include <random>
#include <algorithm>
#include <mutex>
#include <atomic>


namespace Piston {

GuideCurvesDeformer::GuideCurvesDeformer(const std::string& name): BaseCurvesDeformer(BaseCurvesDeformer::Type::GUIDES, name) {
	LOG_TRC << "GuideCurvesDeformer::GuideCurvesDeformer(" << getName() << ")";
	mpGuideCurvesDeformerData = std::make_unique<GuideCurvesDeformerData>();
	mpGuideCurvesContainer = GuideCurvesContainer::create();
}

GuideCurvesDeformer::SharedPtr GuideCurvesDeformer::create(const std::string& name) {
	return SharedPtr(new GuideCurvesDeformer(name));
}

bool GuideCurvesDeformer::validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	return isBasisCurvesGeoPrim(geoPrim);
}

const std::string& GuideCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "GuideCurvesDeformer";
	return kFastDeformerString;
}

void GuideCurvesDeformer::setFastPointBind(bool fast) {
	if(mFastPointBind == fast) return;
	mFastPointBind = fast;
	makeDirty();
}

void GuideCurvesDeformer::setGuideIDPrimAttrName(const std::string& name) {
	if(mGuideIDPrimAttrName == name) return;
	mGuideIDPrimAttrName = name;
	makeDirty();

	LOG_DBG << "Guide ID attribute name is set to: " << mGuideIDPrimAttrName;
}

void GuideCurvesDeformer::setGuidesSkinGeoPrimAttrName(const std::string& name) {
	if(mGuidesSkinPrimAttrName == name) return;
	mGuidesSkinPrimAttrName = name;
	makeDirty();

	LOG_DBG << "Guide skin prim attribute name is set to: " << mGuidesSkinPrimAttrName;
}

void GuideCurvesDeformer::setGuidesSkinGeoPrimRestAttrName(const std::string& name) {
	if(mGuidesSkinPrimRestAttrName == name) return;
	mGuidesSkinPrimRestAttrName = name;
	makeDirty();

	LOG_DBG << "Guide skin prim rest attribute name is set to: " << mGuidesSkinPrimRestAttrName;
}

void GuideCurvesDeformer::setGuidesSkinGeoPrim(const pxr::UsdPrim& geoPrim) {
	if(mGuidesSkinGeoPrimHandle == geoPrim) return;
	
	if(!isMeshGeoPrim(geoPrim)) {
		mGuidesSkinGeoPrimHandle.clear();
		LOG_ERR << "Invalid guides skin geometry prim " <<  geoPrim.GetPath().GetText() << " type!";
		return;
	}

	mGuidesSkinGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	LOG_DBG << "Guides skin geometry prim is set to: " << mGuidesSkinGeoPrimHandle.getPath().GetText();
}

bool GuideCurvesDeformer::deformImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, false, time_code);
}

bool GuideCurvesDeformer::deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, true, time_code);
}

bool GuideCurvesDeformer::__deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code) {
	if(!mpGuideCurvesContainer->update(mDeformerGeoPrimHandle, time_code)) {
		LOG_ERR << "Error updating guide curves from prim" << mDeformerGeoPrimHandle.getPath().GetText() << " !";
		return false;
	}

	const auto bind_mode = getBindMode();
	LOG_TRC << "GuideCurvesDeformer::__deform__() mode " << to_string(bind_mode) << " " << (multi_threaded ? "multi_threaded" : "single thread");
	
	bool result = false;

	switch(bind_mode) {
		case BindMode::SPACE:
			result = deformImpl_SpaceMode(multi_threaded, points, time_code);
			break;
		case BindMode::ANGLE:
			result = deformImpl_AngleMode(multi_threaded, points, time_code);
			break;
		case BindMode::NTB:
			result = deformImpl_NTBMode(multi_threaded, points, time_code);
			break;
		default:
			assert(false && "Unimplemented GuideCurvesDeformer::BindMode");
			LOG_ERR << "Unimplemented bind mode " << to_string(bind_mode) << " !!!";
			break;
	}

	if(result && getBindRootsToSkinSurface() && !mpGuideCurvesDeformerData->getPointSurfaceBinds().empty()) {
		result = moveSkinBoundPoints(multi_threaded, points, time_code);
	}

	return result;
}

bool GuideCurvesDeformer::moveSkinBoundPoints(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
	if(!hasSkinPrimitiveData()) {
		return true;
	}

	assert(mpSkinPhantomTrimeshData);
	PhantomTrimesh* pSkinPhantomTrimesh = mpSkinPhantomTrimeshData->getTrimesh();
	assert(pSkinPhantomTrimesh);
	pSkinPhantomTrimesh->update(mGuidesSkinGeoPrimHandle ,time_code);

	mTempSkinFaceLiveNormals.resize(pSkinPhantomTrimesh->getFaceCount());
	LOG_WRN << "TODO: precalculate skin live face normals first !!!";

	const auto& pointBinds = mpGuideCurvesDeformerData->getPointSurfaceBinds();

	auto func = [&](const std::size_t start, const std::size_t end) {
		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			assert(bind.point_id < points.size());

			const pxr::GfVec3f face_normal = pSkinPhantomTrimesh->getFaceLiveNormal(bind.face_id);
			auto pos = pSkinPhantomTrimesh->getInterpolatedLivePosition(bind.face_id, bind.u, bind.v) + face_normal * bind.dist;
			points[bind.point_id] = pos * bind.weight + points[bind.point_id] * (1.f - bind.weight);
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, pointBinds.size(), func);
		mPool.wait();
	} else {
		func(0u, pointBinds.size());
	}

	return true;
}

bool GuideCurvesDeformer::deformImpl_AngleMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
	const auto& pointBinds = mpGuideCurvesDeformerData->getPointBinds();
	const size_t guide_curves_count = mpGuideCurvesContainer->getCurvesCount();
	const auto& guides_rest_points = mpGuideCurvesContainer->getRestCurvePoints();
	const auto& guides_live_points = mpGuideCurvesContainer->getLiveCurvePoints();

	auto func = [&](const std::size_t start, const std::size_t end) {

		uint32_t guide_id;
		uint8_t segment_id;
		pxr::GfVec3f vec;

		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			if(bind.encoded_id == PointBindData::kInvalid) continue;

			bind.decodeID_modeANGLE(guide_id, segment_id);
			assert(guide_id < guide_curves_count);
			assert((size_t)segment_id < (mpGuideCurvesContainer->getCurveVertexCount(guide_id) - 1));
			bind.getData(vec);
			const size_t guide_segment_start_vtx = mpGuideCurvesContainer->getCurveVertexOffset(guide_id) + segment_id;

			// TODO: precalculate rest vectors. Maybe use CurvesContainter class instead as it's already in vectors form.... 
			const pxr::GfVec3f rest_segment_vector_n = pxr::GfGetNormalized(guides_rest_points[guide_segment_start_vtx + 1] - guides_rest_points[guide_segment_start_vtx]);
			const pxr::GfVec3f live_segment_vector_n = pxr::GfGetNormalized(guides_live_points[guide_segment_start_vtx + 1] - guides_live_points[guide_segment_start_vtx]);

			const pxr::GfMatrix3f m = rotateAlign(rest_segment_vector_n, live_segment_vector_n);

			points[i] = guides_live_points[guide_segment_start_vtx] + (m * vec);
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, pointBinds.size(), func);
		mPool.wait();
	} else {
		func(0u, pointBinds.size());
	}

	return true;
}

bool GuideCurvesDeformer::hasSkinPrimitiveData() const {
	if(!mGuidesSkinGeoPrimHandle.isValid()) return false;
	return mpSkinPhantomTrimeshData && mpSkinPhantomTrimeshData->isValid();
}

bool GuideCurvesDeformer::deformImpl_NTBMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
	assert(mpGuideCurvesDeformerData);
	assert(mpGuideCurvesContainer);

	assert(hasSkinPrimitiveData()); // for now we only work with skin geometry
	PhantomTrimesh* pSkinPhantomTrimesh = hasSkinPrimitiveData() ? mpSkinPhantomTrimeshData->getTrimesh() : nullptr;
	assert(pSkinPhantomTrimesh);
	pSkinPhantomTrimesh->update(mGuidesSkinGeoPrimHandle ,time_code);

	const auto total_guides_count = mpGuideCurvesContainer->getCurvesCount();
	const auto& guides_live_points = mpGuideCurvesContainer->getLiveCurvePoints();

	const auto& pointBinds = mpGuideCurvesDeformerData->getPointBinds();
	std::vector<NTBFrame> live_guide_frames(mpGuideCurvesContainer->getLiveCurvePoints().size());

	static const bool build_live = true;

	if(!buildNTBFrames(live_guide_frames, multi_threaded, build_live)) {
		return false;
	}
	
	auto func = [&](const std::size_t start, const std::size_t end) {
		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			if(bind.encoded_id == PointBindData::kInvalid) continue;

			uint32_t frame_id;
			bind.decodeID_modeNTB(frame_id);
			assert(frame_id < guides_live_points.size());
			assert(frame_id < live_guide_frames.size());
			const std::array<float, 3>& ntbCoord = bind.getData();

			points[i] = guides_live_points[frame_id] + live_guide_frames[frame_id] * ntbCoord;
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, pointBinds.size(), func);
		mPool.wait();
	} else {
		func(0u, pointBinds.size());
	}

	return true;
}

bool GuideCurvesDeformer::deformImpl_SpaceMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
	assert(mpGuidesPhantomTrimeshData);
	PhantomTrimesh* pPhantomTrimesh = mpGuidesPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	if(!pPhantomTrimesh->update(mDeformerGeoPrimHandle, time_code)) {
		return false;
	}

	const auto& pointBinds = mpGuideCurvesDeformerData->getPointBinds();
	auto func = [&](const std::size_t start, const std::size_t end) {

		float u, v, w, x;

		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			if(bind.encoded_id == PointBindData::kInvalid) continue;

			if(bind.encoded_id.mode_space.is_tetra) {
				// bound to tetra
				const auto& tetra = pPhantomTrimesh->getTetrahedron(bind.encoded_id.mode_space.element_id);

				if(bind.encoded_id.mode_space.is_24bit) {
					bind.getData(u, v, w, x);
				} else {
					bind.getData(u, v, w);
					x = 1.f - (u + v + w);
				}
				points[i] = pPhantomTrimesh->getPointPositionFromBarycentricTetrahedronLiveCoords(tetra, u, v, w, x);
			} else {
				// bound to triface
				LOG_WRN << "TODO: precalculate skin live face normals first for SPACE mode !!!";
				const pxr::GfVec3f face_normal = pPhantomTrimesh->getFaceLiveNormal(bind.encoded_id.mode_space.element_id);
				bind.getData(u, v, w);
				points[i] = pPhantomTrimesh->getInterpolatedLivePosition(bind.encoded_id.mode_space.element_id, u, v) + (face_normal * w);
			}
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, pointBinds.size(), func);
		mPool.wait();
	} else {
		func(0u, pointBinds.size());
	}

    return true;
}

bool GuideCurvesDeformer::buildCurvesRootsBindDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	assert(mpCurvesContainer);
	assert(mpGuideCurvesDeformerData);
	assert(mpSkinAdjacencyData && mpSkinAdjacencyData->isValid());
	assert(mpSkinPhantomTrimeshData && mpSkinPhantomTrimeshData->isValid());

	const UsdGeomMeshFaceAdjacency* pSkinAdjacency = mpSkinAdjacencyData->getAdjacency();
	PhantomTrimesh* pSkinPhantomTrimesh = mpSkinPhantomTrimeshData->getTrimesh();
	const PxrCurvesContainer* pCurvesContainer = mpCurvesContainer.get();

	const size_t curves_count = pCurvesContainer->getCurvesCount();

	std::vector<int> skin_prim_indices;

	if(!mCurvesGeoPrimHandle.fetchAttributeValues(getSkinPrimAttrName(), skin_prim_indices, rest_time_code)) {

		// if there is no curves skinprim attr exist we can still try to promote it from guides ...
		if(mGuideIndices.size() == curves_count) {
			std::vector<int> guides_skin_prim_indices;
			if(mDeformerGeoPrimHandle.fetchAttributeValues(getSkinPrimAttrName(), guides_skin_prim_indices, rest_time_code)) {
				skin_prim_indices.resize(curves_count);

				for(size_t i = 0; i < curves_count; ++i) {
					skin_prim_indices[i] = guides_skin_prim_indices[mGuideIndices[i]];
				}
			}
		} else {
			LOG_ERR << "Skin prim ID is needed to bind curves roots for now !!!";
			return false;
		}
	}

	const bool is_per_vertex_attr = pCurvesContainer->getTotalVertexCount() == skin_prim_indices.size();
	const bool is_per_curve_attr = pCurvesContainer->getCurvesCount() == skin_prim_indices.size();

	if(!is_per_vertex_attr && !is_per_curve_attr) {
		LOG_ERR << "Wrong skin prim ID attribute values count ! Should be per curve or per curve vertex !";
		return false;
	}

	std::mutex binds_mutex;
	auto& point_surface_binds = mpGuideCurvesDeformerData->pointSurfaceBinds();
	point_surface_binds.clear();
	point_surface_binds.reserve(curves_count);

	auto bindPointToSkinPrim = [&] (const pxr::GfVec3f& pt, PointSurfaceBindData& bind, uint32_t prim_id, std::vector<float>& _tmp_sq_distances, bool ignore_prim_boundaries = false) {
		bool is_bound = false;
		const uint32_t prim_vertex_count = pSkinAdjacency->getFaceVertexCount(prim_id);
		const uint32_t prim_vertex_offset = pSkinAdjacency->getFaceVertexOffset(prim_id);

		float face_id, u, v, dist;

		if( prim_vertex_count > 3u){
			if(_tmp_sq_distances.size() < prim_vertex_count) _tmp_sq_distances.resize(prim_vertex_count);
				
			const pxr::VtArray<pxr::GfVec3f>& rest_positions = pSkinPhantomTrimesh->getRestPositions();

			for(size_t j = 0; j < prim_vertex_count; ++j) {
				_tmp_sq_distances[j] = distanceSquared(pt, rest_positions[pSkinAdjacency->getFaceVertex(prim_vertex_offset + j)]);
			}

			std::vector<float>::iterator it = std::min_element(_tmp_sq_distances.begin(), _tmp_sq_distances.begin() + prim_vertex_count);
			uint32_t local_index = std::distance(std::begin(_tmp_sq_distances), it);

			static const float kFLT_MAX = std::numeric_limits<float>::max();

			float pt_tri_dist_sq_min = kFLT_MAX;
			float _face_id = PhantomTrimesh::kInvalidTriFaceID, _u, _v, _dist;
			for(uint32_t i = 1; i < (prim_vertex_count - 1); ++i) {
				_face_id = pSkinPhantomTrimesh->getOrCreateFaceID(
					pSkinAdjacency->getFaceVertex(prim_id, local_index), 
					pSkinAdjacency->getFaceVertex(prim_id, (local_index + i) % prim_vertex_count),
					pSkinAdjacency->getFaceVertex(prim_id, (local_index + i + 1) % prim_vertex_count)
				);

				is_bound = pSkinPhantomTrimesh->projectPoint(pt, _face_id, _u, _v, _dist);
				
				if(is_bound) {
					face_id = _face_id; u = _u; v = _v; dist = _dist;
					break;
				} else if(ignore_prim_boundaries) {
					// If outside we push point to triangle squared distance for later closest search
					const auto& face = pSkinPhantomTrimesh->getFace(_face_id);
					const float pt_tri_dist_sq = pointTriangleDistSquared(pt, rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]]);
					if(pt_tri_dist_sq < pt_tri_dist_sq_min) {
						face_id = _face_id;
						u = _u; v =_v; dist = _dist;
						pt_tri_dist_sq_min = pt_tri_dist_sq;
					}
				}
			}
		} else {
			face_id = pSkinPhantomTrimesh->getOrCreateFaceID(
				pSkinAdjacency->getFaceVertex(prim_id, 0), 
				pSkinAdjacency->getFaceVertex(prim_id, 1),
				pSkinAdjacency->getFaceVertex(prim_id, 2)
			);
			is_bound = pSkinPhantomTrimesh->projectPoint(pt, face_id, u, v, dist);
		}


		if(is_bound || ignore_prim_boundaries) {
			bind.face_id = face_id;
			bind.u = u; bind.v = v; bind.dist = dist;
		}
		
		return is_bound;
	};

	auto func = [&](const std::size_t start, const std::size_t end) {
		std::vector<float> tmp_squared_distances(pSkinAdjacency->getMaxFaceVertexCount());

		for(size_t curve_index = start; curve_index < end; ++curve_index) {
			
			const PxrCurvesContainer::CurveDataConstPtr curve_data_ptr = pCurvesContainer->getCurveDataPtr(curve_index);
			const int curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);
			if(curve_vertices_count < 2) continue;

			const uint32_t curve_vertex_offset = pCurvesContainer->getCurveVertexOffset(curve_index);
        	const pxr::GfVec3f& curve_root_pt = pCurvesContainer->getCurveRootPoint(curve_index);

			for(uint32_t i = 0; i < curve_vertices_count; ++i) {
        		const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i);

				PointSurfaceBindData bind;
				bind.point_id = curve_vertex_offset + i;
				bind.weight = 1.0f;

				const uint32_t prim_id = is_per_vertex_attr ? skin_prim_indices[bind.point_id] : skin_prim_indices[curve_index];
				assert(prim_id >= 0 && prim_id < pSkinAdjacency->getFaceCount());

				bindPointToSkinPrim(curr_pt, bind, prim_id, tmp_squared_distances, true /* ignore prim boundaries */); 

				if(bind.face_id != PhantomTrimesh::kInvalidTriFaceID) {
					const std::lock_guard<std::mutex> lock(binds_mutex);
					point_surface_binds.push_back(bind);
				}
			
				break; // We process only root points for now
			}
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, curves_count, func);
		mPool.wait();
	} else {
		func(0u, curves_count);
	}

	LOG_DBG << "Deformer " << getName() << " " << point_surface_binds.size() << " points are bound to skin surface.";

	return true;
}

bool GuideCurvesDeformer::guideIndicesNeeded() const {
	if(getBindRootsToSkinSurface()) return true;
	if(getBindMode() == BindMode::NTB) return true;
	if(getBindMode() == BindMode::ANGLE) return true;

	return false;
}

bool GuideCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	assert(mpGuideCurvesDeformerData);
	assert(mpGuideCurvesContainer);

	if(!mpGuideCurvesContainer->init(mDeformerGeoPrimHandle, rest_time_code)) {
		LOG_ERR << "Error initializing guide curves container !";
		return false;
	}

	const auto total_guides_count = mpGuideCurvesContainer->getCurvesCount();
	const auto total_curves_count = mpCurvesContainer->getCurvesCount();

	if(guideIndicesNeeded()) {
		if(mGuideIDPrimAttrName.empty()) {
			LOG_ERR << "No guide id (clump_id) attribute name set but needed !";
			return false;
		}

		if(!mCurvesGeoPrimHandle.fetchAttributeValues(mGuideIDPrimAttrName, mGuideIndices, rest_time_code)) {
			LOG_ERR << "Error getting curves " << mCurvesGeoPrimHandle << " \"" << mGuideIDPrimAttrName << "\" guide indices !";
			return false;
		}
		assert(mGuideIndices.size() == total_curves_count);

		// check guide indices are not out of range
		int max_guide_index = 0;
		for(const int idx: mGuideIndices) {
			max_guide_index = std::max(max_guide_index, idx);
		}

		if(max_guide_index >= total_guides_count) {
			LOG_ERR << "Curves " << mCurvesGeoPrimHandle << " guide indices are out of range !"; 
			return false;
		}
	}

	if(getBindRootsToSkinSurface()) {
		if(!buildSkinPrimData(multi_threaded)) {
			LOG_ERR << "Error building skin geometry data for " << mGuidesSkinGeoPrimHandle << "!";
			return false;
		}

		if(!buildCurvesRootsBindDeformerData(rest_time_code, multi_threaded)) {
			LOG_ERR << "Error building curves roots bind data for " << mCurvesGeoPrimHandle << "!";
			return false;
		}
	}

	switch(getBindMode()) {
		case BindMode::SPACE:
			return buildDeformerDataSpaceMode(rest_time_code, multi_threaded);
		case BindMode::NTB:
			return buildDeformerDataNTBMode(rest_time_code, multi_threaded);
		default:
			return buildDeformerDataAngleMode(rest_time_code, multi_threaded);
	}
}

bool GuideCurvesDeformer::buildDeformerDataSpaceMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!mpGuidesPhantomTrimeshData) {
		mpGuidesPhantomTrimeshData = std::make_unique<SerializablePhantomTrimesh>();
	}

	if(!getReadJsonDataState() || !mDeformerGeoPrimHandle.getDataFromBson(mpGuidesPhantomTrimeshData.get())) {
		// Build in place if no json data present or not needed
		if(!mpGuidesPhantomTrimeshData->buildInPlace(mDeformerGeoPrimHandle, getDeformerRestAttrName())) {
			LOG_ERR << "Error building phantom mesh data!";
			return false;
		}
	}

	PhantomTrimesh* pPhantomTrimesh = mpGuidesPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	if(!pPhantomTrimesh->hasTetrahedrons()) {
		if(!pPhantomTrimesh->buildTetrahedrons()) {
			LOG_ERR << "Error building tetrahedrons!";
			return false;
		}
	}

	const std::vector<PhantomTrimesh::Tetrahedron>& tetrahedrons = pPhantomTrimesh->getTetrahedrons();

	// Build kdtree
	auto buildTetrahedronsRestCentroidsKDTree = [&](bool multi_threaded) {
		pxr::VtArray<pxr::GfVec3f> centroids(tetrahedrons.size());
		for(uint32_t t_id = 0; t_id < tetrahedrons.size(); ++t_id) {
			centroids[t_id] = pPhantomTrimesh->getTetrahedronRestCentroid(t_id);
		}

		return neighbour_search::KDTree<float, 3>(centroids, multi_threaded);
	};

	auto buildRestPositionsKDTree = [&](bool multi_threaded) {
		return neighbour_search::KDTree<float, 3>(pPhantomTrimesh->getRestPositions(), multi_threaded);
	};

	neighbour_search::KDTree<float, 3> centroids_kdtree = buildTetrahedronsRestCentroidsKDTree(multi_threaded);
	neighbour_search::KDTree<float, 3> deformer_restpoints_kdtree = buildRestPositionsKDTree(multi_threaded);

	const size_t curves_count = mpCurvesContainer->getCurvesCount();

	assert(mpGuideCurvesDeformerData);
	auto& pointBinds = mpGuideCurvesDeformerData->mPointBinds;
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	std::atomic<size_t> total_points = 0;
	std::atomic<size_t> bound_points = 0;
	std::atomic<size_t> unboud_points = 0;
	
	auto bind_func = [&](const std::size_t start, const std::size_t end) {
    	if(multi_threaded) {
    		const std::optional<std::size_t> thread_index = BS::this_thread::get_index();
    		dbg_printf("Binding curves from %zu to %zu by thread id #%zu\n", start, end, *thread_index);
    	}

    	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> sorted_tetra_centroids; // sorted tetrahedron centroids for brute force search
    	sorted_tetra_centroids.reserve(tetrahedrons.size());

    	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> closest_deformer_points(3);

    	const auto& faces = pPhantomTrimesh->getFaces();

		for(size_t curve_idx = start; curve_idx < end; ++curve_idx) {
    		const PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_idx);

    		const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);

        	const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_idx);
        	const uint32_t curve_vertex_offset = mpCurvesContainer->getCurveVertexOffset(curve_idx);

        	float u, v, w, x;

        	for(uint32_t i = 0; i < curve_vertices_count; ++i) {
        		total_points++;

        		bool bound = false;
        		const size_t curr_point_index = curve_vertex_offset + i;
        		auto& bind = pointBinds[curr_point_index];
        		bind.encoded_id = PointBindData::kInvalid;

        		const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 

        		//findKNearestNeighboursWithinRadiusSquared(const PointType &target, std::size_t k, CoordinateType radius_squared, std::vector<ReturnType> &result)

        		// First try to find tetrahedron is connected to cleset point in point cloud
        		const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = deformer_restpoints_kdtree.findNearestNeighbour(curr_pt);
        		const size_t point_connected_tetras_count = pPhantomTrimesh->getPointConnectedTetrahedronsCount(nearest_pt.first);

        		for(size_t lti = 0; lti < point_connected_tetras_count; ++lti) {
        			uint32_t tetra_index = pPhantomTrimesh->getPointConnectedTetrahedronIndex(nearest_pt.first, lti);

        			pPhantomTrimesh->barycentricTetrahedronRestCoords(tetra_index, curr_pt, u, v, w, x);

        			if(u < 0.0f || v < 0.0f || w < 0.0f || x < 0.0f ||
        			   u > 1.0f || v > 1.0f || w > 1.0f || x > 1.0f ) {
        				// neighbour test failed;
        				continue;
        			}

        			// bound by nerest point cloud vertex;
        			bind.encodeID_modeSPACE(tetra_index, true /* is tetra id */, false /* data is 3x32bit floats */);
        			bind.setData(u, v, w);
        			bound_points++;
        			break;
        		}

        		neighbour_search::KDTree<float, 3>::ReturnType closest_tetra_centroid;
        		closest_tetra_centroid.first = neighbour_search::KDTree<float, 3>::kInvalidIndex;

        		// If point os not boud by previous method we test it against closest tetraheron centroid
        		if(bind.encoded_id == PointBindData::kInvalid) {
        			closest_tetra_centroid = centroids_kdtree.findNearestNeighbour(curr_pt);
					
					pPhantomTrimesh->barycentricTetrahedronRestCoords(closest_tetra_centroid.first, curr_pt, u, v, w, x);
        			if(!(u < 0.0f || v < 0.0f || w < 0.0f || x < 0.0f ||
        			   	 u > 1.0f || v > 1.0f || w > 1.0f || x > 1.0f )) {
        				// bound by nearest tetra centroid
        				bind.encodeID_modeSPACE(closest_tetra_centroid.first, true /* is tetra id */, false /* data is 3x32bit floats */);
        				bind.setData(u, v, w);
        				bound_points++;
        			}
        		}

        		// Brute force search for contaning centroid
        		if(bind.encoded_id == PointBindData::kInvalid) {
        			const bool sort = false;
        			centroids_kdtree.findAllNearestNeighboursWithinRadiusSquared(curr_pt, sorted_tetra_centroids, sort);
					
					float x;
        			for(const auto& centroid: sorted_tetra_centroids) {
						pPhantomTrimesh->barycentricTetrahedronRestCoords(centroid.first, curr_pt, u, v, w, x);
        				if(!(u < 0.0f || v < 0.0f || w < 0.0f || x < 0.0f ||
        			   	     u > 1.0f || v > 1.0f || w > 1.0f || x > 1.0f)) {
        				   	// bound by brute force search
        				   	bind.encodeID_modeSPACE(centroid.first, true /* is tetra id */, false /* data is 3x32bit floats */);
        				   	bind.setData(u, v, w);
        					bound_points++;
        					break;
        				}
        			}
        		}

        		// Now for any unbound point we bind it to closest triface or tetra (depends on user choice)

        		const bool bindMissingToTriface = true;
        		
        		if(bind.encoded_id == PointBindData::kInvalid) {
        			if(bindMissingToTriface) {

        				// TODO: make separate point clouds (root and tip guide points) to avoid binding to same guide points

        				deformer_restpoints_kdtree.findKNearestNeighbours(curr_pt, 3, closest_deformer_points);

        				const uint32_t face_id = pPhantomTrimesh->getOrCreateFaceID(closest_deformer_points[0].first, closest_deformer_points[1].first, closest_deformer_points[2].first);
        				const auto& face = faces[face_id];

						const pxr::GfVec3f& p0 = pPhantomTrimesh->getRestPointPosition(face.indices[0]);
						const pxr::GfVec3f& p1 = pPhantomTrimesh->getRestPointPosition(face.indices[1]);
						const pxr::GfVec3f& p2 = pPhantomTrimesh->getRestPointPosition(face.indices[2]);

						const auto& face_normal = face.getRestNormal();
						const Plane face_plane(p0, face_normal);
						const float face_distance = distance(face_plane, curr_pt);

						const pxr::GfVec3f projected_pt = curr_pt - face_normal * face_distance; // project point on to face plane

						const pxr::GfVec3f v0 = p1 - p0, v1 = p2 - p0, v2 = projected_pt - p0;
						float d00 = pxr::GfDot(v0, v0);
						float d01 = pxr::GfDot(v0, v1);
						float d11 = pxr::GfDot(v1, v1);
						float d20 = pxr::GfDot(v2, v0);
						float d21 = pxr::GfDot(v2, v1);
						float denom = d00 * d11 - d01 * d01;

						u = (d11 * d20 - d01 * d21) / denom;
						v = (d00 * d21 - d01 * d20) / denom;
						w = face_distance;
						bind.encodeID_modeSPACE(face_id, false /* not a tetra id */, false /* data is 3x32bit floats */);
						bind.setData(u, v, w);
        				bound_points++;
        			} else {
        				
        				// TODO: this needs to be checked. (weird shit happens)

        				pPhantomTrimesh->barycentricTetrahedronRestCoords(closest_tetra_centroid.first, curr_pt, u, v, w, x);
        				bind.encodeID_modeSPACE(closest_tetra_centroid.first, true /* is tetra id */, true /* data is 4x24bit floats */);
        				bind.setData(u, v, w, x);
        				bound_points++;
        			}
        			
        		}

        		if(bind.encoded_id == PointBindData::kInvalid) {
        			// We should not be here
        			assert((1 == 2) && "What the f..ck !?");
        			unboud_points++;
        		}
        	}
    	}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, curves_count, bind_func);
		mPool.wait();
	} else {
		bind_func(0u, curves_count);
	}

	LOG_DBG << "Total points: " << total_points.load();
	LOG_DBG << "Bound points: " << bound_points.load();
	LOG_DBG << "Unbound points: " << unboud_points.load();

	return true;
}

bool GuideCurvesDeformer::buildNTBFrames(std::vector<NTBFrame>& guide_frames, bool multi_threaded, bool build_live) {
	assert(mpGuideCurvesContainer);

	if(!mpSkinPhantomTrimeshData || !mpSkinPhantomTrimeshData->isValid()) {
		LOG_ERR << "Can't build NTB frames! No skin geometry data.";
		return false;
	}

	const PhantomTrimesh* pSkinPhantomTrimesh = mpSkinPhantomTrimeshData->getTrimesh();
	assert(pSkinPhantomTrimesh);

	const auto total_guides_count = mpGuideCurvesContainer->getCurvesCount();
	const auto& guide_points = build_live ? mpGuideCurvesContainer->getLiveCurvePoints() : mpGuideCurvesContainer->getRestCurvePoints();
	const pxr::VtArray<pxr::GfVec3f>& skin_points = build_live ? pSkinPhantomTrimesh->getLivePositions() : pSkinPhantomTrimesh->getRestPositions();

	const auto& guide_origins = mpGuideCurvesDeformerData->getGuideOrigins();

	auto frame_func = [&](const std::size_t start, const std::size_t end) {

		for(auto guide_id = start; guide_id < end; ++guide_id) {
			assert(guide_id < total_guides_count);

			const size_t guide_vertex_offset = mpGuideCurvesContainer->getCurveVertexOffset(guide_id);
			const size_t curve_points_count = mpGuideCurvesContainer->getCurveVertexCount(guide_id);
			const pxr::GfVec3f* pCurveRootPt = guide_points.data() + guide_vertex_offset;
			const pxr::GfVec3f& root_tangent = *(pCurveRootPt + 1) - *pCurveRootPt; 

			uint32_t face_id = GuideCurvesDeformerData::GuideOrigin::kInvalidFaceID;
			uint32_t axis_id = GuideCurvesDeformerData::GuideOrigin::kInvalidAxisID;

			assert(guide_id < guide_origins.size());
			guide_origins[guide_id].decode(face_id, axis_id);

			const PhantomTrimesh::TriFace& face = pSkinPhantomTrimesh->getFace(face_id);

			const pxr::GfVec3f up_vector = pxr::GfGetNormalized(skin_points[face.indices[0]] - skin_points[face.indices[1]]);
			// TODO: build proper root normal form this up vector

			std::vector<NTBFrame>::iterator it_begin = guide_frames.begin() + guide_vertex_offset;
			std::vector<NTBFrame>::iterator it_end = it_begin + curve_points_count;

			buildRotationMinimizingFrames(pCurveRootPt, curve_points_count, root_tangent, up_vector, it_begin, it_end);
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, total_guides_count, frame_func);
		mPool.wait();
	} else {
		frame_func(0u, total_guides_count);
	}

	return true;
}

bool GuideCurvesDeformer::buildGuideOrigins(bool multi_threaded) {
	assert(mpGuideCurvesContainer);
	assert(mpGuideCurvesDeformerData);
	assert(mpSkinAdjacencyData && mpSkinAdjacencyData->isValid());
	assert(mpSkinPhantomTrimeshData && mpSkinPhantomTrimeshData->isValid());

	const UsdGeomMeshFaceAdjacency* pSkinGeoAdjacency = mpSkinAdjacencyData->getAdjacency();
	PhantomTrimesh* pSkinGeoPhantomTrimesh = mpSkinPhantomTrimeshData->getTrimesh();

	const size_t guide_curves_count = mpGuideCurvesContainer->getCurvesCount();
	const std::vector<int>& skin_prim_indices =	mpGuideCurvesDeformerData->getSkinPrimIndices();

	if(skin_prim_indices.size() != guide_curves_count) {
		return false;
	}

	auto& guide_origins = mpGuideCurvesDeformerData->guideOrigins();
	guide_origins.resize(guide_curves_count);
	const pxr::VtArray<pxr::GfVec3f>& skin_geo_rest_points = pSkinGeoPhantomTrimesh->getRestPositions();

	std::vector<std::mutex> kdtrees_mutexes(guide_curves_count);  // protects kdree initialisation
	std::vector<std::unique_ptr<neighbour_search::KDTree<float, 3>>> kdtrees(guide_curves_count);

	auto func = [&](const std::size_t start, const std::size_t end) {
		std::vector<neighbour_search::KDTree<float, 3>::ReturnType> closest_deformer_points(3);

		for(size_t guide_id = start; guide_id < end; ++guide_id) {
			const neighbour_search::KDTree<float, 3>* pKDTree;

			const int skin_prim_id = skin_prim_indices[guide_id];
			assert(skin_prim_id >= 0 && ((uint32_t)skin_prim_id < pSkinGeoAdjacency->getFaceCount()));
			const uint32_t skin_prim_vtx_offset = pSkinGeoAdjacency->getFaceVertexOffset(skin_prim_id);

			// build guide kdtree if needed
        	const std::lock_guard<std::mutex> lock(kdtrees_mutexes[guide_id]);
        	if(!kdtrees[guide_id]) {
				const uint32_t skin_prim_vtx_count = pSkinGeoAdjacency->getFaceVertexCount(skin_prim_id);

				pxr::VtArray<pxr::GfVec3f> prim_points;
				for(uint32_t i = 0; i < skin_prim_vtx_count; ++i){
					prim_points.push_back(skin_geo_rest_points[pSkinGeoAdjacency->getFaceVertex(skin_prim_vtx_offset + i)]);
				}

        		kdtrees[guide_id] = std::make_unique<neighbour_search::KDTree<float, 3>>(prim_points, false /* no threads */);
        	}
        	pKDTree = kdtrees[guide_id].get();

			const pxr::GfVec3f& root_pt = mpGuideCurvesContainer->getGuideRestPoint(guide_id, 0 /* root vtx */);
        	pKDTree->findKNearestNeighbours(root_pt, 3, closest_deformer_points);

        	const PhantomTrimesh::PxrIndexType a = pSkinGeoAdjacency->getFaceVertex(skin_prim_vtx_offset + closest_deformer_points[0].first);
        	const PhantomTrimesh::PxrIndexType b = pSkinGeoAdjacency->getFaceVertex(skin_prim_vtx_offset + closest_deformer_points[1].first);
        	const PhantomTrimesh::PxrIndexType c = pSkinGeoAdjacency->getFaceVertex(skin_prim_vtx_offset + closest_deformer_points[2].first);

			const uint32_t face_id = pSkinGeoPhantomTrimesh->getOrCreateFaceID(a, b, c);

			uint32_t axis_id = 0; // TODO: find a proper axis id !
			guide_origins[guide_id].encode(face_id, axis_id);
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, guide_curves_count, func);
		mPool.wait();
	} else {
		func(0u, guide_curves_count);
	}

	return true;
}

bool GuideCurvesDeformer::buildDeformerDataNTBMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	multi_threaded = false;

	if(!buildSkinPrimData(multi_threaded)) {
		mpGuideCurvesDeformerData->skinPrimIndices().clear();
		mpGuideCurvesDeformerData->setSkinPrimPath("");
		LOG_ERR << "Error building skin prim geometry data !";
		return false;
	}	

	if(!mDeformerGeoPrimHandle.fetchAttributeValues<int>(mGuidesSkinPrimAttrName, mpGuideCurvesDeformerData->skinPrimIndices(), rest_time_code)) {
		LOG_ERR << "Error getting skin prim indices !";
		return false;
	}

	if(!buildGuideOrigins(multi_threaded)) {
		LOG_ERR << "NTB frames calulation without skin geometry primitive indices is NOT supported yet !";
		return false;
	}

	const size_t curves_count = mpCurvesContainer->getCurvesCount();
	assert(curves_count == mGuideIndices.size());
	const size_t curve_points_count = mpCurvesContainer->getTotalVertexCount();

	std::vector<NTBFrame> rest_guide_frames(mpGuideCurvesContainer->getRestCurvePoints().size());

	static const bool build_live = false;
	if(!buildNTBFrames(rest_guide_frames, multi_threaded, build_live)) {
		return false;
	}

	// inverse frames
	std::vector<pxr::GfMatrix3f> matrices(rest_guide_frames.size());
	for(size_t i = 0; i < rest_guide_frames.size(); ++i) {
		matrices[i] = rest_guide_frames[i].getMatrix3f().GetInverse();
	}

	auto& pointBinds = mpGuideCurvesDeformerData->pointBinds();
	const PxrCurvesContainer* pCurvesContainer = mpCurvesContainer.get();
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	const size_t guide_curves_count = mpGuideCurvesContainer->getCurvesCount();

	std::vector<std::mutex> kdtrees_mutexes(guide_curves_count);  // protects kdree initialisation
	std::vector<std::unique_ptr<neighbour_search::KDTree<float, 3>>> kdtrees(guide_curves_count);
	const auto& guides_rest_points = mpGuideCurvesContainer->getRestCurvePoints();

	const bool fast_bind = isFastPointBind();

	auto func = [&](const std::size_t start, const std::size_t end) {
		std::vector<std::pair<size_t, size_t>> guide_segment_vetrices_pairs(256); // our limit is 256 vertices per guide curve
		std::vector<float> guide_segment_squared_distances(256);

		for(size_t curve_index = start; curve_index < end; ++curve_index) {
			const uint32_t guide_id = (uint32_t)mGuideIndices[curve_index];
			assert(guide_id < guide_curves_count);
			const size_t guide_vertex_count = mpGuideCurvesContainer->getCurveVertexCount(guide_id);
			const size_t guide_vertex_offset = mpGuideCurvesContainer->getCurveVertexOffset(guide_id);

        	const neighbour_search::KDTree<float, 3>* pKDTree;

			PxrCurvesContainer::CurveDataConstPtr curve_data_ptr = pCurvesContainer->getCurveDataPtr(curve_index);
        	const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);

			if(fast_bind) {
				// build guide kdtree if needed
				const std::lock_guard<std::mutex> lock(kdtrees_mutexes[guide_id]);
				if(!kdtrees[guide_id]) {
					kdtrees[guide_id] = std::make_unique<neighbour_search::KDTree<float, 3>>(guides_rest_points, guide_vertex_offset, guide_vertex_count, false /* no threads */);
				}
				pKDTree = kdtrees[guide_id].get();
			} else {
				// accurate binding mode
				size_t ii = 0;
				for(size_t i = guide_vertex_offset; i < (guide_vertex_offset + guide_vertex_count - 1); ++i) {
					guide_segment_vetrices_pairs[ii++] = {i, i + 1};
				}
			}

			const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_index);
			const uint32_t curve_vertex_offset = mpCurvesContainer->getCurveVertexOffset(curve_index);
		
			for(uint32_t i = 0; i < curve_vertices_count; ++i) {
				auto& bind = pointBinds[curve_vertex_offset + i];
				const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 
				uint32_t frame_id = PointBindData::kInvalid;

				if(fast_bind) {
					const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = pKDTree->findNearestNeighbour(curr_pt);
					if(nearest_pt.first == guide_vertex_offset || nearest_pt.first == (guide_vertex_offset + guide_vertex_count - 1)) {
						// guide curve edge vertices
						frame_id = nearest_pt.first;
					} else {
						// rest of guide curve vertices
						const float sd1 = distanceSquared(curr_pt, guides_rest_points[nearest_pt.first], guides_rest_points[nearest_pt.first - 1]);
						const float sd2 = distanceSquared(curr_pt, guides_rest_points[nearest_pt.first], guides_rest_points[nearest_pt.first + 1]);
						if(sd1 < sd2) {
							frame_id = nearest_pt.first - 1;
						} else {
							frame_id = nearest_pt.first;
						}
					}
				} else {
					for(auto i = 0; i < guide_vertex_count; ++i) {
						const auto& pair = guide_segment_vetrices_pairs[i];
						guide_segment_squared_distances[i] = distanceSquared(curr_pt, guides_rest_points[pair.first], guides_rest_points[pair.second]);
					}
					std::vector<float>::iterator min_val_it = std::min_element(std::begin(guide_segment_squared_distances), std::begin(guide_segment_squared_distances) + guide_vertex_count);
					frame_id = guide_vertex_offset + std::distance(std::begin(guide_segment_squared_distances), min_val_it);
				}

				bind.encodeID_modeNTB(frame_id);

				assert(frame_id < matrices.size());
				bind.setData(matrices[frame_id] * (curr_pt - guides_rest_points[frame_id]));
        	}
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, curves_count, func);
		mPool.wait();
	} else {
		func(0u, curves_count);
	}

	return true;
}

bool GuideCurvesDeformer::buildDeformerDataAngleMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	const size_t guide_curves_count = mpGuideCurvesContainer->getCurvesCount();

	std::vector<std::mutex> kdtrees_mutexes(guide_curves_count);  // protects kdree initialisation
	std::vector<std::unique_ptr<neighbour_search::KDTree<float, 3>>> kdtrees(guide_curves_count);

	const size_t curves_count = mpCurvesContainer->getCurvesCount();

	assert(curves_count == mGuideIndices.size());

	auto& pointBinds = mpGuideCurvesDeformerData->mPointBinds;
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	auto func = [&](const size_t curve_index) {
		assert(mGuideIndices[curve_index] >= 0);
    	const uint32_t guide_id = (uint32_t)mGuideIndices[curve_index];
    	assert(guide_id < guide_curves_count);
        const neighbour_search::KDTree<float, 3>* pKDTree;
        
        const auto& guides_rest_points = mpGuideCurvesContainer->getRestCurvePoints();
        const size_t guide_vertex_count = mpGuideCurvesContainer->getCurveVertexCount(guide_id);
        const size_t guide_vertex_offset = mpGuideCurvesContainer->getCurveVertexOffset(guide_id);

		{
			// build guide kdtree if needed
			const std::lock_guard<std::mutex> lock(kdtrees_mutexes[guide_id]);
			if(!kdtrees[guide_id]) {
				kdtrees[guide_id] = std::make_unique<neighbour_search::KDTree<float, 3>>(guides_rest_points, guide_vertex_offset, guide_vertex_count, false /* no threads */);
			}
			pKDTree = kdtrees[guide_id].get();
		}

        PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_index);

        const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);

        const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_index);
        const uint32_t curve_vertex_offset = mpCurvesContainer->getCurveVertexOffset(curve_index);

        for(uint32_t i = 0; i < curve_vertices_count; ++i) {
        	auto& bind = pointBinds[curve_vertex_offset + i];
        	bind.encoded_id = PointBindData::kInvalid;
        	const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 

        	const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = pKDTree->findNearestNeighbour(curr_pt);
			
			const uint32_t guide_vertex_id = nearest_pt.first - guide_vertex_offset; // we have global point indices in kdtree. by substracting we a re making them local to the specific guide curve
			const uint32_t segment_id = std::min(guide_vertex_id, (uint32_t)guide_vertex_count - 2u); // exclude last vertex index

			const pxr::GfVec3f& guide_pt = mpGuideCurvesContainer->getGuideRestPoint(guide_id, segment_id);
			
			bind.encodeID_modeANGLE(guide_id, segment_id);
        	bind.setData(curr_pt - guide_pt);
        }

    };

	if(multi_threaded) {
        BS::multi_future<void> loop = mPool.submit_loop(0, curves_count, func);
        loop.wait();
    } else {
        for(size_t i = 0; i < curves_count; ++i) {
            func(i);
        }
    }

    return true;
}

bool GuideCurvesDeformer::buildSkinPrimData(bool multi_threaded) {
	if(!mGuidesSkinGeoPrimHandle.isValid()) {
		LOG_ERR << "Unable to build guides skin primtive data. No primitive \"" << mGuidesSkinGeoPrimHandle.getFullName() << "\" found!";
		return false;
	}

	DeformerDataCache& dataCache = DeformerDataCache::getInstance();

	if(!mDirty && mpSkinAdjacencyData && mpSkinPhantomTrimeshData && 
		mpSkinAdjacencyData->isValid() && 
		mpSkinPhantomTrimeshData->isValid()) {
		return true;
	}

	if(!mpSkinAdjacencyData) {
		mpSkinAdjacencyData = dataCache.getOrCreateData<SerializableUsdGeomMeshFaceAdjacency>(mGuidesSkinGeoPrimHandle);
	}
	assert(mpSkinAdjacencyData);

	if(!getReadJsonDataState() || !mGuidesSkinGeoPrimHandle.getDataFromBson(mpSkinAdjacencyData.get())) {
		if(!mpSkinAdjacencyData->buildInPlace(mGuidesSkinGeoPrimHandle)) {
			LOG_ERR << "Error building guides skin adjacency data!";
			return false;
		}
	}

	if(!mpSkinPhantomTrimeshData) {
		mpSkinPhantomTrimeshData = dataCache.getOrCreateData<SerializablePhantomTrimesh>(mGuidesSkinGeoPrimHandle);
	}
	assert(mpSkinPhantomTrimeshData);

	if(!getReadJsonDataState() || !mGuidesSkinGeoPrimHandle.getDataFromBson(mpSkinPhantomTrimeshData.get())) {
		if(!mpSkinPhantomTrimeshData->buildInPlace(mGuidesSkinGeoPrimHandle, mGuidesSkinPrimRestAttrName)) {
			LOG_ERR << "Error building guides skin trimesh data!";
			return false;
		}
	}

	return true;
}

bool GuideCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(mpGuidesPhantomTrimeshData && !mDeformerGeoPrimHandle.writeDataToBson(mpGuidesPhantomTrimeshData.get())) {
		LOG_ERR << "Error writing " << mpGuidesPhantomTrimeshData->typeName() << " deformer data to json !";
		return false;
	}

	if(mpGuideCurvesDeformerData && !mCurvesGeoPrimHandle.writeDataToBson(mpGuideCurvesDeformerData.get())) {
		LOG_ERR << "Error writing " << mpGuideCurvesDeformerData->typeName() << " deformer data to json !";
		return false;
	}

	if(mpSkinAdjacencyData && mGuidesSkinGeoPrimHandle.writeDataToBson(mpSkinAdjacencyData.get())) {
		LOG_ERR << "Error writing " << mpSkinAdjacencyData->typeName() << " deformer data to json !";
		return false;
	}

	if(mpSkinPhantomTrimeshData && !mGuidesSkinGeoPrimHandle.writeDataToBson(mpSkinPhantomTrimeshData.get())) {
		LOG_ERR << "Error writing " << mpSkinPhantomTrimeshData->typeName() << " curves data to json !";
		return false;
	}

	return true;
}

void GuideCurvesDeformer::setBindRootsToSkinSurface(bool bind) {
	if( mBindRootsToSkinSurface == bind) return;
	mBindRootsToSkinSurface = bind;
	makeDirty();
}

void GuideCurvesDeformer::setBindMode(GuideCurvesDeformer::BindMode mode) {
	assert(mpGuideCurvesDeformerData);
	if(mpGuideCurvesDeformerData->getBindMode() == mode) return;
	mpGuideCurvesDeformerData->setBindMode(mode);
	makeDirty();
}

GuideCurvesDeformer::BindMode GuideCurvesDeformer::getBindMode() const {
	assert(mpGuideCurvesDeformerData);
	return mpGuideCurvesDeformerData->getBindMode();
}


GuideCurvesDeformer::~GuideCurvesDeformer() {
	PROFILE_PRINT();
}


} // namespace Piston