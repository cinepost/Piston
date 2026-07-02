#include "simple_profiler.h"
#include "guide_curves_deformer.h"
#include "deformer_data_cache.h"
#include "curves_container_utils.h"

#include "common.h"
#include "logging.h"
#include "kdtree.hpp"
#include "geometry_tools.h"
#include "tetrahedron.h"

#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/matrix4f.h>

#include <random>
#include <algorithm>
#include <mutex>
#include <atomic>


namespace Piston {

GuideCurvesDeformer::GuideCurvesDeformer(const std::string& name): BaseCurvesDeformer(BaseCurvesDeformer::Type::GUIDES, name) {
	mBindMode = BindMode::NTB;
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

	DLOG_DBG << "Guide ID attribute name is set to: " << mGuideIDPrimAttrName;
}

void GuideCurvesDeformer::setGuidesSkinGeoPrimAttrName(const std::string& name) {
	if(mGuidesSkinPrimAttrName == name) return;
	mGuidesSkinPrimAttrName = name;
	makeDirty();

	DLOG_DBG << "Guide skin prim attribute name is set to: " << mGuidesSkinPrimAttrName;
}

void GuideCurvesDeformer::setGuidesSkinGeoPrimRestAttrName(const std::string& name) {
	if(mGuidesSkinPrimRestAttrName == name) return;
	mGuidesSkinPrimRestAttrName = name;
	makeDirty();

	DLOG_DBG << "Guide skin prim rest attribute name is set to: " << mGuidesSkinPrimRestAttrName;
}

void GuideCurvesDeformer::setGuidesSkinGeoPrim(const pxr::UsdPrim& geoPrim) {
	if(mGuidesSkinGeoPrimHandle == geoPrim) return;
	
	if(!isMeshGeoPrim(geoPrim)) {
		mGuidesSkinGeoPrimHandle.clear();
		DLOG_ERR << "Invalid guides skin geometry prim " <<  geoPrim.GetPath().GetText() << " type!";
		return;
	}

	mpSkinAdjacencyData = nullptr;
	mpSkinPhantomTrimeshData = nullptr;
	mGuidesSkinGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	DLOG_DBG << "Guides skin geometry prim is set to: " << mGuidesSkinGeoPrimHandle.getPath().GetText();
}

bool GuideCurvesDeformer::deformImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	PROFILE("GuideCurvesDeformer::deformImpl");
	return __deform__(points, false, time_code);
}

bool GuideCurvesDeformer::deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	PROFILE("GuideCurvesDeformer::deformMtImpl");
	return __deform__(points, true, time_code);
}

bool GuideCurvesDeformer::__deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code) {
	if(!mpGuideCurvesContainer->update(mDeformerGeoPrimHandle, time_code, isDirty())) {
		DLOG_ERR << "Error updating guide curves from prim" << mDeformerGeoPrimHandle.getPath().GetText() << " !";
		return false;
	}

	const auto bind_mode = getBindMode();
	DLOG_TRC << "GuideCurvesDeformer::__deform__() mode " << to_string(bind_mode) << " " << (multi_threaded ? "multi_threaded" : "single thread");
	
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
		case BindMode::BLEND:
			result = deformImpl_BlendNTBMode(multi_threaded, points, time_code);
			break;
		case BindMode::LHS:
			result = deformImpl_LHSMode(multi_threaded, points, time_code);
			break;
		default:
			assert(false && "Unimplemented GuideCurvesDeformer::BindMode");
			DLOG_ERR << "Unimplemented bind mode " << to_string(bind_mode) << " !!!";
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

	assert(mpSkinPhantomTrimeshData->isValid());
	PhantomTrimesh* pSkinPhantomTrimesh = mpSkinPhantomTrimeshData->getTrimesh();
	assert(pSkinPhantomTrimesh);

	assert(mpSkinMeshContainer);
	const MeshContainer* pSkinMeshContainer = mpSkinMeshContainer.get();

	pSkinMeshContainer->update(mGuidesSkinGeoPrimHandle, time_code, isDirty());

	mTempSkinFaceLiveNormals.resize(pSkinPhantomTrimesh->getFaceCount());
	TODO(precalculate skin live face normals first !!!)

	const auto& pointBinds = mpGuideCurvesDeformerData->getPointSurfaceBinds();

	auto func = [&](const std::size_t start, const std::size_t end) {
		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			assert(bind.point_id < points.size());

			const PhantomTrimesh::TriFace& skin_face = pSkinPhantomTrimesh->getFace(bind.face_id);
			const pxr::GfVec3f face_normal = pSkinMeshContainer->getFaceLiveNormal(skin_face);
			auto pos = pSkinMeshContainer->getInterpolatedLivePosition(skin_face, bind.u, bind.v) + face_normal * bind.dist;
			points[bind.point_id] = pos * bind.weight + points[bind.point_id] * (1.f - bind.weight);
		}
	};

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, pointBinds.size(), func);
		blocks.wait();
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
			const pxr::GfVec3f rest_segment_vector_n = pxr::GfGetNormalized(guides_rest_points[guide_segment_start_vtx + 1] - guides_rest_points[guide_segment_start_vtx], MIN_VECTOR_LENGTH_F);
			const pxr::GfVec3f live_segment_vector_n = pxr::GfGetNormalized(guides_live_points[guide_segment_start_vtx + 1] - guides_live_points[guide_segment_start_vtx], MIN_VECTOR_LENGTH_F);

			const pxr::GfMatrix3f m = rotateAlign(rest_segment_vector_n, live_segment_vector_n);

			points[i] = guides_live_points[guide_segment_start_vtx] + (m * vec);
		}
	};

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, pointBinds.size(), func);
		blocks.wait();
	} else {
		func(0u, pointBinds.size());
	}

	return true;
}

bool GuideCurvesDeformer::hasSkinPrimitiveData() const {
	if(!mGuidesSkinGeoPrimHandle.isValid()) return false;

	if(!mpSkinPhantomTrimeshData) {
		DLOG_ERR << "Has no skin primitive data!";
		return false;
	}

	if(!mpSkinPhantomTrimeshData->isValid()) {
		DLOG_ERR << "Has no valid skin primitive data!";
		return false;
	}

	return mpSkinPhantomTrimeshData && mpSkinPhantomTrimeshData->isValid();
}

bool GuideCurvesDeformer::deformImpl_LHSMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
	const auto& pointBinds = mpGuideCurvesDeformerData->getLHSPointBinds();

	auto func = [&](const std::size_t start, const std::size_t end) {
		const auto* pGuideCurvesContainer = mpGuideCurvesContainer.get();
		const auto& positions = pGuideCurvesContainer->getLiveCurvePoints();

		for(size_t i = start; i < end; ++i) {
			assert(i < pointBinds.size());

			const auto& bind = pointBinds[i];
			points[i] = {0.0f, 0.0f, 0.0f};

			if(bind.isValid()) {
				points[i] += positions[bind.v[0] + pGuideCurvesContainer->getCurveVertexOffset(bind.curveIndices[0])] * bind.w[0];
				points[i] += positions[bind.v[1] + pGuideCurvesContainer->getCurveVertexOffset(bind.curveIndices[0])] * bind.w[1];

				points[i] += positions[bind.v[2] + pGuideCurvesContainer->getCurveVertexOffset(bind.curveIndices[1])] * bind.w[2];
				points[i] += positions[bind.v[3] + pGuideCurvesContainer->getCurveVertexOffset(bind.curveIndices[1])] * bind.w[3];
				
				points[i] += positions[bind.v[4] + pGuideCurvesContainer->getCurveVertexOffset(bind.curveIndices[2])] * bind.w[4];
				points[i] += positions[bind.v[5] + pGuideCurvesContainer->getCurveVertexOffset(bind.curveIndices[2])] * bind.w[5];

			}
		}
	};

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, pointBinds.size(), func);
		blocks.wait();
	} else {
		func(0u, pointBinds.size());
	}

	return true;
}

bool GuideCurvesDeformer::deformImpl_NTBMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
	assert(mpGuideCurvesDeformerData);
	assert(mpGuideCurvesContainer);

	assert(hasSkinPrimitiveData()); // for now we only work with skin geometry
	assert(mpSkinPhantomTrimeshData->isValid());
	PhantomTrimesh* pSkinPhantomTrimesh = hasSkinPrimitiveData() ? mpSkinPhantomTrimeshData->getTrimesh() : nullptr;
	assert(pSkinPhantomTrimesh);

	assert(mpSkinMeshContainer);
	mpSkinMeshContainer->update(mGuidesSkinGeoPrimHandle, time_code, isDirty());

	const auto& guides_live_points = mpGuideCurvesContainer->getLiveCurvePoints();

	const auto& pointBinds = mpGuideCurvesDeformerData->getPointBinds();
	std::vector<NTBFrame> live_guide_frames(mpGuideCurvesContainer->getLiveCurvePoints().size());

	if(!buildNTBFrames(live_guide_frames, multi_threaded, true /* build live */)) {
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
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, pointBinds.size(), func);
		blocks.wait();
	} else {
		func(0u, pointBinds.size());
	}

	return true;
}

bool GuideCurvesDeformer::deformImpl_BlendNTBMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
	const auto* pGuideCurvesContainer = mpGuideCurvesContainer.get();
	assert(pGuideCurvesContainer);

	assert(hasSkinPrimitiveData()); // for now we only work with skin geometry

	assert(mpSkinMeshContainer);
	mpSkinMeshContainer->update(mGuidesSkinGeoPrimHandle, time_code, isDirty());

	const auto& guides_live_points = mpGuideCurvesContainer->getLiveCurvePoints();
	const auto& pointBinds = mpGuideCurvesDeformerData->getBlendNTBPointBinds();
	assert(pointBinds.size() == points.size());

	std::vector<NTBFrame> live_guide_frames(mpGuideCurvesContainer->getLiveCurvePoints().size());
	if(!buildNTBFrames(live_guide_frames, multi_threaded, true /* build live */)) {
		return false;
	}

	auto func = [&](const std::size_t start, const std::size_t end) {
		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			if(!bind.isValid()) continue;

			points[i] = {0.0f, 0.0f, 0.f};

			// first guide
			uint32_t frame_id_0 = pGuideCurvesContainer->getCurveVertexOffset(bind.guide_id[0]) + bind.v[0];
			
			points[i] += (guides_live_points[frame_id_0] + live_guide_frames[frame_id_0] * bind.coords[0]) * bind.w[0];

			// second guide
			if(bind.guide_id[1] == GuideCurvesDeformerData::BlendedNTBData::kInvalidCurveID) continue;
			uint32_t frame_id_1 = pGuideCurvesContainer->getCurveVertexOffset(bind.guide_id[1]) + bind.v[1];

			points[i] += (guides_live_points[frame_id_1] + live_guide_frames[frame_id_1] * bind.coords[1]) * bind.w[1];

			// third guide
			if(bind.guide_id[2] == GuideCurvesDeformerData::BlendedNTBData::kInvalidCurveID) continue;
			uint32_t frame_id_2 = pGuideCurvesContainer->getCurveVertexOffset(bind.guide_id[2]) + bind.v[2];

			points[i] += (guides_live_points[frame_id_2] + live_guide_frames[frame_id_2] * bind.coords[2]) * bind.w[2];
		}
	};

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, pointBinds.size(), func);
		blocks.wait();
	} else {
		func(0u, pointBinds.size());
	}

	return true;
}

bool GuideCurvesDeformer::deformImpl_SpaceMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code) {
	assert(mpGuidesPhantomTrimeshData);
	PhantomTrimesh* pPhantomTrimesh = mpGuidesPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);


	assert(mpDeformerMeshContainer);
	if(!mpDeformerMeshContainer->update(mDeformerGeoPrimHandle, time_code, isDirty())) {
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
				points[i] = mpDeformerMeshContainer->getPointPositionFromBarycentricTetrahedronLiveCoords(tetra, u, v, w, x);
			} else {
				// bound to triface
				TODO(precalculate skin live face normals first for SPACE mode !!!)

				const auto& face = pPhantomTrimesh->getFace(bind.encoded_id.mode_space.element_id);
				const pxr::GfVec3f face_normal = mpDeformerMeshContainer->getFaceLiveNormal(face);
				bind.getData(u, v, w);
				points[i] = mpDeformerMeshContainer->getInterpolatedLivePosition(face, u, v) + (face_normal * w);
			}
		}
	};

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, pointBinds.size(), func);
		blocks.wait();
	} else {
		func(0u, pointBinds.size());
	}

    return true;
}

bool GuideCurvesDeformer::
buildCurvesRootsBindDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	assert(mpCurvesContainer);
	assert(mpGuideCurvesContainer);
	assert(mpGuideCurvesDeformerData);
	assert(mpSkinAdjacencyData && mpSkinAdjacencyData->isValid());
	assert(mpSkinPhantomTrimeshData);

	DLOG_INF << " Binding " << mCurvesGeoPrimHandle << " root points to skin surface.";

	const UsdGeomMeshFaceAdjacency* pSkinAdjacency = mpSkinAdjacencyData->getAdjacency();
	PhantomTrimesh* pSkinPhantomTrimesh = mpSkinPhantomTrimeshData->getTrimesh();
	const PxrCurvesContainer* pCurvesContainer = mpCurvesContainer.get();

	const size_t curves_count = pCurvesContainer->getCurvesCount();

	std::vector<int> skin_prim_indices;

	if(getSkinPrimAttrName().empty() || !mCurvesGeoPrimHandle.fetchAttributeValues(getSkinPrimAttrName(), skin_prim_indices, rest_time_code) || (skin_prim_indices.size() == 0)) {

		// if there is no curves skinprim attr exist we can still try to promote it from guides ...
		if(mGuideIndices.size() == curves_count) {
			std::vector<int> guides_skin_prim_indices;
			if(mDeformerGeoPrimHandle.fetchAttributeValues(getGuidesSkinGeoPrimAttrName(), guides_skin_prim_indices, rest_time_code)) {
				skin_prim_indices.resize(curves_count);

				for(size_t i = 0; i < curves_count; ++i) {
					skin_prim_indices[i] = guides_skin_prim_indices[mGuideIndices[i]];
				}
			} else {
				DLOG_ERR << "Unable to promote guide skin prim attribute to curves.";
				return false;
			}
		} else {
			DLOG_ERR << "Skin prim ID is needed to bind curves roots for now !!!";
			return false;
		}
	} else {
		return false;
	}

	const bool is_per_vertex_attr = pCurvesContainer->getTotalVertexCount() == skin_prim_indices.size();
	const bool is_per_curve_attr = pCurvesContainer->getCurvesCount() == skin_prim_indices.size();

	if((skin_prim_indices.size() > 0) && !is_per_vertex_attr && !is_per_curve_attr) {
		DLOG_ERR << "Wrong skin prim ID attribute values count (" << skin_prim_indices.size() << ")! Should be per-curve or per-curve-vertex !";
		return false;
	}

	const int max_skin_prim_id = static_cast<int>(pSkinAdjacency->getFaceCount()) - 1;

	if(is_per_vertex_attr) {
		auto err_log_stream = Logger::getInstance().getStream(LogLevel::FATAL);
		if(!validatePrimIndices(skin_prim_indices, pCurvesContainer->getTotalVertexCount(), max_skin_prim_id, &err_log_stream)) {
			return false;
		}
	} else if (is_per_curve_attr) {
		auto err_log_stream = Logger::getInstance().getStream(LogLevel::FATAL);
		if(!validatePrimIndices(skin_prim_indices, pCurvesContainer->getCurvesCount(), max_skin_prim_id, &err_log_stream)) {
			return false;
		}
	}

	std::mutex binds_mutex;
	auto& point_surface_binds = mpGuideCurvesDeformerData->pointSurfaceBinds();
	point_surface_binds.clear();
	point_surface_binds.reserve(curves_count);

	assert(mpSkinMeshContainer);
	const auto* pSkinMeshContainer = mpSkinMeshContainer.get();

	auto bindPointToSkinPrim = [&] (const pxr::GfVec3f& pt, PointSurfaceBindData& bind, uint32_t prim_id, std::vector<float>& _tmp_sq_distances, bool ignore_prim_boundaries = false) {
		bool is_bound = false;
		const uint32_t prim_vertex_count = pSkinAdjacency->getFaceVertexCount(prim_id);
		const uint32_t prim_vertex_offset = pSkinAdjacency->getFaceVertexOffset(prim_id);

		uint32_t face_id;
		float u, v, dist;

		if( prim_vertex_count > 3u){
			if(_tmp_sq_distances.size() < prim_vertex_count) _tmp_sq_distances.resize(prim_vertex_count);
				
			const MeshContainer::ContainerType& rest_positions = pSkinMeshContainer->getRestPositions();

			for(size_t j = 0; j < prim_vertex_count; ++j) {
				_tmp_sq_distances[j] = distanceSquared(pt, rest_positions[pSkinAdjacency->getFaceVertex(prim_vertex_offset + j)]);
			}

			std::vector<float>::iterator it = std::min_element(_tmp_sq_distances.begin(), _tmp_sq_distances.begin() + prim_vertex_count);
			uint32_t local_index = std::distance(std::begin(_tmp_sq_distances), it);

			static const float kFLT_MAX = std::numeric_limits<float>::max();

			float pt_tri_dist_sq_min = kFLT_MAX;
			uint32_t _face_id = PhantomTrimesh::kInvalidTriFaceID; 
			float _u, _v, _dist;
			for(uint32_t i = 1; i < (prim_vertex_count - 1); ++i) {
				_face_id = pSkinPhantomTrimesh->getOrCreateFaceID(
					pSkinAdjacency->getFaceVertex(prim_id, local_index), 
					pSkinAdjacency->getFaceVertex(prim_id, (local_index + i) % prim_vertex_count),
					pSkinAdjacency->getFaceVertex(prim_id, (local_index + i + 1) % prim_vertex_count)
				);

				const auto& face = pSkinPhantomTrimesh->getFace(_face_id);
				is_bound = pSkinMeshContainer->projectPoint(pt, face, _u, _v, _dist);
				
				if(is_bound) {
					face_id = _face_id; u = _u; v = _v; dist = _dist;
					break;
				} else if(ignore_prim_boundaries) {
					// If outside we push point to triangle squared distance for later closest search
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
			is_bound = pSkinMeshContainer->projectPoint(pt, pSkinPhantomTrimesh->getFace(face_id), u, v, dist);
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
			const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);
			if(curve_vertices_count < 2) continue;

			const uint32_t curve_vertex_offset = pCurvesContainer->getCurveVertexOffset(curve_index);
        	const pxr::GfVec3f& curve_root_pt = pCurvesContainer->getCurveRootPoint(curve_index);

			for(uint32_t i = 0; i < curve_vertices_count; ++i) {
        		const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i);

				PointSurfaceBindData bind;
				bind.point_id = curve_vertex_offset + i;
				bind.weight = 1.0f;

				const uint32_t prim_id = is_per_vertex_attr ? skin_prim_indices[bind.point_id] : skin_prim_indices[curve_index];
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
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, curves_count, func);
		blocks.wait();
	} else {
		func(0u, curves_count);
	}

	mpSkinPhantomTrimeshData->setValid(true);
	DLOG_DBG << point_surface_binds.size() << " points are bound to skin surface.";

	return true;
}

bool GuideCurvesDeformer::guideIndicesNeeded() const {
	if(getBindRootsToSkinSurface()) return true;
	if(getBindMode() == BindMode::NTB) return true;
	if(getBindMode() == BindMode::BLEND) return true;
	if(getBindMode() == BindMode::ANGLE) return true;
	if(getBindMode() == BindMode::LHS) return true;

	return false;
}

bool GuideCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!mpGuideCurvesContainer) {
		mpGuideCurvesContainer = GuideCurvesContainer::create();
	}

	if(!mpGuideCurvesContainer->init(mDeformerGeoPrimHandle, getDeformerRestAttrName(), rest_time_code)) {
		DLOG_ERR << "Error initializing guide curves container !";
		return false;
	}

	DeformerDataCache& dataCache = DeformerDataCache::getInstance();

	if(guideIndicesNeeded()) {
		if(getGuideIDPrimAttrName().empty()) {
			DLOG_ERR << "No guide id (clump_id) attribute name set but needed !";
			return false;
		}

		if(!mCurvesGeoPrimHandle.fetchAttributeValues(getGuideIDPrimAttrName(), mGuideIndices, rest_time_code)) {
			DLOG_ERR << "Error getting curves " << mCurvesGeoPrimHandle << " \"" << getGuideIDPrimAttrName() << "\" guide indices !";
			return false;
		}

		const auto total_guides_count = mpGuideCurvesContainer->getCurvesCount();
		const auto total_curves_count = mpCurvesContainer->getCurvesCount();

		if(mGuideIndices.size() != total_curves_count) {
			DLOG_ERR << "Guides count (" << total_guides_count << ") and guide indices count (" << mGuideIndices.size() << ") mismatch !";
			return false;
		}

		// check guide indices are not out of range
		int max_guide_index = 0;
		for(const int idx: mGuideIndices) {
			max_guide_index = std::max(max_guide_index, idx);
		}

		if(max_guide_index >= total_guides_count) {
			DLOG_ERR << "Curves " << mCurvesGeoPrimHandle << " guide indices are out of range !"; 
			return false;
		}
	}

	bool deformer_data_created = true;
	if(!mpGuideCurvesDeformerData) {
		mpGuideCurvesDeformerData = dataCache.getOrCreateData<GuideCurvesDeformerData>(this, {&mDeformerGeoPrimHandle, &mCurvesGeoPrimHandle, &mGuidesSkinGeoPrimHandle}, rest_time_code, deformer_data_created);
		mpGuideCurvesDeformerData->setBindMode(mBindMode);
	}

	bool skin_prim_data_created = true;
	if(getBindRootsToSkinSurface() || getBindMode() == BindMode::NTB) {
		if(!buildSkinPrimData(multi_threaded, rest_time_code, skin_prim_data_created)) {
			DLOG_ERR << "Error building skin geometry data for " << mGuidesSkinGeoPrimHandle << "!";
			return false;
		}

		if( skin_prim_data_created ) {
			if(!buildCurvesRootsBindDeformerData(rest_time_code, multi_threaded)) {
				DLOG_ERR << "Error binding curves roots to skin geometry!";
				return false;
			}
		}
	}

	if(deformer_data_created || skin_prim_data_created || !mpGuideCurvesDeformerData->isValid() || !getReadJsonDataState() || !mGuidesSkinGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpGuideCurvesDeformerData.get())) {

		if(getBindRootsToSkinSurface()) {
			if(!hasSkinPrimitiveData()) {
				DLOG_ERR << "Unable to build bind data for " << mCurvesGeoPrimHandle << "! No skin primitive data was built!";
				return false;
			}
		}

		bool result = false;

		switch(getBindMode()) {
			case BindMode::SPACE:
				result = buildDeformerDataSpaceMode(rest_time_code, multi_threaded);
				break;
			case BindMode::NTB:
				result = buildDeformerDataNTBMode(rest_time_code, multi_threaded);
				break;
			case BindMode::LHS:
				result = buildDeformerDataLHSMode(rest_time_code, multi_threaded);
				break;
			case BindMode::BLEND:
				result = buildDeformerDataBlendNTBMode(rest_time_code, multi_threaded);
				break;
			default:
				result = buildDeformerDataAngleMode(rest_time_code, multi_threaded);
				break;
		}

		mpGuideCurvesDeformerData->setValid(result);
	}

	return mpGuideCurvesDeformerData->isValid();
}

bool GuideCurvesDeformer::buildDeformerDataSpaceMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	DeformerDataCache& dataCache = DeformerDataCache::getInstance();
	
	bool guides_trimesh_data_created = true;
	if(!mpGuidesPhantomTrimeshData) {
		mpGuidesPhantomTrimeshData = dataCache.getOrCreateData<SerializablePhantomTrimesh>(this, {&mDeformerGeoPrimHandle, &mCurvesGeoPrimHandle, &mGuidesSkinGeoPrimHandle}, rest_time_code, guides_trimesh_data_created);
	}

	if(guides_trimesh_data_created || !mpGuidesPhantomTrimeshData->isValid()) {
		if(!getReadJsonDataState() || !mDeformerGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpGuidesPhantomTrimeshData.get())) {
			// Build in place if no json data present or not needed
			if(!mpGuidesPhantomTrimeshData->buildInPlace(mDeformerGeoPrimHandle, getDeformerRestAttrName())) {
				DLOG_ERR << "Error building phantom mesh data!";
				return false;
			}
		}

		mpGuidesPhantomTrimeshData->setValid(false);

		PhantomTrimesh* pPhantomTrimesh = mpGuidesPhantomTrimeshData->getTrimesh();
		assert(pPhantomTrimesh);

		assert(mpDeformerMeshContainer);
		const auto* pDeformerMeshContainer = mpDeformerMeshContainer.get();

		if(!pPhantomTrimesh->hasTetrahedrons()) {
			assert(mpDeformerMeshContainer);
			if(!pPhantomTrimesh->buildTetrahedrons(mpDeformerMeshContainer->getRestPositions(), mpGuideCurvesContainer.get())) {
				DLOG_ERR << "Error building tetrahedrons!";
				return false;
			}
		}

		const std::vector<PhantomTrimesh::Tetrahedron>& tetrahedrons = pPhantomTrimesh->getTetrahedrons();

		DLOG_DBG << "Tetrahedrons count " << tetrahedrons.size();

		// Build kdtree
		auto buildTetrahedronsRestCentroidsKDTree = [&](bool multi_threaded) {
			pxr::VtArray<pxr::GfVec3f> centroids(tetrahedrons.size());
			for(uint32_t t_id = 0; t_id < tetrahedrons.size(); ++t_id) {
				centroids[t_id] = pDeformerMeshContainer->getTetrahedronRestCentroid(pPhantomTrimesh->getTetrahedron(t_id));
			}

			return neighbour_search::KDTree<float, 3>(centroids, multi_threaded);
		};

		auto buildRestPositionsKDTree = [&](bool multi_threaded) {
			return neighbour_search::KDTree<float, 3>(pDeformerMeshContainer->getRestPositions(), multi_threaded);
		};

		//neighbour_search::KDTree<float, 3> centroids_kdtree = buildTetrahedronsRestCentroidsKDTree(multi_threaded);
		neighbour_search::KDTree<float, 3> deformer_restpoints_kdtree = buildRestPositionsKDTree(multi_threaded);

		const size_t curves_count = mpCurvesContainer->getCurvesCount();

		assert(mpGuideCurvesDeformerData);
		auto& pointBinds = mpGuideCurvesDeformerData->mPointBinds;
		pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

		std::atomic<size_t> total_points = 0;
		std::atomic<size_t> bound_points = 0;
		std::atomic<size_t> unboud_points = 0;
		
		TetrahedronKDTree<PhantomTrimesh::PxrIndexType> tetraKDTree(pPhantomTrimesh->getTetrahedrons(), pDeformerMeshContainer->getRestPositions());

		auto bind_func = [&](const std::size_t start, const std::size_t end) {
	    	if(multi_threaded) {
	    		const std::optional<std::size_t> thread_index = BS::this_thread::get_index();
	    		LOG_TRC << "Binding curves from " << start << " to " << end << " by thread id #" << *thread_index;
	    	}

	    	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> sorted_tetra_centroids; // sorted tetrahedron centroids for brute force search
	    	sorted_tetra_centroids.reserve(tetrahedrons.size());

	    	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> closest_deformer_points(3);

	    	const auto& faces = pPhantomTrimesh->getFaces();

	    	TetrahedronKDTree<PhantomTrimesh>::TetraIndexType cachedTetraIndex = TetrahedronKDTree<PhantomTrimesh>::kInvalidTetraID;

			for(size_t curve_idx = start; curve_idx < end; ++curve_idx) {
	    		const PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_idx);

	    		const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);

	        	const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_idx);
	        	const uint32_t curve_vertex_offset = mpCurvesContainer->getCurveVertexOffset(curve_idx);

	        	float u, v, w, x;

	        	for(uint32_t i = 0; i < curve_vertices_count; ++i) {
	        		total_points++;

	        		const size_t curr_point_index = curve_vertex_offset + i;
	        		auto& bind = pointBinds[curr_point_index];
	        		bind.encoded_id = PointBindData::kInvalid;

	        		const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 

	        		bool is_inside = false;
	        		
	        		//auto tetra_index = tetraKDTree.queryPoint(curr_pt, is_inside);
	        		auto tetra_index = tetraKDTree.queryPointWithCache(curr_pt, cachedTetraIndex, is_inside);

	        		if(is_inside) {
	        			PROFILE("inside");
	        			pDeformerMeshContainer->barycentricTetrahedronRestCoords(pPhantomTrimesh->getTetrahedron(tetra_index), curr_pt, u, v, w, x);

	        			// bound inside tetra;
	        			bind.encodeID_modeSPACE(tetra_index, true /* is tetra id */, false /* data is 3x32bit floats */);
	        			bind.setData(u, v, w);
	        			bound_points++;
	        		} else {
	        			PROFILE("outside");
	        			// Bind to triface
						deformer_restpoints_kdtree.findKNearestNeighbours(curr_pt, 3, closest_deformer_points);

        				const uint32_t face_id = pPhantomTrimesh->getOrCreateFaceID(closest_deformer_points[0].first, closest_deformer_points[1].first, closest_deformer_points[2].first);
        				const auto& face = faces[face_id];

						const pxr::GfVec3f& p0 = pDeformerMeshContainer->getRestPointPosition(face.indices[0]);
						const pxr::GfVec3f& p1 = pDeformerMeshContainer->getRestPointPosition(face.indices[1]);
						const pxr::GfVec3f& p2 = pDeformerMeshContainer->getRestPointPosition(face.indices[2]);

						const auto& face_normal = pDeformerMeshContainer->getFaceRestNormal(face);
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
	        		}

	        		break;

	        		assert(bind.encoded_id != PointBindData::kInvalid && "SPACE mode encoded_id should not be PointBindData::kInvalid !");
	        	}
	    	}
		};

		DLOG_INF << "Binding curves to guides using SPACE mode.";

		if(multi_threaded) {
			BS::multi_future<void> blocks = mPool.submit_blocks(0u, curves_count, bind_func);
			blocks.wait();
		} else {
			bind_func(0u, curves_count);
		}

		DLOG_DBG << "Total points: " << total_points.load();
		DLOG_DBG << "Bound points: " << bound_points.load();
		DLOG_DBG << "Unbound points: " << unboud_points.load();

		SimpleProfiler::printReport();

		mpGuidesPhantomTrimeshData->setValid(true);
	}

	return mpGuidesPhantomTrimeshData->isValid();
}

bool GuideCurvesDeformer::buildNTBFrames(std::vector<NTBFrame>& guide_frames, bool multi_threaded, bool build_live) {
	assert(mpGuideCurvesContainer);

	if(!mpSkinPhantomTrimeshData || !mpSkinPhantomTrimeshData->isValid()) {
		DLOG_ERR << "Can't build NTB frames! No skin geometry data.";
		return false;
	}

	const PhantomTrimesh* pSkinPhantomTrimesh = mpSkinPhantomTrimeshData->getTrimesh();
	assert(pSkinPhantomTrimesh);

	const auto total_guides_count = mpGuideCurvesContainer->getCurvesCount();
	const auto& guide_points = build_live ? mpGuideCurvesContainer->getLiveCurvePoints() : mpGuideCurvesContainer->getRestCurvePoints();

	assert(mpSkinMeshContainer);
	const MeshContainer::ContainerType& skin_points = build_live ? mpSkinMeshContainer->getLivePositions() : mpSkinMeshContainer->getRestPositions();

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
			
			std::vector<NTBFrame>::iterator it_begin = guide_frames.begin() + guide_vertex_offset;
			std::vector<NTBFrame>::iterator it_end = it_begin + curve_points_count;

			buildRotationMinimizingFrames(pCurveRootPt, curve_points_count, root_tangent, up_vector, it_begin, it_end);
		}
	};

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, total_guides_count, frame_func);
		blocks.wait();
	} else {
		frame_func(0u, total_guides_count);
	}

	return true;
}

bool GuideCurvesDeformer::buildGuideOrigins(bool multi_threaded) {
	assert(mpGuideCurvesContainer);
	assert(mpGuideCurvesDeformerData);
	assert(mpSkinAdjacencyData && mpSkinAdjacencyData->isValid());
	assert(mpSkinPhantomTrimeshData);

	const UsdGeomMeshFaceAdjacency* pSkinGeoAdjacency = mpSkinAdjacencyData->getAdjacency();
	PhantomTrimesh* pSkinGeoPhantomTrimesh = mpSkinPhantomTrimeshData->getTrimesh();

	const size_t guide_curves_count = mpGuideCurvesContainer->getCurvesCount();
	const std::vector<int>& skin_prim_indices =	mpGuideCurvesDeformerData->getSkinPrimIndices();

	if(skin_prim_indices.size() != guide_curves_count) {
		return false;
	}

	auto& guide_origins = mpGuideCurvesDeformerData->guideOrigins();
	guide_origins.resize(guide_curves_count);

	assert(mpSkinMeshContainer);
	const auto* pSkinMeshContainer = mpSkinMeshContainer.get();

	const MeshContainer::ContainerType& skin_geo_rest_points = pSkinMeshContainer->getRestPositions();

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

			const pxr::GfVec3f& root_pt = mpGuideCurvesContainer->getCurveRestPoint(guide_id, 0 /* root vtx */);
        	pKDTree->findKNearestNeighbours(root_pt, 3, closest_deformer_points);

        	assert( closest_deformer_points[0].first != closest_deformer_points[1].first &&
        			closest_deformer_points[1].first != closest_deformer_points[2].first &&
        			closest_deformer_points[2].first != closest_deformer_points[0].first
        	);

        	const PhantomTrimesh::PxrIndexType a = pSkinGeoAdjacency->getFaceVertex(skin_prim_vtx_offset + closest_deformer_points[0].first);
        	const PhantomTrimesh::PxrIndexType b = pSkinGeoAdjacency->getFaceVertex(skin_prim_vtx_offset + closest_deformer_points[1].first);
        	const PhantomTrimesh::PxrIndexType c = pSkinGeoAdjacency->getFaceVertex(skin_prim_vtx_offset + closest_deformer_points[2].first);

			const uint32_t face_id = pSkinGeoPhantomTrimesh->getOrCreateFaceID(a, b, c);

			uint32_t axis_id = 0; // TODO: find a proper axis id !
			guide_origins[guide_id].encode(face_id, axis_id);
		}
	};

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, guide_curves_count, func);
		blocks.wait();
	} else {
		func(0u, guide_curves_count);
	}

	mpSkinPhantomTrimeshData->setValid(true);

	return true;
}


// Computes the determinant of a 3x3 matrix (helper for 4x4 inversion)
template<typename T>
static inline T det3x3(T m00, T m01, T m02,
              T m10, T m11, T m12,
              T m20, T m21, T m22) {
    return m00 * (m11 * m22 - m12 * m21) -
           m01 * (m10 * m22 - m12 * m20) +
           m02 * (m10 * m21 - m11 * m20);
}

// Fixed, highly accurate 4x4 matrix inverter using standard minor determinants
template<typename T>
static inline bool invert4x4_fixed(const T m[16], T invOut[16]) {
    T inv[16];

    inv[0] = m[5]  * m[10] * m[15] - 
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9]  * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5]  * m[15] + 
             m[8]  * m[7]  * m[13] + 
             m[12] * m[5]  * m[11] - 
             m[12] * m[7]  * m[9];

    inv[12] = -m[4]  * m[9]  * m[14] + 
               m[4]  * m[10] * m[13] + 
               m[8]  * m[5]  * m[14] - 
               m[8]  * m[6]  * m[13] - 
               m[12] * m[5]  * m[10] + 
               m[12] * m[6]  * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2]  * m[15] - 
              m[9]  * m[3]  * m[14] - 
              m[13] * m[2]  * m[11] + 
              m[13] * m[3]  * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2]  * m[15] + 
             m[8]  * m[3]  * m[14] + 
             m[12] * m[2]  * m[11] - 
             m[12] * m[3]  * m[10];

    inv[9] = -m[0]  * m[9]  * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1]  * m[15] - 
              m[8]  * m[3]  * m[13] - 
              m[12] * m[1]  * m[11] + 
              m[12] * m[3]  * m[9];

    inv[13] = m[0]  * m[9]  * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1]  * m[14] + 
              m[8]  * m[2]  * m[13] + 
              m[12] * m[1]  * m[10] - 
              m[12] * m[2]  * m[9];

    inv[2] = m[1]  * m[6]  * m[15] - 
             m[1]  * m[7]  * m[14] - 
             m[5]  * m[2]  * m[15] + 
             m[5]  * m[3]  * m[14] + 
             m[13] * m[2]  * m[7] - 
             m[13] * m[3]  * m[6];

    inv[6] = -m[0]  * m[6]  * m[15] + 
              m[0]  * m[7]  * m[14] + 
              m[4]  * m[2]  * m[15] - 
              m[4]  * m[3]  * m[14] - 
              m[12] * m[2]  * m[7] + 
              m[12] * m[3]  * m[6];

    inv[10] = m[0]  * m[5]  * m[15] - 
              m[0]  * m[7]  * m[13] - 
              m[4]  * m[1]  * m[15] + 
              m[4]  * m[3]  * m[13] + 
              m[12] * m[1]  * m[7] - 
              m[12] * m[3]  * m[5];

    inv[14] = -m[0]  * m[5]  * m[14] + 
               m[0]  * m[6]  * m[13] + 
               m[4]  * m[1]  * m[14] - 
               m[4]  * m[2]  * m[13] - 
               m[12] * m[1]  * m[6] + 
               m[12] * m[2]  * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    T det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (std::abs(det) < 1e-9) return false;

    T invDet = 1.0 / det;
    for (int i = 0; i < 16; i++) {
        invOut[i] = inv[i] * invDet;
    }
    return true;
}

// Inverts a 4x4 matrix using Cramer's Rule (fast and efficient for fixed sizes)
template<typename T>
static inline bool invert4x4(const T src[16], T dst[16]) {
    T sub[12];
    
    sub[0]  = src[10] * src[15] - src[11] * src[14];
    sub[1]  = src[9]  * src[15] - src[11] * src[13];
    sub[2]  = src[9]  * src[14] - src[10] * src[13];
    sub[3]  = src[8]  * src[15] - src[11] * src[12];
    sub[4]  = src[8]  * src[14] - src[10] * src[12];
    sub[5]  = src[8]  * src[13] - src[9]  * src[12];
    sub[6]  = src[6]  * src[15] - src[7]  * src[14];
    sub[7]  = src[5]  * src[15] - src[7]  * src[13];
    sub[8]  = src[5]  * src[14] - src[6]  * src[13];
    sub[9]  = src[4]  * src[15] - src[7]  * src[12];
    sub[10] = src[4]  * src[14] - src[6]  * src[12];
    sub[11] = src[4]  * src[13] - src[5]  * src[12];

    dst[0] =  (src[5] * sub[0] - src[6] * sub[1] + src[7] * sub[2]);
    dst[1] = -(src[1] * sub[0] - src[2] * sub[1] + src[3] * sub[2]);
    dst[2] =  (src[13] * det3x3<T>(src[1], src[2], src[3], src[5], src[6], src[7], src[9], src[10], src[11]));
    dst[3] = -(src[9]  * det3x3<T>(src[1], src[2], src[3], src[5], src[6], src[7], src[13], src[14], src[15]));
    
    dst[4] = -(src[4] * sub[0] - src[6] * sub[3] + src[7] * sub[4]);
    dst[5] =  (src[0] * sub[0] - src[2] * sub[3] + src[3] * sub[4]);
    dst[6] = -(src[12] * det3x3<T>(src[0], src[2], src[3], src[4], src[6], src[7], src[8], src[10], src[11]));
    dst[7] =  (src[8]  * det3x3<T>(src[0], src[2], src[3], src[4], src[6], src[7], src[12], src[14], src[15]));
    
    dst[8] =  (src[4] * sub[1] - src[5] * sub[3] + src[7] * sub[5]);
    dst[9] = -(src[0] * sub[1] - src[1] * sub[3] + src[3] * sub[5]);
    dst[10] = (src[12] * det3x3<T>(src[0], src[1], src[3], src[4], src[5], src[7], src[8], src[9], src[11]));
    dst[11] =-(src[8]  * det3x3<T>(src[0], src[1], src[3], src[4], src[5], src[7], src[12], src[13], src[15]));
    
    dst[12] = -(src[4] * sub[2] - src[5] * sub[4] + src[6] * sub[5]);
    dst[13] =  (src[0] * sub[2] - src[1] * sub[4] + src[2] * sub[5]);
    dst[14] = -(src[12] * det3x3<T>(src[0], src[1], src[2], src[4], src[5], src[6], src[8], src[9], src[10]));
    dst[15] =  (src[8]  * det3x3<T>(src[0], src[1], src[2], src[4], src[5], src[6], src[12], src[13], src[14]));

    T det = src[0] * dst[0] + src[1] * dst[4] + src[2] * dst[8] + src[3] * dst[12];
    
    if (std::abs(det) < 1e-9) return false; // Matrix is singular (points are co-planar/degenerate)

    T invDet = 1.0 / det;
    for (int i = 0; i < 16; ++i) dst[i] *= invDet;
    return true;
}

// Calculates 6 weights using A^T * (A * A^T)^-1
template<typename T>
static inline bool calculateWeights(const pxr::GfVec3f& P, const pxr::VtArray<pxr::GfVec3f>& points, const std::array<uint32_t, 6>& vertices,  T outWeights[6]) {
    // 1. Explicitly construct A (4x6 matrix)
    T A[4][6];
    for (int i = 0; i < 6; ++i) {
        A[0][i] = points[vertices[i]][0];
        A[1][i] = points[vertices[i]][1];
        A[2][i] = points[vertices[i]][2];
        A[3][i] = 1.0; // Partition of unity constraint
    }

    // 2. Compute M = A * A^T (Result is a 4x4 symmetric matrix)
    T M[16] = {0};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            T sum = 0.0;
            for (int k = 0; k < 6; ++k) {
                sum += A[r][k] * A[c][k]; // A[c][k] serves as A^T[k][c]
            }
            M[r * 4 + c] = sum;
        }
    }

    // 3. Compute M_inv = (A * A^T)^-1
    T M_inv[16];
    if (!invert4x4_fixed<T>(M, M_inv)) {
        return false; // Error: control points are degenerate (e.g., all flattened on a single plane)
    }

    // 4. Compute Vector B (4x1)
    T B[4] = { P[0], P[1], P[2], 1.0 };

    // 5. Compute intermediate vector: Y = M_inv * B (4x1)
    T Y[4] = {0};
    for (int r = 0; r < 4; ++r) {
        Y[r] = M_inv[r * 4 + 0] * B[0] +
               M_inv[r * 4 + 1] * B[1] +
               M_inv[r * 4 + 2] * B[2] +
               M_inv[r * 4 + 3] * B[3];
    }

    // 6. Compute final weights: weights = A^T * Y (6x1)
    for (int i = 0; i < 6; ++i) {
        outWeights[i] = A[0][i] * Y[0] +
                        A[1][i] * Y[1] +
                        A[2][i] * Y[2] +
                        A[3][i] * Y[3];
    }

    return true;
}

// Computes the optimal reconstruction weights for point P using 6 3D points
static std::array<float, 6> calculate3DWeights(const pxr::GfVec3f& P, const std::array<pxr::GfVec3f, 6>& points) {
    size_t N = points.size(); // Designed for N=6
    
    // We set up a system of linear equations using Lagrange Multipliers
    // To minimize sum(w_i^2 * dist_i) subject to sum(w_i) = 1 and sum(w_i * v_i) = P
    // For simplicity and high precision, we solve the local coordinate matrix.
    
    // Rows 0,1,2: Coordinate reconstruction constraints (X, Y, Z)
    // Row 3: Sum of weights equals 1 constraint
    // Rows 4,5: Regularization/minimization parameters for the remaining degrees of freedom
    
    int matrixSize = N + 4; // 6 points + 4 constraints (X, Y, Z, and Sum=1)
    std::vector<std::vector<float>> A(matrixSize, std::vector<float>(matrixSize, 0.0));
    std::vector<float> B(matrixSize, 0.0);

    // Populate the objective function (minimize distance-based energy)
    for (size_t i = 0; i < N; ++i) {
        float distSq = std::pow(points[i][0] - P[0], 2) + 
                        std::pow(points[i][1]- P[1], 2) + 
                        std::pow(points[i][2]- P[2], 2);
        
        // Prevent division by zero if P is exactly on a point
        if (distSq < 1e-9) {
            std::array<float, 6> exactWeights;
            std::fill(exactWeights.begin(), exactWeights.end(), 0.0f);
            exactWeights[i] = 1.0f;
            return exactWeights;
        }
        
        A[i][i] = 2.0f * distSq; // Weight penalization matrix
    }

    // Add Constraints to the linear system
    for (size_t i = 0; i < N; ++i) {
        A[i][N]     = points[i][0]; // X constraint row
        A[N][i]     = points[i][0];
        
        A[i][N + 1] = points[i][1]; // Y constraint row
        A[N + 1][i] = points[i][1];
        
        A[i][N + 2] = points[i][2]; // Z constraint row
        A[N + 2][i] = points[i][2];
        
        A[i][N + 3] = 1.0f;         // Sum = 1 constraint row
        A[N + 3][i] = 1.0f;
    }

    // Set target vector B
    B[N]     = P[0];
    B[N + 1] = P[1];
    B[N + 2] = P[2];
    B[N + 3] = 1.0f;

    // Solve Ax = B using Gaussian Elimination with partial pivoting
    for (int i = 0; i < matrixSize; ++i) {
        int maxRow = i;
        for (int k = i + 1; k < matrixSize; ++k) {
            if (std::abs(A[k][i]) > std::abs(A[maxRow][i])) {
                maxRow = k;
            }
        }
        std::swap(A[i], A[maxRow]);
        std::swap(B[i], B[maxRow]);

        for (int k = i + 1; k < matrixSize; ++k) {
            float factor = A[k][i] / A[i][i];
            for (int j = i; j < matrixSize; ++j) {
                A[k][j] -= factor * A[i][j];
            }
            B[k] -= factor * B[i];
        }
    }

    // Back substitution
    std::vector<float> X(matrixSize, 0.0f);
    for (int i = matrixSize - 1; i >= 0; --i) {
        X[i] = B[i];
        for (int j = i + 1; j < matrixSize; ++j) {
            X[i] -= A[i][j] * X[j];
        }
        X[i] /= A[i][i];
    }

    // Extract the first N elements as our weights
    std::array<float, 6> weights;
    for (size_t i = 0; i < N; ++i) {
        weights[i] = X[i];
    }

    return weights;
}

bool GuideCurvesDeformer::buildDeformerDataLHSMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	// Check we have guide indices (clump ids) and curves count matches.
	const size_t curves_count = mpCurvesContainer->getCurvesCount();
	assert(curves_count <= mGuideIndices.AsConst().size());


	const GuideCurvesContainer* pGuideCurvesContainer = mpGuideCurvesContainer.get();
	if(!pGuideCurvesContainer) return false;


	const size_t guide_vertex_count = pGuideCurvesContainer->getTotalVertexCount();

	// Hair curves binding part -----
	const auto& guides_rest_points = mpGuideCurvesContainer->getRestCurvePoints();
	const auto& guides_curve_vertex_counts = mpGuideCurvesContainer->getCurveVertexCounts();
	const PxrCurvesContainer* pCurvesContainer = mpCurvesContainer.get();
	const USDCurveKDTree guides_kdtree(pGuideCurvesContainer);

	size_t bound_curves = 0;
	size_t bound_points = 0;

	// preallocate 
	auto& pointBinds = mpGuideCurvesDeformerData->lhsPointBinds();
	pointBinds.resize(pCurvesContainer->getTotalVertexCount()); 

	// pre compute per vertex distances
	std::vector<float> dist_from_root(mpCurvesContainer->getTotalVertexCount());
	for(uint32_t curve_index = 0; curve_index < mpCurvesContainer->getCurvesCount(); ++curve_index) {
		uint32_t curve_vertex_offset = pCurvesContainer->getCurveVertexOffset(curve_index);
		dist_from_root[curve_vertex_offset] = 0.f;
		for(uint32_t i = 1; i < mpCurvesContainer->getCurveVertexCount(curve_index); ++i) {
			dist_from_root[curve_vertex_offset + i] = dist_from_root[curve_vertex_offset + i - 1] + 
				(pCurvesContainer->getCurveVector(curve_index, i - 1) - pCurvesContainer->getCurveVector(curve_index, i)).GetLength();
		}
	}

	for(uint32_t curve_index = 0; curve_index < mpCurvesContainer->getCurvesCount(); ++curve_index) {

		PxrCurvesContainer::CurveDataConstPtr curve_data_ptr = pCurvesContainer->getCurveDataPtr(curve_index);
		const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);

		const pxr::GfVec3f& curve_root_pt = pCurvesContainer->getCurveRootPoint(curve_index);
		const uint32_t curve_vertex_offset = pCurvesContainer->getCurveVertexOffset(curve_index);

		assert(curve_index < mGuideIndices.AsConst().size());
		const auto guide_id = mGuideIndices.AsConst()[curve_index];

		//DLOG_DBG << "Guide 0 ID " << guide_id;
		//DLOG_DBG << "Guide 0 length " << mpGuideCurvesContainer->getRestCurveLength(guide_id);

		const auto root_points = guides_kdtree.findKNearestCurveRoots(curve_root_pt, 2, guide_id /* ignore guide_id itself */);

		bool has_doubled_guide_id = false;
		for(const auto& root_point: root_points) {
			assert(root_point.first.positionType == USDCurveKDTree::CurvePositionType::Root);
			if(root_point.first.curveIndex == guide_id) has_doubled_guide_id = true;
		}

		if(has_doubled_guide_id || root_points.size() != 2) {
			DLOG_DBG << "!!!";
			continue;
		}

		//DLOG_DBG << "Guide 1 ID " << root_points[0].first.curveIndex;
		//DLOG_DBG << "Guide 2 ID " << root_points[1].first.curveIndex;

		// Bind rest of curves points
		for(uint32_t i = 0; i < curve_vertices_count; ++i) {
			float curr_dist_from_root = dist_from_root[curve_vertex_offset + i];

			//DLOG_DBG << "Curr dist " << curr_dist_from_root;

			const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 


			std::array<uint32_t, 6> gv; // 6 neightbour guides vertices

			pGuideCurvesContainer->getVeticesAtLength(guide_id, curr_dist_from_root, gv[0], gv[1]);
			pGuideCurvesContainer->getVeticesAtLength(root_points[0].first.curveIndex, curr_dist_from_root, gv[2], gv[3]);
			pGuideCurvesContainer->getVeticesAtLength(root_points[1].first.curveIndex, curr_dist_from_root, gv[4], gv[5]);

			auto& bind = pointBinds[curve_vertex_offset + i];

			// another test

			bind.curveIndices[0] = guide_id;
			bind.curveIndices[1] = root_points[0].first.curveIndex;
			bind.curveIndices[2] = root_points[1].first.curveIndex;

			std::array<pxr::GfVec3f, 6> pts;
			pts[0] = guides_rest_points[gv[0]];
			pts[1] = guides_rest_points[gv[1]];
			pts[2] = guides_rest_points[gv[2]];
			pts[3] = guides_rest_points[gv[3]];
			pts[4] = guides_rest_points[gv[4]];
			pts[5] = guides_rest_points[gv[5]];


			std::array<float, 6> weights = calculate3DWeights(curr_pt, pts);
			bind.w[0] = weights[0];
			bind.w[1] = weights[1];
			bind.w[2] = weights[2];
			bind.w[3] = weights[3];
			bind.w[4] = weights[4];
			bind.w[5] = weights[5];

			bind.v[0] = gv[0] - pGuideCurvesContainer->getCurveVertexOffset(guide_id);
			bind.v[1] = gv[1] - pGuideCurvesContainer->getCurveVertexOffset(guide_id);

			bind.v[2] = gv[2] - pGuideCurvesContainer->getCurveVertexOffset(root_points[0].first.curveIndex);
			bind.v[3] = gv[3] - pGuideCurvesContainer->getCurveVertexOffset(root_points[0].first.curveIndex);

			bind.v[4] = gv[4] - pGuideCurvesContainer->getCurveVertexOffset(root_points[1].first.curveIndex);
			bind.v[5] = gv[5] - pGuideCurvesContainer->getCurveVertexOffset(root_points[1].first.curveIndex);
       	}

       	bound_curves++;
	}


	DLOG_DBG << "LHS stats";
	DLOG_DBG << "Total curves count: " << mpCurvesContainer->getCurvesCount();
	DLOG_DBG << "Curves bound: " << bound_curves;
	DLOG_DBG << "Total curves points: " << mpCurvesContainer->getTotalVertexCount();
	DLOG_DBG << "Points bound: " << bound_points;

	return true;
}

static inline pxr::GfVec2f calculateInverseDistanceWeights(
    const pxr::GfVec3f& P,
    const pxr::GfVec3f& L1_A, const pxr::GfVec3f& L1_B,
    const pxr::GfVec3f& L2_A, const pxr::GfVec3f& L2_B) {

	// Lambda to compute squared perpendicular distance from P to an infinite line (A-B)
    auto distanceSqToLine = [&P](const pxr::GfVec3f& A, const pxr::GfVec3f& B) -> float {
        pxr::GfVec3f lineDir = B - A;
        float lineLenSq = pxr::GfDot(lineDir, lineDir);
        
        // Handle degenerate line (A and B are the same point)
        if (lineLenSq < 1e-7f) {
            return pxr::GfDot(P - A, P - A);
        }

        pxr::GfVec3f ap = P - A;
        pxr::GfVec3f crossProd = pxr::GfCross(ap, lineDir);
        return pxr::GfDot(crossProd, crossProd) / lineLenSq;
    };

    // 1. Calculate squared distances using the lambda
    float d2_1 = distanceSqToLine(L1_A, L1_B);
    float d2_2 = distanceSqToLine(L2_A, L2_B);

    // Epsilon to safeguard against division by zero if P sits directly on a line
    const float eps = 1e-6f; 

    // 2. Compute inverse squared distances (closer line = stronger weight)
    float w1 = 1.0f / (d2_1 + eps);
    float w2 = 1.0f / (d2_2 + eps);

    // 3. Normalize the weights so they sum to 1.0
    float sum = w1 + w2;
    return pxr::GfVec2f(w1 / sum, w2 / sum);
}

static inline pxr::GfVec3f calculateInverseDistanceWeights(
    const pxr::GfVec3f& P,
    const pxr::GfVec3f& L1_A, const pxr::GfVec3f& L1_B,
    const pxr::GfVec3f& L2_A, const pxr::GfVec3f& L2_B,
    const pxr::GfVec3f& L3_A, const pxr::GfVec3f& L3_B) {

	// Lambda to compute squared perpendicular distance from P to an infinite line (A-B)
    auto distanceSqToLine = [&P](const pxr::GfVec3f& A, const pxr::GfVec3f& B) -> float {
        pxr::GfVec3f lineDir = B - A;
        float lineLenSq = pxr::GfDot(lineDir, lineDir);
        
        // Handle degenerate line (A and B are the same point)
        if (lineLenSq < 1e-7f) {
            return pxr::GfDot(P - A, P - A);
        }

        pxr::GfVec3f ap = P - A;
        pxr::GfVec3f crossProd = pxr::GfCross(ap, lineDir);
        return pxr::GfDot(crossProd, crossProd) / lineLenSq;
    };

    // 1. Calculate squared distances using the lambda
    float d2_1 = distanceSqToLine(L1_A, L1_B);
    float d2_2 = distanceSqToLine(L2_A, L2_B);
    float d2_3 = distanceSqToLine(L3_A, L3_B);

    // Epsilon to safeguard against division by zero if P sits directly on a line
    const float eps = 1e-6f; 

    // 2. Compute inverse squared distances (closer line = stronger weight)
    float w1 = 1.0f / (d2_1 + eps);
    float w2 = 1.0f / (d2_2 + eps);
    float w3 = 1.0f / (d2_3 + eps);

    // 3. Normalize the weights so they sum to 1.0
    float sum = w1 + w2 + w3;
    return pxr::GfVec3f(w1 / sum, w2 / sum, w3 / sum);
}

bool GuideCurvesDeformer::buildDeformerDataBlendNTBMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	bool skin_prim_data_created = true;
	if(!buildSkinPrimData(multi_threaded, rest_time_code, skin_prim_data_created)) {
		mpGuideCurvesDeformerData->skinPrimIndices().clear();
		mpGuideCurvesDeformerData->setSkinPrimPath("");
		DLOG_ERR << "Error building skin prim geometry data !";
		return false;
	}	

	if(!mDeformerGeoPrimHandle.fetchAttributeValues<int>(mGuidesSkinPrimAttrName, mpGuideCurvesDeformerData->skinPrimIndices(), rest_time_code)) {
		DLOG_ERR << "Error getting skin prim indices !";
		return false;
	}

	if(!buildGuideOrigins(multi_threaded)) {
		DLOG_ERR << "NTB frames calulation without skin geometry primitive indices is NOT supported yet !";
		return false;
	}

	const GuideCurvesContainer* pGuideCurvesContainer = mpGuideCurvesContainer.get();
	if(!pGuideCurvesContainer) return false;

	const size_t curves_count = mpCurvesContainer->getCurvesCount();
	assert(curves_count == mGuideIndices.size());

	// pre allocate
	auto& pointBinds = mpGuideCurvesDeformerData->blendNTBPointBinds();
	const PxrCurvesContainer* pCurvesContainer = mpCurvesContainer.get();
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	std::vector<NTBFrame> rest_guide_frames(pGuideCurvesContainer->getRestCurvePoints().size());

	static const bool build_live = false;
	if(!buildNTBFrames(rest_guide_frames, multi_threaded, build_live)) {
		return false;
	}

	// inverse frames
	std::vector<pxr::GfMatrix3f> matrices(rest_guide_frames.size());
	for(size_t i = 0; i < rest_guide_frames.size(); ++i) {
		matrices[i] = rest_guide_frames[i].getMatrix3f().GetInverse();
	}

	const size_t guide_curves_count = pGuideCurvesContainer->getCurvesCount();

	std::vector<std::mutex> kdtrees_mutexes(guide_curves_count);  // protects kdree initialisation
	std::vector<std::unique_ptr<neighbour_search::KDTree<float, 3>>> kdtrees(guide_curves_count);
	const auto& guides_rest_points = pGuideCurvesContainer->getRestCurvePoints();

	const USDCurveKDTree guides_kdtree(pGuideCurvesContainer);

	// pre compute per vertex distances
	std::vector<float> dist_from_root(mpCurvesContainer->getTotalVertexCount());

	for(uint32_t curve_index = 0; curve_index < mpCurvesContainer->getCurvesCount(); ++curve_index) {
		uint32_t curve_vertex_offset = pCurvesContainer->getCurveVertexOffset(curve_index);
		dist_from_root[curve_vertex_offset] = 0.f;
	
		for(uint32_t i = 1; i < mpCurvesContainer->getCurveVertexCount(curve_index); ++i) {
			dist_from_root[curve_vertex_offset + i] = dist_from_root[curve_vertex_offset + i - 1] + 
				(pCurvesContainer->getCurveVector(curve_index, i - 1) - pCurvesContainer->getCurveVector(curve_index, i)).GetLength();
		}
	}

	auto func = [&](const std::size_t start, const std::size_t end) {

		for(size_t curve_index = start; curve_index < end; ++curve_index) {
			const auto guide_id = mGuideIndices.AsConst()[curve_index];

			PxrCurvesContainer::CurveDataConstPtr curve_data_ptr = pCurvesContainer->getCurveDataPtr(curve_index);
			const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);

			const pxr::GfVec3f& curve_root_pt = pCurvesContainer->getCurveRootPoint(curve_index);
			const uint32_t curve_vertex_offset = pCurvesContainer->getCurveVertexOffset(curve_index);

			const auto root_points = guides_kdtree.findKNearestCurveRoots(curve_root_pt, 2, guide_id /* ignore guide_id itself */);

			for(uint32_t i = 0; i < curve_vertices_count; ++i) {
				float curr_dist_from_root = dist_from_root[curve_vertex_offset + i];

				const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 
				auto& bind = pointBinds[curve_vertex_offset + i];

				std::array<uint32_t, 6> gv; // 6 neightbour guides vertices

				pGuideCurvesContainer->getVeticesAtLength(guide_id, curr_dist_from_root, gv[0], gv[1]);
				bind.guide_id[0] = guide_id;
				bind.v[0] = gv[0] - pGuideCurvesContainer->getCurveVertexOffset(guide_id);

				const uint8_t neighbour_guides_count = static_cast<uint8_t>(root_points.size());

				switch (neighbour_guides_count) {
					case 0:
						{
						bind.w[0] = 1.f; bind.w[1] = 0.f; bind.w[2] = 0.f;
						bind.coords[0] = matrices[gv[0]] * (curr_pt - guides_rest_points[gv[0]]);
						}
						break;
					case 1:
						{
						pGuideCurvesContainer->getVeticesAtLength(root_points[0].first.curveIndex, curr_dist_from_root, gv[2], gv[3]);
						bind.guide_id[1] = root_points[0].first.curveIndex;
						bind.v[1] = gv[2] - pGuideCurvesContainer->getCurveVertexOffset(root_points[0].first.curveIndex);

						pxr::GfVec2f weights = calculateInverseDistanceWeights(curr_pt, guides_rest_points[gv[0]], guides_rest_points[gv[1]], guides_rest_points[gv[2]], guides_rest_points[gv[3]]);

						bind.w[0] = weights[0];
						bind.w[1] = weights[1];
						bind.w[2] = 0.f;

						bind.coords[0] = matrices[gv[0]] * (curr_pt - guides_rest_points[gv[0]]);
						bind.coords[1] = matrices[gv[2]] * (curr_pt - guides_rest_points[gv[2]]);
						}
						break;
					case 2:
						{
						pGuideCurvesContainer->getVeticesAtLength(root_points[0].first.curveIndex, curr_dist_from_root, gv[2], gv[3]);
						bind.guide_id[1] = root_points[0].first.curveIndex;
						bind.v[1] = gv[2] - pGuideCurvesContainer->getCurveVertexOffset(root_points[0].first.curveIndex);
						pGuideCurvesContainer->getVeticesAtLength(root_points[1].first.curveIndex, curr_dist_from_root, gv[4], gv[5]);
						bind.guide_id[2] = root_points[1].first.curveIndex;
						bind.v[2] = gv[4] - pGuideCurvesContainer->getCurveVertexOffset(root_points[1].first.curveIndex);

						pxr::GfVec3f weights = calculateInverseDistanceWeights(curr_pt, guides_rest_points[gv[0]], guides_rest_points[gv[1]], guides_rest_points[gv[2]], guides_rest_points[gv[3]], guides_rest_points[gv[4]], guides_rest_points[gv[5]]);

						bind.w[0] = weights[0];
						bind.w[1] = weights[1];
						bind.w[2] = weights[2];

						bind.coords[0] = matrices[gv[0]] * (curr_pt - guides_rest_points[gv[0]]);
						bind.coords[1] = matrices[gv[2]] * (curr_pt - guides_rest_points[gv[2]]);
						bind.coords[2] = matrices[gv[4]] * (curr_pt - guides_rest_points[gv[4]]);
						}
						break;
					default:
						break;
				}
			}
		}
	};

	DLOG_INF << "Binding curves to guides using Blended NTB mode.";

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, curves_count, func);
		blocks.wait();
	} else {
		func(0u, curves_count);
	}

	return true;

}


bool GuideCurvesDeformer::buildDeformerDataNTBMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	bool skin_prim_data_created = true;
	if(!buildSkinPrimData(multi_threaded, rest_time_code, skin_prim_data_created)) {
		mpGuideCurvesDeformerData->skinPrimIndices().clear();
		mpGuideCurvesDeformerData->setSkinPrimPath("");
		DLOG_ERR << "Error building skin prim geometry data !";
		return false;
	}	

	if(!mDeformerGeoPrimHandle.fetchAttributeValues<int>(mGuidesSkinPrimAttrName, mpGuideCurvesDeformerData->skinPrimIndices(), rest_time_code)) {
		DLOG_ERR << "Error getting skin prim indices !";
		return false;
	}

	if(!buildGuideOrigins(multi_threaded)) {
		DLOG_ERR << "NTB frames calulation without skin geometry primitive indices is NOT supported yet !";
		return false;
	}

	const size_t curves_count = mpCurvesContainer->getCurvesCount();
	assert(curves_count == mGuideIndices.size());

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

	DLOG_INF << "Binding curves to guides using NTB mode.";

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, curves_count, func);
		blocks.wait();
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

			const pxr::GfVec3f& guide_pt = mpGuideCurvesContainer->getCurveRestPoint(guide_id, segment_id);
			
			bind.encodeID_modeANGLE(guide_id, segment_id);
        	bind.setData(curr_pt - guide_pt);
        }

    };

    DLOG_INF << "Binding curves to guides using ANGLE mode.";

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

bool GuideCurvesDeformer::buildSkinPrimData(bool multi_threaded, pxr::UsdTimeCode rest_time_code, bool& created) {
	DeformerDataCache& dataCache = DeformerDataCache::getInstance();

	if(!mGuidesSkinGeoPrimHandle) {
		return false;
	}

	if(!mpSkinMeshContainer) {
		mpSkinMeshContainer = MeshContainer::create(mGuidesSkinGeoPrimHandle, mGuidesSkinPrimRestAttrName, getRestTimeCode());
		if(!mpSkinMeshContainer) {
			DLOG_FTL << "Error creating mesh container for skin prim " << mGuidesSkinGeoPrimHandle << " !!!";
			return false;
		}
	} else {
		if(!mpSkinMeshContainer->init(mGuidesSkinGeoPrimHandle, mGuidesSkinPrimRestAttrName, getRestTimeCode())) {
			DLOG_FTL << "Error initializing mesh container for skin prim " << mGuidesSkinGeoPrimHandle << " !!!";
			return false;
		}
	}

	if(!mDirty && mpSkinAdjacencyData && mpSkinPhantomTrimeshData && 
		mpSkinAdjacencyData->isValid() && 
		mpSkinPhantomTrimeshData->isValid()) {
		return true;
	}

	bool skin_adjacency_data_created = true;
	if(!mpSkinAdjacencyData) {
		mpSkinAdjacencyData = dataCache.getOrCreateData<SerializableUsdGeomMeshFaceAdjacency>(this, mGuidesSkinGeoPrimHandle, rest_time_code, skin_adjacency_data_created);
		assert(mpSkinAdjacencyData);
	}

	if(skin_adjacency_data_created || !getReadJsonDataState() || !mGuidesSkinGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpSkinAdjacencyData.get())) {
		if(!mpSkinAdjacencyData->buildInPlace(mGuidesSkinGeoPrimHandle)) {
			DLOG_ERR << "Error building guides skin adjacency data!";
			return false;
		}
	}

	bool skin_trimesh_data_created = true;
	if(!mpSkinPhantomTrimeshData) {
		mpSkinPhantomTrimeshData = dataCache.getOrCreateData<SerializablePhantomTrimesh>(this, {&mDeformerGeoPrimHandle, &mCurvesGeoPrimHandle, &mGuidesSkinGeoPrimHandle}, rest_time_code, skin_trimesh_data_created);
		assert(mpSkinPhantomTrimeshData);
	}

	if(skin_trimesh_data_created || !getReadJsonDataState() || !mGuidesSkinGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpSkinPhantomTrimeshData.get())) {
		if(!mpSkinPhantomTrimeshData->buildInPlace(mGuidesSkinGeoPrimHandle, mGuidesSkinPrimRestAttrName)) {
			DLOG_ERR << "Error building guides skin trimesh data!";
			return false;
		}
	}
	
	created = skin_trimesh_data_created || skin_adjacency_data_created;

	return true;
}

bool GuideCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(mpGuidesPhantomTrimeshData){
		if(!mDeformerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpGuidesPhantomTrimeshData.get())) {
			DLOG_ERR << "Error writing " << mpGuidesPhantomTrimeshData->typeName() << " deformer data to json !";
			return false;
		} else {
			DLOG_DBG << "Deformer data " << mpGuidesPhantomTrimeshData->jsonDataKey() << " written.";
		}
	}

	if(mpGuideCurvesDeformerData) {
		if(!mCurvesGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpGuideCurvesDeformerData.get())) {
			DLOG_ERR << "Error writing " << mpGuideCurvesDeformerData->typeName() << " deformer data to json !";
			return false;
		} else {
			DLOG_DBG << "Deformer data " << mpGuideCurvesDeformerData->jsonDataKey() << " written.";
		}
	}

	if(mGuidesSkinGeoPrimHandle) {
		if(mpSkinPhantomTrimeshData && mpSkinPhantomTrimeshData->isValid()) {
			if(!mGuidesSkinGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpSkinPhantomTrimeshData.get())) {
				DLOG_ERR << "Error writing " << mpSkinPhantomTrimeshData->typeName() << " curves data to json !";
				return false;
			} else {
				DLOG_DBG << "Deformer data " << mpSkinPhantomTrimeshData->jsonDataKey() << " written.";
			}
		}

		if(mpSkinAdjacencyData && mpSkinAdjacencyData->isValid()) {
			if(!mGuidesSkinGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpSkinAdjacencyData.get())) {
				DLOG_ERR << "Error writing " << mpSkinAdjacencyData->typeName() << " deformer data to json !";
				return false;
			} else {
				DLOG_DBG << "Deformer data " << mpSkinAdjacencyData->jsonDataKey() << " written.";
			}
		}
	}

	return true;
}

void GuideCurvesDeformer::drawDebugGeometry(pxr::UsdTimeCode time_code, const PointsList* pDeformedPoints) {
	assert(mpGuideCurvesDeformerData);
	assert(mpGuideCurvesContainer);
	assert(mpCurvesContainer);

	if(!mpDebugGeo) {
		mpDebugGeo = DebugGeo::create(getName());
	} else {
		mpDebugGeo->clear();
	}

	const auto& guides_live_points = mpGuideCurvesContainer->getLiveCurvePoints();

	switch(mpGuideCurvesDeformerData->getBindMode()) {
		case GuideCurvesDeformerData::BindMode::BLEND:
		{
			std::vector<NTBFrame> live_guide_frames(guides_live_points.size());
			
			if(!buildNTBFrames(live_guide_frames, true /* multi threaded */, true /* live */)) {
				DLOG_ERR << "drawDebugGeometry(time_code) NTB frames building failed.";
				return;
			}

			// vector from curve point to frame
			const auto& pointBinds = mpGuideCurvesDeformerData->getBlendNTBPointBinds();
			
			const auto& deformed_points = pDeformedPoints->getVtArray();

			for(size_t i = 0; i < pointBinds.size(); ++i) {
				const auto& bind = pointBinds[i];
				if(!bind.isValid()) continue;

				uint32_t frame_id_0 = mpGuideCurvesContainer->getCurveVertexOffset(bind.guide_id[0]) + bind.v[0];
				uint32_t frame_id_1 = mpGuideCurvesContainer->getCurveVertexOffset(bind.guide_id[1]) + bind.v[1];
				uint32_t frame_id_2 = mpGuideCurvesContainer->getCurveVertexOffset(bind.guide_id[2]) + bind.v[2];

				const pxr::GfVec3f& pt = deformed_points[i];

				DebugGeo::Line l0(guides_live_points[frame_id_0], pt);
				l0.setColor({1.0f, 1.0f, 1.0f});
				l0.setWidth(0.0025f);
				mpDebugGeo->addLine(l0);

				DebugGeo::Line l1(guides_live_points[frame_id_1], pt);
				l1.setColor({1.0f, 1.0f, 1.0f});
				l1.setWidth(0.0025f);
				mpDebugGeo->addLine(l1);

				DebugGeo::Line l2(guides_live_points[frame_id_2], pt);
				l2.setColor({1.0f, 1.0f, 1.0f});
				l2.setWidth(0.0025f);
				mpDebugGeo->addLine(l2);
			}
			
			mpDebugGeo->build("/debugBlebdedNTBFrames", mCurvesGeoPrimHandle.getStage());
		}
			break;
		case GuideCurvesDeformerData::BindMode::NTB:
		{
			std::vector<NTBFrame> live_guide_frames(guides_live_points.size());
			
			if(!buildNTBFrames(live_guide_frames, true /* multi threaded */, true /* live */)) {
				DLOG_ERR << "drawDebugGeometry(time_code) NTB frames building failed.";
				return;
			}

			// ntb frames
			for(size_t guide_id = 0; guide_id < mpGuideCurvesContainer->getCurvesCount(); ++guide_id) {
				const size_t guide_vertex_offset = mpGuideCurvesContainer->getCurveVertexOffset(guide_id);
				for(size_t i = 0; i < 1 /*mpGuideCurvesContainer->getCurveVertexCount(guide_id)*/; ++i) {
					size_t vtx = guide_vertex_offset + i; 
					const auto& frame = live_guide_frames[vtx];
					const auto& p = guides_live_points[vtx];

					DebugGeo::Line lN(p, p + frame.n * mDebugGeometryMult);
					lN.setColor({1.0f, 0.0f, 0.0f});
					lN.setWidth(0.05f);
					DebugGeo::Line lT(p, p + frame.t);
					lT.setColor({0.0f, 1.0f, 0.0f});
					lT.setWidth(0.05f);
					DebugGeo::Line lB(p, p + frame.b * mDebugGeometryMult);
					lB.setColor({0.0f, 0.0f, 1.0f});
					lB.setWidth(0.05f);

					mpDebugGeo->addLine(lN);
					mpDebugGeo->addLine(lT);
					mpDebugGeo->addLine(lB);
				}
			}

			// vector from curve point to frame
			if(1==2){
				const auto& pointBinds = mpGuideCurvesDeformerData->getPointBinds();
				
				for(size_t i = 0; i < pointBinds.size(); ++i) {
					const auto& bind = pointBinds[i];
					if(bind.encoded_id == PointBindData::kInvalid) continue;

					uint32_t frame_id;
					bind.decodeID_modeNTB(frame_id);
					assert(frame_id < guides_live_points.size());
					assert(frame_id < live_guide_frames.size());
					const std::array<float, 3>& ntbCoord = bind.getData();

					//points[i] = guides_live_points[frame_id] + live_guide_frames[frame_id] * ntbCoord;
					DebugGeo::Line lV(guides_live_points[frame_id], guides_live_points[frame_id] + live_guide_frames[frame_id] * ntbCoord);
					lV.setColor({1.0f, 1.0f, 1.0f});
					lV.setWidth(0.05f);
					mpDebugGeo->addLine(lV);
				}
			}

			mpDebugGeo->build("/debugNTBFrames", mCurvesGeoPrimHandle.getStage());
		}
			break;
		case GuideCurvesDeformerData::BindMode::SPACE:
		{
			PhantomTrimesh* pPhantomTrimesh = mpGuidesPhantomTrimeshData->getTrimesh();
			assert(pPhantomTrimesh);

			const auto* pDeformerMeshContainer = mpDeformerMeshContainer.get();
			assert(pDeformerMeshContainer);

			DLOG_DBG << "DebugGeo tetrahedrons count " << pPhantomTrimesh->getTetrahedrons().size();

			if(!pPhantomTrimesh->hasTetrahedrons()) {
				DLOG_ERR << "No tetrahedrons!";
				return;
			}

			const std::vector<PhantomTrimesh::Tetrahedron>& tetrahedrons = pPhantomTrimesh->getTetrahedrons();
			TetrahedronKDTree<PhantomTrimesh::PxrIndexType> tetraKDTree(tetrahedrons, pDeformerMeshContainer->getRestPositions());

			const auto* pRootNode = tetraKDTree.getRootNode();
			if(!pRootNode) {
				DLOG_ERR << "No tetrahedrons KDTree root node!";
				return;
			}

			std::function<void(const TetrahedronKDTree<PhantomTrimesh::PxrIndexType>::KDNode*, float)> drawNodeBound = 
			[&](const TetrahedronKDTree<PhantomTrimesh::PxrIndexType>::KDNode* pNode, float k_c) {
				if (!pNode) return;

				DebugGeo::WireframeBox box(pNode->nodeBounds);
				box.setColor({1.0 * k_c, 0.0, 0.0});
				box.setWidth(0.05);
				mpDebugGeo->addWireBox(box);

				if (pNode->left)  drawNodeBound(pNode->left, k_c * 0.85);
				if (pNode->right) drawNodeBound(pNode->right, k_c * 0.85);
			};

			drawNodeBound(pRootNode, 1.0 /* color k */);

			// Tetras
			const auto& points = pDeformerMeshContainer->getRestPositions();
			for(const auto& tetra: tetrahedrons) {

				DebugGeo::WireTetra tet(points[tetra.indices[0]], points[tetra.indices[1]], points[tetra.indices[2]], points[tetra.indices[3]]);
				tet.setColor({0.0, 1.0, 0.0});
				tet.setWidth(0.05);
				mpDebugGeo->addWireTetra(tet);
			}

			mpDebugGeo->build("/debugKDtree", mCurvesGeoPrimHandle.getStage());
		}	
			break;
		default:
			break;
	}
}

void GuideCurvesDeformer::setBindRootsToSkinSurface(bool bind) {
	if( mBindRootsToSkinSurface == bind) return;
	mBindRootsToSkinSurface = bind;
	makeDirty();
}

void GuideCurvesDeformer::invalidateData(DeformerDataCache& cache) {
	//if(mpGuideCurvesDeformerData && mpGuideCurvesDeformerData->isValid()) cache.invalidate<GuideCurvesDeformerData>({&mDeformerGeoPrimHandle, &mCurvesGeoPrimHandle, &mGuidesSkinGeoPrimHandle});
	//if(mpSkinPhantomTrimeshData && mpSkinPhantomTrimeshData->isValid()) cache.invalidate<SerializablePhantomTrimesh>({&mDeformerGeoPrimHandle, &mCurvesGeoPrimHandle, &mGuidesSkinGeoPrimHandle});
	//if(mpGuidesPhantomTrimeshData && mpGuidesPhantomTrimeshData->isValid()) cache.invalidate<SerializablePhantomTrimesh>({&mDeformerGeoPrimHandle, &mCurvesGeoPrimHandle, &mGuidesSkinGeoPrimHandle});

	cache.invalidate(mpGuideCurvesDeformerData);

	cache.invalidate(mpSkinAdjacencyData);
	cache.invalidate(mpSkinPhantomTrimeshData);
	cache.invalidate(mpGuidesPhantomTrimeshData);
}

void GuideCurvesDeformer::setBindMode(GuideCurvesDeformer::BindMode mode) {
	if(mBindMode == mode) return;
	mBindMode = mode;
	makeDirty();
}

GuideCurvesDeformer::BindMode GuideCurvesDeformer::getBindMode() const {
	return mBindMode;
}


GuideCurvesDeformer::~GuideCurvesDeformer() {
	PROFILE_PRINT();
}


} // namespace Piston