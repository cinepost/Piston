#include "simple_profiler.h"
#include "wrap_curves_deformer.h"

#include "common.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <pxr/base/gf/matrix4f.h>

#include <atomic>
#include <random>
#include <algorithm>
#include <optional>

BS::synced_stream sync_out;

namespace Piston {

static constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
static constexpr float kMaxFloat = std::numeric_limits<float>::max();

WrapCurvesDeformer::WrapCurvesDeformer(): BaseCurvesDeformer() {
	dbg_printf("WrapCurvesDeformer::WrapCurvesDeformer()\n");

	mpWrapCurvesDeformerData = std::make_unique<WrapCurvesDeformerData>();
	mpWrapCurvesDeformerData->setBindMode(BindMode::SPACE);
}

WrapCurvesDeformer::SharedPtr WrapCurvesDeformer::create() {
	return SharedPtr(new WrapCurvesDeformer());
}

const std::string& WrapCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "WrapCurvesDeformer";
	return kFastDeformerString;
}

bool WrapCurvesDeformer::deformImpl(pxr::UsdTimeCode time_code) {
	return __deform__(false, time_code);
}

bool WrapCurvesDeformer::deformMtImpl(pxr::UsdTimeCode time_code) {
	return __deform__(true, time_code);
}

bool WrapCurvesDeformer::__deform__(bool multi_threaded, pxr::UsdTimeCode time_code) {
	PROFILE(multi_threaded ? "WrapCurvesDeformer::deformMtImpl" : "WrapCurvesDeformer::deformImpl");
	
	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();

	if(!pPhantomTrimesh || !pPhantomTrimesh->isValid() || !pPhantomTrimesh->update(mMeshGeoPrimHandle, time_code)) {
		return false;
	}

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	
	if(!mpCurvesContainer || mpCurvesContainer->empty()) {
		return false;
	}

	std::vector<pxr::GfVec3f>& points = mpCurvesContainer->getPointsCache();

	assert(points.size() == mpWrapCurvesDeformerData->getPointBinds().size());
	assert(mpAdjacencyData);

	buildVertexNormals(mpAdjacencyData->getAdjacency(), pPhantomTrimesh, mLiveVertexNormals, true);

	bool result = false;
	switch(mpWrapCurvesDeformerData->getBindMode()) {
		case BindMode::SPACE:
			result = deformImpl_SpaceMode(multi_threaded, points, time_code);
			break;
		default:
			result = deformImpl_DistMode(multi_threaded, points, time_code);
			break;
	}

	if(!result) return false;

	if(!curves.GetPointsAttr().Set(mpCurvesContainer->getPointsCacheVtArray(), time_code)) {
		return false;
	}

	return true;
}

static inline bool saturate(bool a) {
	return a < 0.f ? 0.f : (a > 1.f ? 1.f : a);
}

bool WrapCurvesDeformer::deformImpl_SpaceMode(bool multi_threaded, std::vector<pxr::GfVec3f>& points, pxr::UsdTimeCode time_code) {
	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	if(!pPhantomTrimesh || !pPhantomTrimesh->isValid()) return false;

	const auto& pointBinds = mpWrapCurvesDeformerData->getPointBinds();

	auto func = [&](const std::size_t start, const std::size_t end) {
		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			if(bind.face_id == PointBindData::kInvalidFaceID) continue;

			const auto& face = pPhantomTrimesh->getFace(bind.face_id);
			
			pxr::GfVec3f interpolated_normal = pxr::GfGetNormalized(
				bind.u * mLiveVertexNormals[face.indices[1]] + bind.v * mLiveVertexNormals[face.indices[2]] + (1.f - bind.u - bind.v) * mLiveVertexNormals[face.indices[0]]
			);

			points[i] = pPhantomTrimesh->getInterpolatedLivePosition(bind.face_id, bind.u, bind.v) + (interpolated_normal * bind.dist);
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

bool WrapCurvesDeformer::deformImpl_DistMode(bool multi_threaded, std::vector<pxr::GfVec3f>& points, pxr::UsdTimeCode time_code) {
	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	if(!pPhantomTrimesh || !pPhantomTrimesh->isValid()) return false;

	const auto& pointBinds = mpWrapCurvesDeformerData->getPointBinds();

	auto func = [&](const std::size_t start, const std::size_t end) {	
		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			if(bind.face_id == PointBindData::kInvalidFaceID) continue;

			pxr::GfVec3f face_normal = pPhantomTrimesh->getFaceLiveNormal(bind.face_id);
			points[i] = pPhantomTrimesh->getInterpolatedLivePosition(bind.face_id, bind.u, bind.v) + (face_normal * bind.dist);
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

bool WrapCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(!mCurvesGeoPrimHandle.writeDataToBson(mpWrapCurvesDeformerData.get())) {
		std::cerr << "Error writing " << mpWrapCurvesDeformerData->typeName() << " deformer data to json !";	
		return false;
	}
	return true;
}

bool WrapCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code) {
	assert(mpAdjacencyData);

	// Get mesh adjacency
	const auto* pAdjacency = mpAdjacencyData->getAdjacency();
	assert(pAdjacency);

	if(!pAdjacency->isValid()) {
		std::cerr << "No mesh adjacency data !" << std::endl;
		return false;
	}

	assert(pAdjacency->getMaxFaceVertexCount() > 0);

	// Get phantom mesh
	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	if(!pPhantomTrimesh || !pPhantomTrimesh->isValid()) return false;

	// Clear deformer data
	mpWrapCurvesDeformerData->clear();

	if(!getReadJsonDataState() || !mCurvesGeoPrimHandle.getDataFromBson(mpWrapCurvesDeformerData.get())) {
		// Build deformer data in place if no json data present or not needed

		// First triangulate using simple "fan" triangulation
		const size_t src_mesh_face_count = pAdjacency->getFaceCount();

		for(size_t face_id = 0; face_id < src_mesh_face_count; ++face_id) {
			const uint32_t face_vertex_count = pAdjacency->getFaceVertexCount(face_id);
			
			if(face_vertex_count < 3 ) {
				std::cerr << "Source mesh polygon " << face_id << " is invalid !!!" << std::endl;
				continue;
			}
			
			const uint32_t face_vertex_offset = pAdjacency->getFaceVertexOffset(face_id);

			switch(face_vertex_count) {
				case 3:
					pPhantomTrimesh->getOrCreate(
						pAdjacency->getFaceVertex(face_id, 0), 
						pAdjacency->getFaceVertex(face_id, 1),
						pAdjacency->getFaceVertex(face_id, 2)
					);
					break;
				default:
					for(uint32_t ii = 1; ii < (face_vertex_count - 1); ++ii) {
						pPhantomTrimesh->getOrCreate(
							pAdjacency->getFaceVertex(face_id, 0), 
							pAdjacency->getFaceVertex(face_id, ii % face_vertex_count),
							pAdjacency->getFaceVertex(face_id, (ii + 1) % face_vertex_count)
						);
					}
					break;
			}
		}

		const size_t tri_face_count = pPhantomTrimesh->getFaceCount();

		dbg_printf("%zu source mesh faces triangulated to %zu triangles\n", src_mesh_face_count, tri_face_count);

		std::vector<pxr::GfVec3f> rest_vertex_normals;
		buildVertexNormals(pAdjacency, pPhantomTrimesh, rest_vertex_normals, false);
		mLiveVertexNormals.resize(rest_vertex_normals.size());

		// Bind curve points
		dbg_printf("Binding %zu curves (%zu total vertices).\n", mpCurvesContainer->getCurvesCount(), mpCurvesContainer->getTotalVertexCount());	
		dbg_printf("Using %s search method.\n", to_string(mpWrapCurvesDeformerData->getBindMode()).c_str());

		bool result = false;
		auto threads_timer = Timer();
		threads_timer.start();

		// Build bind data
		switch(mpWrapCurvesDeformerData->getBindMode()) {
			case BindMode::SPACE:
				result = buildDeformerData_SpaceMode(rest_vertex_normals, rest_time_code);
				break;
			default:
				result = buildDeformerData_DistMode(rest_vertex_normals, rest_time_code);
				break;
		} 

		threads_timer.stop();
		dbg_printf("%zu threads finished in %s\n", mPool.get_thread_count(), threads_timer.toString().c_str());
	
		mpWrapCurvesDeformerData->setPopulated(result);
	}

	return mpWrapCurvesDeformerData->isPopulated();
}

static neighbour_search::KDTree<float, 3> buildTrimeshCentroidsKDTree(const PhantomTrimesh* pTrimesh, bool threaded_kdtree_creation) {
	
	pxr::VtArray<pxr::GfVec3f> trimesh_centroids(pTrimesh->getFaceCount());
	for(uint32_t face_id = 0; face_id < pTrimesh->getFaceCount(); ++face_id) {
		trimesh_centroids[face_id] = pTrimesh->getFaceRestCentroid(face_id);
	}

	return neighbour_search::KDTree<float, 3>(trimesh_centroids, threaded_kdtree_creation);
}

bool WrapCurvesDeformer::buildDeformerData_DistMode(const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code) {
	PROFILE("WrapCurvesDeformer::buildDeformerData_DistMode");

	const size_t curves_count = mpCurvesContainer->getCurvesCount();

	auto& pointBinds = mpWrapCurvesDeformerData->mPointBinds;
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	const PhantomTrimesh* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	// Build kdtree
	neighbour_search::KDTree<float, 3> kdtree = buildTrimeshCentroidsKDTree(pPhantomTrimesh, true);

    auto func = [&](const std::size_t start, const std::size_t end) {
    	const std::optional<std::size_t> thread_index = BS::this_thread::get_index();
    	dbg_printf("Binding curves from %zu to %zu by thread id #%zu\n", start, end, *thread_index);

    	const auto& meshRestPositions = pPhantomTrimesh->getRestPositions();
    	const auto& faces = pPhantomTrimesh->getFaces();

    	for(size_t curve_idx = start; curve_idx < end; ++curve_idx) {
    		const PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_idx);
        	if(curve_data_ptr.first < 2) continue;

        	const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);

        	const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_idx);
        	const uint32_t curve_vertex_offset = mpCurvesContainer->getCurveVertexOffset(curve_idx);
        	
        	for(uint32_t i = 0; i < curve_vertices_count; ++i) {
        		auto& bind = pointBinds[curve_vertex_offset + i];
        		bind.face_id = PointBindData::kInvalidFaceID;
        		const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 

        		const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = kdtree.findNearestNeighbour(curr_pt);
        		const uint32_t face_id = nearest_pt.first;

				auto bindCurvePointToPrim = [&] (const uint32_t curve_vtx, const uint32_t face_id, PointBindData& bind) {
					const auto& face = faces[face_id];

					const pxr::GfVec3f& p0 = meshRestPositions[face.indices[0]];
					const pxr::GfVec3f& p1 = meshRestPositions[face.indices[1]];
					const pxr::GfVec3f& p2 = meshRestPositions[face.indices[2]];

					const auto& face_normal = face.getRestNormal();
					const Plane face_plane(meshRestPositions[face.indices[0]], face_normal);
					const float face_distance = distance(face_plane, curr_pt);

					const pxr::GfVec3f projected_pt = curr_pt - face_normal * face_distance; // project point on to face plane

					const pxr::GfVec3f v0 = p1 - p0, v1 = p2 - p0, v2 = projected_pt - p0;
					float d00 = pxr::GfDot(v0, v0);
					float d01 = pxr::GfDot(v0, v1);
					float d11 = pxr::GfDot(v1, v1);
					float d20 = pxr::GfDot(v2, v0);
					float d21 = pxr::GfDot(v2, v1);
					float denom = d00 * d11 - d01 * d01;

					bind.u = (d11 * d20 - d01 * d21) / denom;
					bind.v = (d00 * d21 - d01 * d20) / denom;

					const pxr::GfVec3f projected_point = pPhantomTrimesh->getInterpolatedRestPosition(face_id, bind.u, bind.v);
            		bind.dist = face_distance;
					bind.face_id = face_id;
				};

				bindCurvePointToPrim(i, face_id, bind);
        	}
    	}
    };

	mPool.detach_blocks(0, curves_count, func);
	mPool.wait();
	//func(0, curves_count);

    dbg_printf("WrapCurvesDeformer::buildDeformerData_DistMode() done.\n");

	return true;
}

bool WrapCurvesDeformer::buildDeformerData_SpaceMode(const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code) {
	PROFILE("WrapCurvesDeformer::buildDeformerData_SpaceMode");

	const size_t curves_count = mpCurvesContainer->getCurvesCount();
	std::atomic<size_t> bound_curves = 0;
	std::atomic<size_t> partially_bound_curves = 0;

	auto& pointBinds = mpWrapCurvesDeformerData->mPointBinds;
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	const PhantomTrimesh* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	// Build kdtree
	neighbour_search::KDTree<float, 3> kdtree = buildTrimeshCentroidsKDTree(pPhantomTrimesh, true);


    auto func = [&](const std::size_t start, const std::size_t end) {
    	const std::optional<std::size_t> thread_index = BS::this_thread::get_index();
    	dbg_printf("Binding curves from %zu to %zu by thread id #%zu\n", start, end, *thread_index);

    	const auto& meshRestPositions = pPhantomTrimesh->getRestPositions();
    	const auto& faces = pPhantomTrimesh->getFaces();
    	const uint32_t faces_count = faces.size();

    	std::vector<uint8_t> curve_flags(mpCurvesContainer->getCurvesCount());
    	std::fill(curve_flags.begin(), curve_flags.end(), 0);

    	std::vector<std::pair<uint32_t, float>> tmp_curve_distances; // pairs of <face_id, face_distance
    	tmp_curve_distances.reserve(128);

        for(std::size_t curve_idx = start; curve_idx < end; ++curve_idx) {

        	const PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_idx);
        	if(curve_data_ptr.first < 2) continue;

        	const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);
        	uint32_t bound_curve_vertices_count = 0;

        	const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_idx);
        	const uint32_t curve_vertex_offset = mpCurvesContainer->getCurveVertexOffset(curve_idx);
        	
        	for(uint32_t i = 0; i < curve_vertices_count; ++i) {

        		auto& bind = pointBinds[curve_vertex_offset + i];
        		bind.face_id = PointBindData::kInvalidFaceID;
        		pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 

        		for(uint32_t face_id = 0; face_id < faces_count; ++face_id) {
        			const auto& face = faces[face_id];
        			const auto& face_normal = face.getRestNormal();
        			const Plane face_plane(meshRestPositions[face.indices[0]], face_normal);

					pxr::GfVec3f p0 = meshRestPositions[face.indices[0]];
					pxr::GfVec3f p1 = meshRestPositions[face.indices[1]];
					pxr::GfVec3f p2 = meshRestPositions[face.indices[2]];

        			const float face_distance = distance(face_plane, curr_pt);

					//if(tmp_curve_distances[i].second)

        			const bool point_is_in_plane = abs(face_distance) <= kEpsilon;
        			const bool point_is_below_surface = face_distance < 0.f;

        			if(!point_is_in_plane) { 
						const pxr::GfVec3f point_to_plane_vector = face_normal * face_distance;

						p0 += rest_vertex_normals[face.indices[0]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[0]]);
						p1 += rest_vertex_normals[face.indices[1]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[1]]);
						p2 += rest_vertex_normals[face.indices[2]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[2]]);

        			}

					const pxr::GfVec3f v0 = p1 - p0, v1 = p2 - p0, v2 = curr_pt - p0;
					float d00 = pxr::GfDot(v0, v0);
					float d01 = pxr::GfDot(v0, v1);
					float d11 = pxr::GfDot(v1, v1);
					float d20 = pxr::GfDot(v2, v0);
					float d21 = pxr::GfDot(v2, v1);
					float denom = d00 * d11 - d01 * d01;

					float u = (d11 * d20 - d01 * d21) / denom;
					if(u < 0.0f || u > 1.0f) continue;

					float v = (d00 * d21 - d01 * d20) / denom;
					if((v < 0.f) || ((u + v) > 1.f)) continue;

					const pxr::GfVec3f projected_point = pPhantomTrimesh->getInterpolatedRestPosition(face_id, u, v);
        			float bind_dist = point_is_in_plane ? 0.f : distance(projected_point, curr_pt);
        			
        			//if(point_is_below_surface) {
        			//	bind_dist = -bind_dist;
        			//}

        			if(abs(bind_dist) < abs(bind.dist)) {
        				bind.face_id = face_id;
        				bind.dist = bind_dist;
        				bind.u = u;
        				bind.v = v;
        			}         			
        		} // faces loop
        	
        		if(bind.face_id != PointBindData::kInvalidFaceID) {
        			bound_curve_vertices_count += 1;
        		}

        	} // curve vertices loop

        	// This binds point to face ignoring 'u' and 'v' check
			auto bindCurvePointToPrimForceI = [&] (const uint32_t curve_vtx, const uint32_t face_id, PointBindData& bind) {
				const auto& face = faces[face_id];

				pxr::GfVec3f p0 = meshRestPositions[face.indices[0]];
				pxr::GfVec3f p1 = meshRestPositions[face.indices[1]];
				pxr::GfVec3f p2 = meshRestPositions[face.indices[2]];

				const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + curve_vtx); 

				const auto& face_normal = face.getRestNormal();
				const Plane face_plane(meshRestPositions[face.indices[0]], face_normal);
				const float face_distance = distance(face_plane, curr_pt);

				const bool point_is_in_plane = abs(face_distance) <= kEpsilon;
    			
    			if(!point_is_in_plane) { 
					const pxr::GfVec3f point_to_plane_vector = face_normal * face_distance;

					p0 += rest_vertex_normals[face.indices[0]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[0]]);
					p1 += rest_vertex_normals[face.indices[1]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[1]]);
					p2 += rest_vertex_normals[face.indices[2]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[2]]);

    			}

				const pxr::GfVec3f projected_pt = curr_pt - face_normal * face_distance; // project point on to face plane

				const pxr::GfVec3f v0 = p1 - p0, v1 = p2 - p0, v2 = curr_pt - p0;
				float d00 = pxr::GfDot(v0, v0);
				float d01 = pxr::GfDot(v0, v1);
				float d11 = pxr::GfDot(v1, v1);
				float d20 = pxr::GfDot(v2, v0);
				float d21 = pxr::GfDot(v2, v1);
				float denom = d00 * d11 - d01 * d01;

				bind.u = (d11 * d20 - d01 * d21) / denom;
				bind.v = (d00 * d21 - d01 * d20) / denom;

				const pxr::GfVec3f projected_point = pPhantomTrimesh->getInterpolatedRestPosition(face_id, bind.u, bind.v);
        		bind.dist = point_is_in_plane ? 0.f : distance(projected_point, curr_pt);
				bind.face_id = face_id;
			};

        	// TODO: Fully unbound curves loop to find best vertex-face candidate for each curve so we can populate remaining vertices
        	// in a subsequent pass
        	if(bound_curve_vertices_count == 0) {
        		for(uint32_t curve_vtx = 0; curve_vtx < curve_vertices_count; ++curve_vtx) {
        			auto& bind = pointBinds[curve_vertex_offset + curve_vtx];
        			pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + curve_vtx);
        			const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = kdtree.findNearestNeighbour(curve_root_pt);
        			const auto nearest_face_id = nearest_pt.first;
        			
        			bindCurvePointToPrimForceI(curve_vtx, nearest_face_id, bind);
					bound_curve_vertices_count += 1;
        		}
        	}

        	// Partially unbound curve vertices pass tha loops twice (forward and back) filling the gaps
			// Forward loop (starting from curve root)
			if((bound_curve_vertices_count > 0) && (bound_curve_vertices_count < curve_vertices_count)) {
				for(uint32_t curve_vtx = 1; curve_vtx < curve_vertices_count; ++curve_vtx) {
					const uint32_t ii = curve_vertex_offset + curve_vtx;
					const auto& prev_bind = pointBinds[ii - 1];
					auto& bind = pointBinds[ii];
					if(bind.isValid() || !prev_bind.isValid()) {
						continue;
					}

					bindCurvePointToPrimForceI(curve_vtx, prev_bind.face_id, bind);
					bound_curve_vertices_count += 1;
				}
			} // partially unbound vertices loop

			// Backward loop (starting from curve tip)
			if((bound_curve_vertices_count > 0) && (bound_curve_vertices_count < curve_vertices_count)) {
				for(uint32_t curve_vtx = curve_vertices_count - 2; curve_vtx != (uint32_t)-1; curve_vtx--) {
					const uint32_t ii = curve_vertex_offset + curve_vtx;
					const auto& prev_bind = pointBinds[ii + 1];
					auto& bind = pointBinds[ii];
					if(bind.isValid() || !prev_bind.isValid()) {
						continue;
					}

					bindCurvePointToPrimForceI(curve_vtx, prev_bind.face_id, bind);
					bound_curve_vertices_count += 1;
				}
			} // partially unbound vertices loop
			

        	if(bound_curve_vertices_count > 0) {
        		if(bound_curve_vertices_count == curve_vertices_count) {
        			curve_flags[curve_idx] = 1;
        			bound_curves++;
        		} else {
        			partially_bound_curves++;
        		}
        	} 
        } // curves loop
    };
    
    mPool.detach_blocks(0, curves_count, func);
	mPool.wait();
	//func(0, curves_count);

    dbg_printf("Bound curves count: %zu\n", size_t(bound_curves));
    dbg_printf("Partially bound curves count: %zu\n", size_t(partially_bound_curves));
    dbg_printf("Unbound curves count: %zu\n", curves_count - size_t(bound_curves + partially_bound_curves));

	return true; 
}

void WrapCurvesDeformer::setBindMode(WrapCurvesDeformer::BindMode mode) {
	if(mpWrapCurvesDeformerData->getBindMode() == mode) return;

	mpWrapCurvesDeformerData->setBindMode(mode);
	makeDirty();
}

const WrapCurvesDeformer::BindMode& WrapCurvesDeformer::getBindMode() const {
	return mpWrapCurvesDeformerData->getBindMode();
}

WrapCurvesDeformer::~WrapCurvesDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston