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


namespace Piston {

WrapCurvesDeformer::WrapCurvesDeformer(): BaseCurvesDeformer() {
	dbg_printf("WrapCurvesDeformer::WrapCurvesDeformer()\n");
}

WrapCurvesDeformer::SharedPtr WrapCurvesDeformer::create() {
	return SharedPtr(new WrapCurvesDeformer());
}

const std::string& WrapCurvesDeformer::toString() const {
	static const std::string kFastDeformerString = "WrapCurvesDeformer";
	return kFastDeformerString;
}

bool WrapCurvesDeformer::deformImpl(pxr::UsdTimeCode time_code) {
	PROFILE("WrapCurvesDeformer::deformImpl");
	dbg_printf("WrapCurvesDeformer::deformImpl()\n");
	if(!mpPhantomTrimesh) return false;

	if(!mpPhantomTrimesh->update(mMeshGeoPrimHandle, time_code)) {
		return false;
	}

	pxr::UsdGeomCurves curves(mCurvesGeoPrimHandle.getPrim());
	
	if(!mpCurvesContainer || mpCurvesContainer->empty()) {
		return false;
	}

	pxr::VtArray<pxr::GfVec3f>& points = mpCurvesContainer->getPointsCache();

	assert(points.size() == mPointBinds.size());

	buildVertexNormals(mpAdjacency.get(), mpPhantomTrimesh.get(), mLiveVertexNormals, true);


	mPool.detach_blocks(0, mPointBinds.size(),
        [&](const std::size_t start, const std::size_t end)
        {

        	for(size_t i = start; i < end; ++i) {
        		const auto& bind = mPointBinds[i];
        		if(bind.face_id == PointBindData::kInvalidFaceID) continue;

        		const auto& face = mpPhantomTrimesh->getFace(bind.face_id);
        		pxr::GfVec3f interpolated_normal = 
        			pxr::GfGetNormalized(bind.u * mLiveVertexNormals[face.indices[1]] + bind.v * mLiveVertexNormals[face.indices[2]] + (1.f - bind.u - bind.v) * mLiveVertexNormals[face.indices[0]]);
        		//points[i] = mpPhantomTrimesh->getInterpolatedLivePosition(bind.face_id, bind.u, bind.v) + interpolated_normal * bind.dist;
        	}

        }
    );
    mPool.wait();

	if(!curves.GetPointsAttr().Set(points, time_code)) {
		return false;
	}

	return true;
}

bool WrapCurvesDeformer::buildDeformerData(pxr::UsdTimeCode rest_time_code) {
	PROFILE("WrapCurvesDeformer::buildDeformerData");
	dbg_printf("WrapCurvesDeformer::buildDeformerData()\n");

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

	// First triangulate using simple "fan" triangulation
	const size_t src_mesh_face_count = mpAdjacency->getFaceCount();

	for(size_t face_id = 0; face_id < src_mesh_face_count; ++face_id) {
		const uint32_t face_vertex_count = mpAdjacency->getFaceVertexCount(face_id);
		
		if(face_vertex_count < 3 ) {
			std::cerr << "Source mesh polygon " << face_id << " is invalid !!!" << std::endl;
			continue;
		}
		
		const uint32_t face_vertex_offset = mpAdjacency->getFaceVertexOffset(face_id);

		switch(face_vertex_count) {
			case 3:
				mpPhantomTrimesh->getOrCreate(
					mpAdjacency->getFaceVertex(face_id, 0), 
					mpAdjacency->getFaceVertex(face_id, 1),
					mpAdjacency->getFaceVertex(face_id, 2)
				);
				break;
			default:
				for(uint32_t ii = 1; ii < (face_vertex_count - 1); ++ii) {
					mpPhantomTrimesh->getOrCreate(
						mpAdjacency->getFaceVertex(face_id, 0), 
						mpAdjacency->getFaceVertex(face_id, ii % face_vertex_count),
						mpAdjacency->getFaceVertex(face_id, (ii + 1) % face_vertex_count)
					);
				}
				break;
		}
	}

	const size_t tri_face_count = mpPhantomTrimesh->getFaceCount();

	dbg_printf("%zu source mesh faces triangulated to %zu triangles\n", src_mesh_face_count, tri_face_count);

	std::vector<pxr::GfVec3f> rest_vertex_normals;
	buildVertexNormals(mpAdjacency.get(), mpPhantomTrimesh.get(), rest_vertex_normals, false);
	mLiveVertexNormals.resize(rest_vertex_normals.size());

	// Bind curve points
	dbg_printf("Binding %zu curves (%zu total vertices) using thread pool.\n", mpCurvesContainer->getCurvesCount(), mpCurvesContainer->getTotalVertexCount());

	auto threads_timer = Timer();
	threads_timer.start();

	const size_t curves_count = mpCurvesContainer->getCurvesCount();
	std::atomic<size_t> bound_curves = 0;
	std::atomic<size_t> partially_bound_curves = 0;

	mPointBinds.resize(mpCurvesContainer->getTotalVertexCount());
	mPool.detach_blocks(0, curves_count,
        [&, &rest_vertex_normals](const std::size_t start, const std::size_t end)
        {
        	const std::optional<std::size_t> thread_index = BS::this_thread::get_index();
        	dbg_printf("Binding curves from %zu to %zu by thread id #%zu\n", start, end, *thread_index);

        	const auto& meshRestPositions = mpPhantomTrimesh->getRestPositions();
        	const auto& faces = mpPhantomTrimesh->getFaces();
        	const uint32_t faces_count = faces.size();

            for(std::size_t curve_idx = start; curve_idx < end; ++curve_idx) {

            	const PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_idx);
            	if(curve_data_ptr.first < 1) continue;

            	const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);
            	uint32_t bound_curve_vertices_count = 0;

            	const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_idx);
            	const uint32_t curve_vertex_offset = mpCurvesContainer->getCurveVertexOffset(curve_idx);
            	
            	for(uint32_t i = 0; i < curve_vertices_count; ++i) {

            		auto& bind = mPointBinds[curve_vertex_offset + i];
            		bind.face_id = PointBindData::kInvalidFaceID;
            		pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 

            		for(uint32_t face_id = 0; face_id < faces_count; ++face_id) {
            			const auto& face = faces[face_id];
            			const auto& face_normal = face.getRestNormal();
            			const Plane face_plane(meshRestPositions[face.indices[0]], face_normal);
            			const pxr::GfVec3f point_to_plane_vector = face_normal * distance(face_plane, curr_pt);

            			const pxr::GfVec3f vec0 = rest_vertex_normals[face.indices[0]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[0]]);
            			const pxr::GfVec3f vec1 = rest_vertex_normals[face.indices[1]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[1]]);
            			const pxr::GfVec3f vec2 = rest_vertex_normals[face.indices[2]] / pxr::GfDot(point_to_plane_vector, rest_vertex_normals[face.indices[2]]);

            			const pxr::GfVec3f p0 = meshRestPositions[face.indices[0]] + vec0;
            			const pxr::GfVec3f p1 = meshRestPositions[face.indices[1]] + vec1;
            			const pxr::GfVec3f p2 = meshRestPositions[face.indices[2]] + vec2;

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

            			float squared_bind_dist = distanceSquared(mpPhantomTrimesh->getInterpolatedRestPosition(face_id, u, v), curr_pt);

            			if(squared_bind_dist < bind.dist) {
            				bind.face_id = face_id;
            				bind.dist = squared_bind_dist;
            				bind.u = u;
            				bind.v = v;
            			}         			
            		}
            	
            		if(bind.face_id != PointBindData::kInvalidFaceID) {
            			bind.dist = sqrt(bind.dist);
            			bound_curve_vertices_count += 1;
            		}

            	}

            	if(bound_curve_vertices_count > 0) {
            		if(bound_curve_vertices_count == curve_vertices_count) {
            			bound_curves++;
            		} else {
            			partially_bound_curves++;
            		}
            	} 
            }
        });
    mPool.wait();

    threads_timer.stop();

    dbg_printf("%zu threads finished in %s\n", mPool.get_thread_count(), threads_timer.toString().c_str());
    dbg_printf("Bound curves count: %zu\n", size_t(bound_curves));
    dbg_printf("Partially bound curves count: %zu\n", size_t(partially_bound_curves));
    dbg_printf("Unbound curves count: %zu\n", curves_count - size_t(bound_curves + partially_bound_curves));

	return true; 
}


bool WrapCurvesDeformer::bindCurveVertexToTriface(uint32_t curve_vtx, uint32_t face_id, PointBindData& bind) {

	return false;
}

bool WrapCurvesDeformer::buildPointsBindingData(pxr::UsdTimeCode rest_time_code) {
	if(!mpCurvesContainer) return false;

	return true;
}

WrapCurvesDeformer::~WrapCurvesDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston