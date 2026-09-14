#include "simple_profiler.h"
#include "point_instancer_deformer.h"

#include "common.h"
#include "logging.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/matrix4f.h>
#include <pxr/base/gf/rotation.h>

#include <atomic>
#include <random>
#include <algorithm>
#include <optional>


namespace Piston {

static constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
static constexpr float kMaxFloat = std::numeric_limits<float>::max();

PointInstancerDeformer::PointInstancerDeformer(const std::string& name): BaseDeformer(BaseDeformer::Type::POINT_INSTANCER, name), mBindMode(BindMode::SIMPLE) {

}

PointInstancerDeformer::SharedPtr PointInstancerDeformer::create(const std::string& name) {
	return SharedPtr(new PointInstancerDeformer(name));
}

const std::string& PointInstancerDeformer::toString() const {
	static const std::string kFastDeformerString = "PointInstancerDeformer";
	return kFastDeformerString;
}

bool PointInstancerDeformer::validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	return isMeshGeoPrim(geoPrim);
}

void PointInstancerDeformer::setInstancerGeoPrim(const pxr::UsdPrim& prim) {
	if(!prim.IsValid() || mInstancerGeoPrimHandle == prim) return;

	if(!isPointInstancerGeoPrim(prim)) {
		DLOG_ERR << "Instancer geometry prim is not \"pxr::UsdGeomPointInstancer\"!";
		return;
	}

	if(mDeformerGeoPrimHandle == prim) {
		DLOG_ERR << "Can't use the same prim " << mDeformerGeoPrimHandle << " for deformer and Instancer geometry !!!";
		return;
	}

	auto new_handle = UsdPrimHandle(prim);
	const bool same_topology = mInstancerGeoPrimHandle.isValid() ? isSameTopology(mInstancerGeoPrimHandle, new_handle, getRestTimeCode()) : false;

	mInstancerGeoPrimHandle = std::move(new_handle);
	if(!same_topology) {
		makeDirty();
	}

	DLOG_DBG << "Instancer geometry prim is set to: " << mInstancerGeoPrimHandle;
}

const pxr::UsdPrim& PointInstancerDeformer::getInstancerGeoPrim() const {
	return mInstancerGeoPrimHandle.getPrim();
}

void PointInstancerDeformer::setInstancerRestAttrName(const std::string& name) {
	if(mInstancerGeoPrimHandle.getRestAttrName() == name) return;
	mInstancerGeoPrimHandle.setRestAttrName(name);
	makeDirty();

	DLOG_DBG << "Deformer instancer rest attribute name is set to: " <<  name;
}

void PointInstancerDeformer::invalidateData(DeformerDataCache& cache) {
	cache.invalidate(mpAdjacencyData);
	cache.invalidate(mpPhantomTrimeshData);
	cache.invalidate(mpPointInstancerDeformerData);
}

bool PointInstancerDeformer::deformerOutputsOrientations() const {
	if(!mInstancerGeoPrimHandle.isValid()) return false;

	pxr::UsdGeomPointInstancer instancer(mInstancerGeoPrimHandle.getPrim());
	pxr::UsdAttribute attr_o = instancer.GetOrientationsAttr();

	if(!attr_o) return false;
	return true;
}

size_t PointInstancerDeformer::getDeformedPointsCount() const {
	LOG_DBG << "PointInstancerDeformer::getDeformedPointsCount";
	assert(mpInstancerContainer);

	LOG_DBG << "PointInstancerDeformer::getDeformedPointsCount" << mpInstancerContainer->getInstanceCount();
	return mpInstancerContainer->getInstanceCount();
}

bool PointInstancerDeformer::outputDeformedPoints(const PointsList* pPointsList, pxr::UsdTimeCode time_code) {
	assert(pPointsList);

	pxr::UsdGeomPointInstancer instancer(mInstancerGeoPrimHandle.getPrim());
	pxr::UsdAttribute attr_p = instancer.GetPositionsAttr();

	if(!attr_p || !attr_p.Set(pPointsList->getPointsVtArray(), time_code)) {
		return false;
	}

	pxr::UsdAttribute attr_o = instancer.GetOrientationsAttr();
	const auto& orientations = pPointsList->getOrientationsVtArray();
	if(attr_o && orientations.size() == pPointsList->size()) {
		attr_o.Set(orientations, time_code);
	}

	return true;
}

bool PointInstancerDeformer::outputVelocites(const PointsList* pVelocitiesList, pxr::UsdTimeCode time_code) {
	assert(pVelocitiesList);

	pxr::UsdGeomPointInstancer instancer(mInstancerGeoPrimHandle.getPrim());
	pxr::UsdAttribute attr_v = instancer.GetVelocitiesAttr();

	if(!attr_v || !attr_v.Set(pVelocitiesList->getPointsVtArray(), time_code)) {
		return false;	
	}

	return true;
}

bool PointInstancerDeformer::writeJsonDataToPrimImpl() const {
	LOG_DBG << "PointInstancerDeformer::buildDeformerDataImpl";

	if(mpAdjacencyData && !mDeformerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpAdjacencyData.get())) {
		DLOG_ERR << "Error writing " << mpAdjacencyData->typeName() << " deformer data to json !";
		return false;
	}

	if(mpPhantomTrimeshData && !mInstancerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpPhantomTrimeshData.get())) {
		DLOG_ERR << "Error writing " << mpPhantomTrimeshData->typeName() << " curves data to json !";
		return false;
	}

	if(mpPointInstancerDeformerData && !mInstancerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpPointInstancerDeformerData.get())) {
		DLOG_ERR << "Error writing " << mpPointInstancerDeformerData->typeName() << " deformer data to json !";	
		return false;
	}
	return true;
}

bool PointInstancerDeformer::__deform__(PointsList& points_list, bool multi_threaded, pxr::UsdTimeCode time_code) {
	DLOG_DBG << "PointInstancerDeformer::__deform__";

	assert(mpInstancerContainer);
	assert(mpDeformerMeshContainer);
	assert(mpPointInstancerDeformerData);

	assert(points_list.size() == mpInstancerContainer->getInstanceCount());

	DLOG_DBG << "Points count " << points_list.size();
	DLOG_DBG << "Instances container instances count " << mpInstancerContainer->getInstanceCount();

	if(mpPointInstancerDeformerData->getBindMode() == PointInstancerDeformerData::BindMode::SIMPLE) {
		return __deform__simple__(points_list, multi_threaded, time_code);
	} else {
		return __deform__mppp__(points_list, multi_threaded, time_code);
	}
}

bool PointInstancerDeformer::__deform__simple__(PointsList& points_list, bool multi_threaded, pxr::UsdTimeCode time_code) {
	DLOG_DBG << "PointInstancerDeformer::__deform__simple__";

	assert(mpInstancerContainer);

	const MeshContainer::ContainerType& mesh_live_positions = mpDeformerMeshContainer->getLivePositions();

	const auto& bindings = mpPointInstancerDeformerData->mPointBinds;
	assert(points_list.size() == bindings.size());

	assert(mpAdjacencyData);
	const UsdGeomMeshFaceAdjacency* pAdjacency = mpAdjacencyData->getAdjacencyFinal();
	assert(pAdjacency);

	/* 
	// We dont use these for now
	mLiveVertexNormals.resize(pAdjacency->getVertexCount());
	buildVertexNormals(pAdjacency, nullptr, mLiveVertexNormals, mesh_live_positions, (multi_threaded ? &mPool : nullptr));
	evaluateBaseMeshLiveFrames(pAdjacency, mLiveVertexNormals);
	*/

	const auto& instancer_rest_orientations = mpInstancerContainer->getRestOrientations();

	auto* pOutPoints = points_list.points();
	auto* pOutOrientations = points_list.orientations();

	uint32_t projected_inside_count = 0;

	auto func = [&](const std::size_t start, const std::size_t end) {
		for(size_t i = start; i < end; ++i) {
			const auto& bind = bindings[i];
			assert(bind.isValid());

			const bool is_quad_bound = bind.isQuadBound();

			static const pxr::GfVec3f sZeroPoint(0.0, 0.0, 0.0); // just for code logic
			const pxr::GfVec3f& p0 = mesh_live_positions[bind.point_indices[0]];
			const pxr::GfVec3f& p1 = mesh_live_positions[bind.point_indices[1]];
			const pxr::GfVec3f& p2 = mesh_live_positions[bind.point_indices[2]];
			const pxr::GfVec3f& p3 = is_quad_bound ? mesh_live_positions[bind.point_indices[3]] : sZeroPoint;

			pxr::GfMatrix4f liveMatrix;
	    	liveMatrix.SetIdentity();

	    	pxr::GfVec3f du, dv, normal;

			if(is_quad_bound) {
				// quad bound
				du = (p3 - p0) * (1.0f - bind.u) + (p2 - p1) * bind.u;
				dv = (p1 - p0) * (1.0f - bind.v) + (p2 - p3) * bind.v;
			} else {
				// Triangle bound
				du = p1 - p0;
	    		dv = p2 - p0;
			}

			normal = pxr::GfCross(du, dv).GetNormalized();

			if (bind.edge_id < 0) {
				// prim bound
				liveMatrix.SetRow3(0, du);
				liveMatrix.SetRow3(1, dv);
				liveMatrix.SetRow3(2, normal);

				if(is_quad_bound) {
					float w0 = (1.0f - bind.u) * (1.0f - bind.v);
	   				float w1 = bind.u * (1.0f - bind.v);
	   				float w2 = bind.u * bind.v;
	   				float w3 = (1.0f - bind.u) * bind.v;
	   				// Linearly combine the positions using the weights
	    			pxr::GfVec3f surfacePoint = (p0 * w0) + (p1 * w1) + (p2 * w2) + (p3 * w3);
	    			liveMatrix.SetRow3(3, surfacePoint);
	    		} else {
					liveMatrix.SetRow3(3, p0);
				}
			} else {
				// edge bound
				// rebuild edge-locked orthonormal matrix
		        pxr::GfVec3f origin, axisX;
		        if(is_quad_bound) {
		        	// quad
			        if (bind.edge_id == 0) {
			            origin = p0; axisX = p1 - p0;
			        } else if (bind.edge_id == 1) {
			            origin = p1; axisX = p2 - p1;
			        } else if (bind.edge_id == 2) {
			            origin = p2; axisX = p3 - p2;
			        }else {
			            origin = p3; axisX = p0 - p3;
			        }
			    } else {
			    	// triangle
			    	if (bind.edge_id == 0) {
			            origin = p0; axisX = p1 - p0;
			        } else if (bind.edge_id == 1) {
			            origin = p1; axisX = p2 - p1;
			        } else {
			            origin = p2; axisX = p0 - p2;
			        }
			    }

			    if(bind.isOutside()) {
					axisX = axisX.GetNormalized();
				}

		        pxr::GfVec3f normalY = pxr::GfCross(axisX, normal).GetNormalized();

				liveMatrix.SetRow3(0, axisX);
				liveMatrix.SetRow3(1, normalY);
				liveMatrix.SetRow3(2, normal);
				liveMatrix.SetRow3(3, origin);
			}

			pOutPoints[i] = liveMatrix.Transform(bind.localPos);

			if(pOutOrientations) {
				pxr::GfMatrix4f restMatrix;
				restMatrix.SetRow(0, pxr::GfVec4f(bind.restTangent[0], bind.restTangent[1], bind.restTangent[2], 0.0f));
				restMatrix.SetRow(1, pxr::GfVec4f(bind.restBinormal[0], bind.restBinormal[1], bind.restBinormal[2], 0.0f));	
				restMatrix.SetRow(2, pxr::GfVec4f(bind.restNormal[0], bind.restNormal[1], bind.restNormal[2], 0.0f));	
				restMatrix.SetRow(3, pxr::GfVec4f(bind.localPos[0], bind.localPos[1], bind.localPos[2], 1.0f));

				pxr::GfMatrix3f rLive = liveMatrix.ExtractRotationMatrix();
				pxr::GfMatrix3f rRest = restMatrix.ExtractRotationMatrix();

				rLive.Orthonormalize();
				rRest.Orthonormalize();

				pxr::GfMatrix3f rDeltaLocal = rRest.GetInverse() * rLive;
				pxr::GfRotation deltaRotation = rDeltaLocal.ExtractRotation();

				pxr::GfQuatd deltaQuat = deltaRotation.GetQuat();
				pOutOrientations[i] = pxr::GfQuath(deltaQuat * instancer_rest_orientations[i]);
			}
		}
	};

	if(multi_threaded) {
		BS::multi_future<void> blocks = mPool.submit_blocks(0u, bindings.size(), func);
		blocks.wait();
	} else {
		func(0u, bindings.size());
	}

	return true;
}

bool PointInstancerDeformer::__deform__mppp__(PointsList& points_list, bool multi_threaded, pxr::UsdTimeCode time_code) {
	DLOG_DBG << "PointInstancerDeformer::__deform__mppp__";

	const auto& bindings = mpPointInstancerDeformerData->mMPPPointBindings;
	assert(points_list.size() == bindings.size());

	const MeshContainer::ContainerType& mesh_live_positions = mpDeformerMeshContainer->getLivePositions();

	const size_t stride = bindings.getStride(); // neighbors per bind

	auto* pOutPoints = points_list.points();
	auto* pOutOrientations = points_list.orientations();

	assert(mpAdjacencyData);
	const UsdGeomMeshFaceAdjacency* pAdjacency = mpAdjacencyData->getAdjacencyFinal();
	assert(pAdjacency);

	mLiveVertexNormals.resize(pAdjacency->getVertexCount());
	buildVertexNormals(pAdjacency, nullptr, mLiveVertexNormals, mesh_live_positions, (multi_threaded ? &mPool : nullptr));

	const auto& instancer_rest_orientations = mpInstancerContainer->getRestOrientations();

	evaluateBaseMeshLiveFrames(pAdjacency, mLiveVertexNormals);
	assert(mLiveVertexNormals.size() == mesh_live_positions.size());

	for(size_t i = 0; i < points_list.size(); ++i) {
		pxr::GfVec3f blendedPos(0.0f);
        pxr::GfQuatf blendedQuat(0.0f, 0.0f, 0.0f, 0.0f);
        size_t flatIndexStart = i * stride;

		// Sequence reading from a single block of contiguous memory
        for (size_t n = 0; n < stride; ++n) {
            const PointInstancerDeformerData::BindSample& sample = bindings.mSamples[flatIndexStart + n];
            size_t mIdx = sample.meshVertexIndex;
            float w = sample.weight;

            // Direct index array lookup
            const pxr::GfQuatf& activeSurfaceAlign = mDeformerMeshLiveFrames[mIdx];

            // Reconstruct precise world position relative to the moving mesh contour
            pxr::GfVec3f worldOffset = activeSurfaceAlign.Transform(sample.localOffset);
            blendedPos += (mesh_live_positions[mIdx] + worldOffset) * w;
        
            // Reconstruct absolute instance orientations
            pxr::GfQuatf neighborInstanceOrient = activeSurfaceAlign * instancer_rest_orientations[i];
            
			// Linear quaternion blending accumulator with double-hemisphere alignment
            if (pxr::GfDot(neighborInstanceOrient, blendedQuat) < 0.0f) {
                blendedQuat -= neighborInstanceOrient * w;
            } else {
                blendedQuat += neighborInstanceOrient * w;
            }
        }

		pOutPoints[i] = blendedPos;

		if(pOutOrientations) {
        	// Store orientation compressed down to GfQuath
        	pOutOrientations[i] = pxr::GfQuath(blendedQuat.GetNormalized());
		}
	}

	return true;
}

bool PointInstancerDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	DLOG_DBG << "PointInstancerDeformer::buildDeformerDataImpl";

	if(!mInstancerGeoPrimHandle) {
		DLOG_ERR << "No instancer UsdPrim is set !!!";
		return false;
	}

	if(!mpInstancerContainer) {
		mpInstancerContainer = InstancerContainer::create();
		if(!mpInstancerContainer) {
			DLOG_ERR << "Error creating instancer container for prim " << mInstancerGeoPrimHandle << " !";
			return false;
		}
	} 
	
	if(!mpInstancerContainer->init(mInstancerGeoPrimHandle, rest_time_code)) {
		DLOG_ERR << "Error initializing instancer container for prim " << mInstancerGeoPrimHandle << " !";
		return false;
	}

	// 
	DeformerDataCache& dataCache = DeformerDataCache::getInstance();

	bool adjacency_data_created = true;
	if(!mpAdjacencyData) {
		mpAdjacencyData = dataCache.getOrCreateData<SerializableUsdGeomMeshFaceAdjacency>(this, mDeformerGeoPrimHandle, rest_time_code, adjacency_data_created);
	}

	// Get primitive adjacency json data if present
	if(adjacency_data_created || !getReadJsonDataState() || !mDeformerGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpAdjacencyData.get())) {
		// Build in place if no json data present or not needed
		if(!mpAdjacencyData->buildInPlace(mDeformerGeoPrimHandle)) {
			DLOG_ERR << "Error building mesh adjacency data!";
			return false;
		}
	}

	if(!mpAdjacencyData || !mpAdjacencyData->getAdjacency() || !mpAdjacencyData->getAdjacency()->isValid()) {
		DLOG_ERR << "No valid mesh adjacency data!";
		return false;
	}

	bool trimesh_data_created = true;
	if(!mpPhantomTrimeshData) {
		mpPhantomTrimeshData = dataCache.getOrCreateData<SerializablePhantomTrimesh>(this, {&mDeformerGeoPrimHandle, &mInstancerGeoPrimHandle}, rest_time_code, trimesh_data_created);
	}

	// Get phantom mesh json data if present
	if(trimesh_data_created || !getReadJsonDataState() || !mInstancerGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpPhantomTrimeshData.get())) {
		// Build in place if no json data present or not needed
		if(!mpPhantomTrimeshData->buildInPlace(mDeformerGeoPrimHandle)) {
			DLOG_ERR << "Error building phantom mesh data!";
			return false;
		}
	}

	assert(mpAdjacencyData);
	const auto* pAdjacency = mpAdjacencyData->getAdjacencyFinal();
	
	assert(mpPhantomTrimeshData);
	auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();

	bool deformer_data_created;
	if(!mpPointInstancerDeformerData) {
		mpPointInstancerDeformerData = dataCache.getOrCreateData<PointInstancerDeformerData>(this, {&mDeformerGeoPrimHandle, &mInstancerGeoPrimHandle}, rest_time_code, deformer_data_created);
		mpPointInstancerDeformerData->setBindMode(mBindMode);
	}

	if(deformer_data_created || !mpPointInstancerDeformerData->isValid() || !mpPhantomTrimeshData->isValid()) {
		if(!getReadJsonDataState() || !mInstancerGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpPointInstancerDeformerData.get())) {
			// Build deformer data in place if no json data present or not needed

			std::vector<pxr::GfVec3f> rest_vertex_normals;
			buildVertexNormals(pAdjacency, nullptr, rest_vertex_normals, mpDeformerMeshContainer->getRestPositions(), (multi_threaded ? &mPool : nullptr));
			mLiveVertexNormals.resize(rest_vertex_normals.size());

			// Bind curve points
			DLOG_DBG << "Binding " << mpInstancerContainer->getInstanceCount() << " instancer points.";	

			bool result = false;
			auto threads_timer = Timer();
			threads_timer.start();

			// Build bind data
			switch(mpPointInstancerDeformerData->getBindMode()) {
				case BindMode::SIMPLE:
					result = buildDeformerData_SimpleMode(multi_threaded, rest_vertex_normals, rest_time_code);
					break;
				default:
					result = buildDeformerData_MPPPMode(multi_threaded, rest_vertex_normals, rest_time_code);
					break;
			}

			//auto& face_flags = pPhantomTrimesh->getFaceFlags(); 
			//assert(face_flags.size() == pPhantomTrimesh->getFaceCount());

			//for(const auto& bind: mpPointInstancerDeformerData->getPointBinds()) {
			//	if(bind.isValid()) face_flags[bind.face_id] = PhantomTrimesh::TriFace::Flags::Bound;
			//}

			threads_timer.stop();
			if(multi_threaded) {
				DLOG_TRC << "PointInstancerDeformer::buildDeformerDataImpl() " << mPool.get_thread_count() << " threads finished in " << threads_timer.toString();
			} else {
				DLOG_TRC << "PointInstancerDeformer::buildDeformerDataImpl() finished in " << threads_timer.toString();
			}

			mpPointInstancerDeformerData->setValid(result);
		}
		mpPhantomTrimeshData->setValid(mpPointInstancerDeformerData->isValid());
	}

	return mpPointInstancerDeformerData->isValid();
}

void PointInstancerDeformer::captureBaseMeshRestFrames(const UsdGeomMeshFaceAdjacency* pAdjacency, const std::vector<pxr::GfVec3f>& rest_vertex_normals) {
	assert(mpDeformerMeshContainer);
	const MeshContainer::ContainerType& mesh_rest_positions = mpDeformerMeshContainer->getRestPositions();
	assert(rest_vertex_normals.size() == mesh_rest_positions.size());

	mDeformerMeshRestFrames.resize(mesh_rest_positions.size());

	for (size_t i = 0; i < mesh_rest_positions.size(); ++i) {
        const pxr::GfVec3f& normal = rest_vertex_normals[i];

        pxr::GfVec3f tangent, bitangent;
        // This is safe because it only runs once at rest pose to build an anchor
        pxr::GfBuildOrthonormalFrame(normal, &tangent, &bitangent); 

        pxr::GfMatrix3f rotationMatrix;
        rotationMatrix.SetRow(0, tangent);
        rotationMatrix.SetRow(1, normal);
        rotationMatrix.SetRow(2, bitangent);

        pxr::GfQuaternion doubleQuat = rotationMatrix.GetTranspose().ExtractRotationQuaternion();
        pxr::GfQuatf surfaceAlign(static_cast<float>(doubleQuat.GetReal()), pxr::GfVec3f(doubleQuat.GetImaginary()));
        
        mDeformerMeshRestFrames[i] = { surfaceAlign.GetNormalized(), tangent };
    }
}

void PointInstancerDeformer::evaluateBaseMeshLiveFrames(const UsdGeomMeshFaceAdjacency* pAdjacency, const std::vector<pxr::GfVec3f>& live_vertex_normals) {
    assert(mpDeformerMeshContainer);
	const MeshContainer::ContainerType& mesh_live_positions = mpDeformerMeshContainer->getLivePositions();
	assert(live_vertex_normals.size() == mesh_live_positions.size());

    mDeformerMeshLiveFrames.resize(mDeformerMeshRestFrames.size());

    for (size_t i = 0; i < mesh_live_positions.size(); ++i) {
        const pxr::GfVec3f& normal = live_vertex_normals[i];
       
        // Retrieve the locked reference direction from Phase 1
        const pxr::GfVec3f& cachedTangent = mDeformerMeshRestFrames[i].tangentGuide;

        // Run Gram-Schmidt to flatten it perfectly onto the new moving surface plane
        pxr::GfVec3f animTangent = (cachedTangent - normal * pxr::GfDot(cachedTangent, normal)).GetNormalized();
        pxr::GfVec3f animBitangent = pxr::GfCross(normal, animTangent).GetNormalized();

        pxr::GfMatrix3f rotationMatrix;
        rotationMatrix.SetRow(0, animTangent);
        rotationMatrix.SetRow(1, normal);
        rotationMatrix.SetRow(2, animBitangent);

        pxr::GfQuaternion doubleQuat = rotationMatrix.GetTranspose().ExtractRotationQuaternion();
        pxr::GfQuatf surfaceAlign(static_cast<float>(doubleQuat.GetReal()), pxr::GfVec3f(doubleQuat.GetImaginary()));
        
        mDeformerMeshLiveFrames[i] = surfaceAlign.GetNormalized();
    }
}

bool PointInstancerDeformer::buildDeformerData_MPPPMode(bool multi_threaded, const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code) {
	assert(mpPointInstancerDeformerData);
	assert(mpDeformerMeshContainer);
	assert(mpPhantomTrimeshData);
	assert(mpInstancerContainer);

	const auto instances_count = mpInstancerContainer->getInstanceCount();

	const auto& instancer_rest_positions = mpInstancerContainer->getRestInstancePoints();
	const auto& instancer_rest_orientations = mpInstancerContainer->getRestOrientations();

	const bool has_orientations = (instancer_rest_positions.size() == instancer_rest_orientations.size());

	assert(instances_count == instancer_rest_positions.size());

	auto& bindings = mpPointInstancerDeformerData->mMPPPointBindings;
	bindings.resize(instances_count);
  
	assert(mpAdjacencyData);
	const UsdGeomMeshFaceAdjacency* pAdjacency = mpAdjacencyData->getAdjacencyFinal();
	assert(pAdjacency);

	// Build kdtree
	const MeshContainer::ContainerType& mesh_rest_positions = mpDeformerMeshContainer->getRestPositions();
	std::unique_ptr<neighbour_search::KDTree<float, 3>> pKDtree = std::make_unique<neighbour_search::KDTree<float, 3>>(mesh_rest_positions, false /* no threads */);

	captureBaseMeshRestFrames(pAdjacency, rest_vertex_normals);
	
	auto func = [&](const uint32_t start, const uint32_t end) {
    	if(multi_threaded) {
			LOG_TRC << "Binding instances from " << start << " to " << end << " by thread id #" << *BS::this_thread::get_index();
		}

		const auto kNeighbors = mpPointInstancerDeformerData->mMPPPointBindings.getStride();
		std::vector<neighbour_search::KDTree<float, 3>::ReturnType> closest_deformer_points(kNeighbors);

		for(uint32_t i = start; i < end; ++i) {
			const pxr::GfVec3f& instPos = instancer_rest_positions[i];

			uint32_t flatIndexStart = i * kNeighbors;

			// Nearest K points
			pKDtree->findKNearestNeighbours(instPos, kNeighbors, closest_deformer_points);

			// Weights
			float totalWeight = 0.0f;
	        std::vector<float> rawWeights(kNeighbors);
	        for (uint32_t n = 0; n < kNeighbors; ++n) {
	            float dist_squared = std::max(closest_deformer_points[n].second, 1e-5f);
	            rawWeights[n] = 1.0f / dist_squared;
	            totalWeight += rawWeights[n];
	        }

	        // Populate flat lookup tables
	        for (uint32_t n = 0; n < kNeighbors; ++n) {
	            uint32_t mIdx = closest_deformer_points[n].first;
	            float normWeight = rawWeights[n] / totalWeight;

	            // Instantly grab precomputed alignment frames
	            const pxr::GfQuatf& surfaceAlign = mDeformerMeshRestFrames[mIdx].alignmentQuat;

	            pxr::GfVec3f worldOffset = instPos - mesh_rest_positions[mIdx];
	            pxr::GfVec3f localOffset = surfaceAlign.GetInverse().Transform(worldOffset);

	            bindings.mSamples[flatIndexStart + n] = { mIdx, normWeight, localOffset };
	        }
		}
	};

	DLOG_INF << "Binding instances using MPPP method.";
    
    if(multi_threaded) {
    	BS::multi_future<void> blocks = mPool.submit_blocks(0, instances_count, func);
		blocks.wait();
	} else {
		func(0, instances_count);
	}

	return true;
}

static std::vector<uint32_t> combineVectors(const std::vector<uint32_t>& short_vec, const std::vector<uint32_t>& large_vec) {
    assert(short_vec.size() <= large_vec.size());

    std::vector<uint32_t> result = short_vec;
    result.reserve(large_vec.size());
    
    std::unordered_set<uint32_t> seen(short_vec.begin(), short_vec.end());
    
    for (uint32_t val : large_vec) {
        if (seen.find(val) == seen.end()) {
            result.push_back(val);
        }
    }
    
    return result;
}


bool PointInstancerDeformer::buildDeformerData_SimpleMode(bool multi_threaded, const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code) {
	const auto instances_count = mpInstancerContainer->getInstanceCount();
	const auto& instancer_rest_positions = mpInstancerContainer->getRestInstancePoints();

	std::vector<PointBindData>& bindings = mpPointInstancerDeformerData->mPointBinds;
	bindings.resize(instances_count);

	const UsdGeomMeshFaceAdjacency* pAdjacency = mpAdjacencyData->getAdjacencyFinal();
	assert(pAdjacency);

	// Build kdtree
	const MeshContainer::ContainerType& mesh_rest_positions = mpDeformerMeshContainer->getRestPositions();

	neighbour_search::KDTree<float, 3> deformer_restpoints_kdtree = neighbour_search::KDTree<float, 3>(mesh_rest_positions, multi_threaded);

	const bool ignore_outside_test = false;

	uint32_t projected_inside_count = 0;

	/* 
	// We dont use it for now
	captureBaseMeshRestFrames(pAdjacency, rest_vertex_normals);
	*/

	uint32_t mesh_prims_count = pAdjacency->getPrimCount();
	std::vector<uint32_t> mesh_prims(mesh_prims_count);
	for(uint32_t i = 0; i < mesh_prims_count; ++i) mesh_prims[i] = i;

	auto func = [&](const std::size_t start, const std::size_t end) {
    	if(multi_threaded) {
			DLOG_TRC << "Binding instances from " << start << " to " << end << " by thread id #" << *BS::this_thread::get_index();
		}

		for(size_t i = start; i < end; ++i) {
			PointInstancerDeformerData::PointBindData& bind = bindings[i];
			bind.flags = PointBindData::Flags::NONE;

			const pxr::GfVec3f& instPos = instancer_rest_positions[i];

			const neighbour_search::KDTree<float, 3>::ReturnType nearest_point = deformer_restpoints_kdtree.findNearestNeighbour(instPos);
			uint32_t prim_id = UsdGeomMeshFaceAdjacency::kInvalidID;

			// try to find prim we can project instance point on
			const std::vector<uint32_t> candidate_prim_indices = pAdjacency->getNeighborPrims(nearest_point.first);

			// try to project on prims sharing closest point
			for(uint32_t candidate_prim_id: candidate_prim_indices) {
				const uint32_t prim_vertex_count = pAdjacency->getPrimVertexCount(candidate_prim_id);
				if(prim_vertex_count == 3) {
					const pxr::GfVec3f& p0 = mesh_rest_positions[pAdjacency->getPrimVertex(candidate_prim_id, 0)];
					const pxr::GfVec3f& p1 = mesh_rest_positions[pAdjacency->getPrimVertex(candidate_prim_id, 1)];
					const pxr::GfVec3f& p2 = mesh_rest_positions[pAdjacency->getPrimVertex(candidate_prim_id, 2)];
					if(getTriUV(instPos, p0, p1, p2, bind.u, bind.v)) {
						prim_id = candidate_prim_id;
						break;
					}
				} else if(prim_vertex_count == 4) {
					const pxr::GfVec3f& p0 = mesh_rest_positions[pAdjacency->getPrimVertex(candidate_prim_id, 0)];
					const pxr::GfVec3f& p1 = mesh_rest_positions[pAdjacency->getPrimVertex(candidate_prim_id, 1)];
					const pxr::GfVec3f& p2 = mesh_rest_positions[pAdjacency->getPrimVertex(candidate_prim_id, 2)];
					const pxr::GfVec3f& p3 = mesh_rest_positions[pAdjacency->getPrimVertex(candidate_prim_id, 3)];
					if(getQuadUV(instPos, p0, p1, p2, p3, bind.u, bind.v)) {
						prim_id = candidate_prim_id;
						break;
					}
				} else {
					DLOG_ERR << "Unsupported deformer mesh prim " << candidate_prim_id << " vertex count " << prim_vertex_count << "!";
					continue;
				}
			}

			// if projection test failed we pick just best prim based on proximity and orientation
			if(prim_id == UsdGeomMeshFaceAdjacency::kInvalidID) {
				// start search with prims sharing closest point
				prim_id = pAdjacency->findBestPrimOriented(instPos, nearest_point.first, mesh_rest_positions);
			}

			const uint32_t prim_vertex_count = pAdjacency->getPrimVertexCount(prim_id);

			if(prim_vertex_count < 3 || prim_vertex_count > 4) {
				DLOG_ERR << "Unsupported deformer mesh prim " << prim_id << "! Should be quad or triangle."; 
				continue;
			}

			pxr::GfVec3f surfacePoint = instPos; // safety
			pxr::GfVec3f du;
			pxr::GfVec3f dv;

			const bool is_quad = prim_vertex_count == 4;

			bind.point_indices[0] = pAdjacency->getPrimVertex(prim_id, 0);
			bind.point_indices[1] = pAdjacency->getPrimVertex(prim_id, 1);
			bind.point_indices[2] = pAdjacency->getPrimVertex(prim_id, 2);
			bind.point_indices[3] = is_quad ? pAdjacency->getPrimVertex(prim_id, 3) : PointInstancerDeformerData::PointBindData::kInvalidPointID;

			static const pxr::GfVec3f sZeroPoint(0.0, 0.0, 0.0); // just for code logic
			const pxr::GfVec3f& p0 = mesh_rest_positions[pAdjacency->getPrimVertex(prim_id, 0)];
			const pxr::GfVec3f& p1 = mesh_rest_positions[pAdjacency->getPrimVertex(prim_id, 1)];
			const pxr::GfVec3f& p2 = mesh_rest_positions[pAdjacency->getPrimVertex(prim_id, 2)];
			const pxr::GfVec3f& p3 = is_quad ? mesh_rest_positions[pAdjacency->getPrimVertex(prim_id, 3)] : sZeroPoint;

			bool is_projected_inside = false;

			if(is_quad) {
				// Prim is quad
				is_projected_inside = getQuadUV(instPos, p0, p1, p2, p3, bind.u, bind.v);

				bind.u = std::max(0.0f, std::min(1.0f, bind.u));
				bind.v = std::max(0.0f, std::min(1.0f, bind.v));

				du = (p3 - p0) * (1.0f - bind.u) + (p2 - p1) * bind.u;
    			dv = (p1 - p0) * (1.0f - bind.v) + (p2 - p3) * bind.v;

    			// Surface point
    			float w0 = (1.0f - bind.u) * (1.0f - bind.v);
    			float w1 = bind.u * (1.0f - bind.v);
    			float w2 = bind.u * bind.v;
    			float w3 = (1.0f - bind.u) * bind.v;
   				surfacePoint = (p0 * w0) + (p1 * w1) + (p2 * w2) + (p3 * w3);
    		} else {
				// prim is triangle
				is_projected_inside = getTriUV(instPos, p0, p1, p2, bind.u, bind.v);

				barycentrics_clamp_to_triangle(bind.u, bind.v);

				du = p1 - p0; // edge01
    			dv = p2 - p0; // edge02
    			surfacePoint = p0;
    		}

			static const float eps = 1e-12;
			pxr::GfVec3f raw_normal = pxr::GfCross(du, dv);

			if (pxr::GfDot(du, du) <= eps || pxr::GfDot(raw_normal, raw_normal) <= eps) {
				DLOG_ERR << "Degenerate surface frame on primitive " << prim_id << " for point " << i;
				continue;
			}

			bind.restNormal = raw_normal.GetNormalized();

    		// barycentric test
			if (is_projected_inside || ignore_outside_test) {
		        // point projected inside primitive
		        bind.edge_id = -1;

		        projected_inside_count++;

		        // build a skew-capable matrix where X and Y are raw edge vectors
		        pxr::GfMatrix4f localToWorld;
		        localToWorld.SetIdentity();
		        localToWorld.SetRow3(0, du); // Row 0 = X basis vector
		        localToWorld.SetRow3(1, dv); // Row 1 = Y basis vector
		        localToWorld.SetRow3(2, bind.restNormal); // Row 2 = Z basis vector
		        localToWorld.SetRow3(3, surfacePoint); // Row 3 = Translation Origin

		        bind.restTangent = du;
		        bind.restBinormal = dv;
		        bind.localPos = localToWorld.GetInverse().Transform(instPos);
		    
		    } else {
			    // point projected outside primitive. bind to edge
			    auto closestPointOnSegment = [](const pxr::GfVec3f& p, const pxr::GfVec3f& a, const pxr::GfVec3f& b, float& t) {
			        pxr::GfVec3f ab = b - a;
			        float lenSq = pxr::GfDot(ab, ab);
			        t = (lenSq > 1e-6f) ? pxr::GfDot(p - a, ab) / lenSq : 0.0f;
			        t = std::max(0.0f, std::min(1.0f, t));
			        return a + t * ab;
			    };

			    auto isProjectedOutsideSegment = [](const pxr::GfVec3f& p, const pxr::GfVec3f& a, const pxr::GfVec3f& b) {
			        pxr::GfVec3f ab = b - a;
			        float lenSq = pxr::GfDot(ab, ab);
			        float t = (lenSq > 1e-6f) ? pxr::GfDot(p - a, ab) / lenSq : 0.0f;
			        return(t < 0.0f || t > 1.0f);    
			    };

			    float t0, t1, t2, t3;
			    pxr::GfVec3f cp0, cp1, cp2, cp3;
			    float sqdist0, sqdist1, sqdist2, sqdist3;

			    pxr::GfVec3f origin, axisX;

			    bool is_outside_edge = false;
			    if(is_quad) {
			    	// Quad
			    	cp0 = closestPointOnSegment(surfacePoint, p0, p1, t0);
					cp1 = closestPointOnSegment(surfacePoint, p1, p2, t1);
					cp2 = closestPointOnSegment(surfacePoint, p2, p3, t2);
					cp3 = closestPointOnSegment(surfacePoint, p3, p0, t3);

					sqdist0 = lengthSquared(surfacePoint - cp0);
					sqdist1 = lengthSquared(surfacePoint - cp1);
					sqdist2 = lengthSquared(surfacePoint - cp2);
					sqdist3 = lengthSquared(surfacePoint - cp3);

			    	if (sqdist0 <= sqdist1 && sqdist0 <= sqdist2 && sqdist0 <= sqdist3) {
				        bind.edge_id = 0; origin = p0; axisX = p1 - p0;   // Edge 01
				        is_outside_edge = isProjectedOutsideSegment(instPos, p0, p1);
				    } else if (sqdist1 <= sqdist0 && sqdist1 <= sqdist2 && sqdist1 <= sqdist3) {
				        bind.edge_id = 1; origin = p1; axisX = p2 - p1;   // Edge 12
				        is_outside_edge = isProjectedOutsideSegment(instPos, p1, p2);
				    } else if (sqdist2 <= sqdist0 && sqdist2 <= sqdist1 && sqdist2 <= sqdist3) {
				        bind.edge_id = 2; origin = p2; axisX = p3 - p2;   // Edge 23
				        is_outside_edge = isProjectedOutsideSegment(instPos, p2, p3);
				    } else {
				        bind.edge_id = 3; origin = p3; axisX = p0 - p3;   // Edge 30
				    	is_outside_edge = isProjectedOutsideSegment(instPos, p3, p0);
				    }
			    } else {
			    	// Triangle
			    	float planeDist = pxr::GfDot(instPos - p0, bind.restNormal);
    				pxr::GfVec3f surfacePoint = instPos - (bind.restNormal * planeDist);

					cp0 = closestPointOnSegment(surfacePoint, p0, p1, t0);
					cp1 = closestPointOnSegment(surfacePoint, p1, p2, t1);
					cp2 = closestPointOnSegment(surfacePoint, p2, p0, t2);

					sqdist0 = lengthSquared(surfacePoint - cp0);
					sqdist1 = lengthSquared(surfacePoint - cp1);
					sqdist2 = lengthSquared(surfacePoint - cp2);

				    if (sqdist0 <= sqdist1 && sqdist0 <= sqdist2) {
				        bind.edge_id = 0; origin = p0; axisX = p1 - p0;   // Edge 01
				        is_outside_edge = isProjectedOutsideSegment(instPos, p0, p1);
				    } else if (sqdist1 <= sqdist0 && sqdist1 <= sqdist2) {
				        bind.edge_id = 1; origin = p1; axisX = p2 - p1;   // Edge 12
				        is_outside_edge = isProjectedOutsideSegment(instPos, p1, p2);
				    } else {
				        bind.edge_id = 2; origin = p2; axisX = p0 - p2;   // Edge 20
				    	is_outside_edge = isProjectedOutsideSegment(instPos, p2, p0);
				    }
				}

				if(is_outside_edge) {
					bind.flags |= PointBindData::Flags::OUTSIDE;
					axisX = axisX.GetNormalized();
				}

			    // Rigid Orthonormal Frame relative to the active edge
			    pxr::GfVec3f normalY = pxr::GfCross(axisX, bind.restNormal).GetNormalized();
			    
			    pxr::GfMatrix4f localToWorld;
				localToWorld.SetIdentity();
				localToWorld.SetRow3(0, axisX);   // Row 0 = Scales dynamically with the edge length
				localToWorld.SetRow3(1, normalY);  // Row 1 = Rigid absolute distance outward from edge
				localToWorld.SetRow3(2, bind.restNormal);   // Row 2 = Rigid absolute distance from face plane
				localToWorld.SetRow3(3, origin);   // Row 3 = Translation Origin

				bind.restTangent = axisX;
		        bind.restBinormal = normalY;
			    bind.localPos = localToWorld.GetInverse().Transform(instPos);
			}
		}
	};

	DLOG_INF << "Binding instances using Simple method.";
    
    if(multi_threaded) {
    	BS::multi_future<void> blocks = mPool.submit_blocks(0, instances_count, func);
		blocks.wait();
	} else {
		func(0, instances_count);
	}

	mpPhantomTrimeshData->setValid(true);

	return true;
}

void PointInstancerDeformer::drawDebugGeometry(pxr::UsdTimeCode time_code, const PointsList* pDeformedPoints) {
	assert(pDeformedPoints);
	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	const auto& pointBinds = mpPointInstancerDeformerData->mPointBinds;
	assert(pDeformedPoints->size() == pointBinds.size() && "PointInstancerDeformer::drawDebugGeometry(...) deformed points and bindings count mismatch!!!");

	const MeshContainer* pDeformerMeshContainer = mpDeformerMeshContainer.get();
	const MeshContainer::ContainerType& mesh_live_positions = pDeformerMeshContainer->getLivePositions();

	if(!mpDebugGeo) {
		mpDebugGeo = DebugGeo::create(getName());
	} else {
		mpDebugGeo->clear();
	}

	const MeshContainer::ContainerType& mesh_rest_positions = mpDeformerMeshContainer->getRestPositions();

	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> closest_deformer_points(3);
	neighbour_search::KDTree<float, 3> deformer_kdtree = neighbour_search::KDTree<float, 3>(mesh_live_positions, false /* no threads */);

	const auto* pOutPoints = pDeformedPoints->points();
	const auto& instancer_rest_positions = mpInstancerContainer->getRestInstancePoints();


	LOG_DBG << "Trimesh face count is " << pPhantomTrimesh->getFaceCount();

	for(size_t i = 0; i < mesh_live_positions.size(); ++i) {
		const pxr::GfVec3f& vtxNormal = mLiveVertexNormals[i];
		DebugGeo::Line lvtxN(mesh_live_positions[i], mesh_live_positions[i] - vtxNormal * 0.5);
		lvtxN.setColor({0.0, 0.1, 0.0}, {0.0, 0.1, 0.0});
		lvtxN.setWidth(0.01);
		mpDebugGeo->addLine(lvtxN);
	}

	for(size_t i = 0; i < pointBinds.size(); ++i) {

		const PointInstancerDeformerData::PointBindData& bind = pointBinds[i];
		const pxr::GfVec3f& pt = pOutPoints[i];

		assert(bind.isValid());
		if(!bind.isValid()) continue;
		
		const bool is_quad_bound = bind.isQuadBound();

		static const pxr::GfVec3f sZeroPoint(0.0, 0.0, 0.0);
		const pxr::GfVec3f& p0 = mesh_live_positions[bind.point_indices[0]];
		const pxr::GfVec3f& p1 = mesh_live_positions[bind.point_indices[1]];
		const pxr::GfVec3f& p2 = mesh_live_positions[bind.point_indices[2]];
		const pxr::GfVec3f& p3 = is_quad_bound ? mesh_live_positions[bind.point_indices[3]] : sZeroPoint;

		pxr::GfVec3f du, dv, normal;

		if(is_quad_bound) {
			// quad bound
			du = (p3 - p0) * (1.0f - bind.u) + (p2 - p1) * bind.u;
    		dv = (p1 - p0) * (1.0f - bind.v) + (p2 - p3) * bind.v;
		} else {
			// Triangle bound
			du = p1 - p0;
    		dv = p2 - p0;
		}

		normal = pxr::GfCross(du, dv).GetNormalized();

		// Surface point
		pxr::GfVec3f surfacePoint;
		if(is_quad_bound) {
	    	float w0 = (1.0f - bind.u) * (1.0f - bind.v);
	    	float w1 = bind.u * (1.0f - bind.v);
	    	float w2 = bind.u * bind.v;
	    	float w3 = (1.0f - bind.u) * bind.v;
	   		surfacePoint = (p0 * w0) + (p1 * w1) + (p2 * w2) + (p3 * w3);
	   	} else {
	   		surfacePoint = ((1.0 - bind.u - bind.v) * p0) + (bind.u * p1) + (bind.v * p2);
	   	}

  		DebugGeo::Pt sP(surfacePoint);
		sP.setColor({1.0, 0.0, 0.0});
		sP.setWidth(0.1);
		mpDebugGeo->addPoint(sP);

		DebugGeo::Line edgeN(surfacePoint, surfacePoint + normal * 0.5);
		edgeN.setColor({0.0, 1.0, 0.0}, {0.0, 1.0, 0.0});
		edgeN.setWidth(0.01);
		mpDebugGeo->addLine(edgeN);

		DebugGeo::Line lineDU(surfacePoint, surfacePoint + du.GetNormalized() * 0.5);
		lineDU.setColor({1.0, 0.0, 0.0});
		lineDU.setWidth(0.01);
		mpDebugGeo->addLine(lineDU);

		DebugGeo::Line lineDV(surfacePoint, surfacePoint + dv.GetNormalized() * 0.5);
		lineDV.setColor({0.0, 1.0, 0.0});
		lineDV.setWidth(0.01);
		mpDebugGeo->addLine(lineDV);

		if(bind.edge_id < 0) {
			/*
			// Prim surface bound
			DebugGeo::Line lp0(pt, p0);
			lp0.setColor({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
			lp0.setWidth(0.01);
			mpDebugGeo->addLine(lp0);

			DebugGeo::Line lp1(pt, p1);
			lp1.setColor({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
			lp1.setWidth(0.01);
			mpDebugGeo->addLine(lp1);

			DebugGeo::Line lp2(pt, p2);
			lp2.setColor({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
			lp2.setWidth(0.01);
			mpDebugGeo->addLine(lp2);


			if(is_quad_bound) {
				DebugGeo::Line lp3(pt, p3);
				lp3.setColor({1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
				lp3.setWidth(0.01);
				mpDebugGeo->addLine(lp3);
			}
			*/

		} else {
			// Prim edge bound
			const uint32_t edges_count = is_quad_bound ? 4 : 3;
			const pxr::GfVec3f& edge_pA = mesh_live_positions[bind.point_indices[bind.edge_id]];
			const pxr::GfVec3f& edge_pB = mesh_live_positions[bind.point_indices[(bind.edge_id + 1) % edges_count]];

			DebugGeo::Line lE(edge_pA, edge_pB);
			lE.setColor({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});
			lE.setWidth(0.02);
			mpDebugGeo->addLine(lE);

			DebugGeo::Line lpE(pt, surfacePoint);

			if(bind.isOutside()) {
				lpE.setColor({0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
			} else {
				lpE.setColor({1.0, 0.0, 0.0}, {0.0, 0.0, 1.0});
			}
			lpE.setWidth(0.02);
			mpDebugGeo->addLine(lpE);

			// rebuild edge-locked orthonormal matrix
	        pxr::GfVec3f origin, axisX;
	        if(is_quad_bound) {
	        	// quad
		        if (bind.edge_id == 0) {
		            origin = p0; axisX = p1 - p0;
		        } else if (bind.edge_id == 1) {
		            origin = p1; axisX = p2 - p1;
		        } else if (bind.edge_id == 2) {
		            origin = p2; axisX = p3 - p2;
		        }else {
		            origin = p3; axisX = p0 - p3;
		        }
		    } else {
		    	// triangle
		    	if (bind.edge_id == 0) {
		            origin = p0; axisX = p1 - p0;
		        } else if (bind.edge_id == 1) {
		            origin = p1; axisX = p2 - p1;
		        } else {
		            origin = p2; axisX = p0 - p2;
		        }
		    }

	        pxr::GfVec3f normalY = pxr::GfCross(axisX, normal).GetNormalized();

	        // X axis (scaling one)
	        DebugGeo::Line edgeX(surfacePoint, surfacePoint + axisX);
			edgeX.setColor({1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
			edgeX.setWidth(0.01);
			mpDebugGeo->addLine(edgeX);

			DebugGeo::Line normY(surfacePoint, surfacePoint + normalY);
			normY.setColor({0.0, 0.0, 1.0}, {0.0, 0.0, 1.0});
			normY.setWidth(0.01);
			mpDebugGeo->addLine(normY);
		}
	}

	mpDebugGeo->build("/debugLinks", mInstancerGeoPrimHandle.getStage());
}

PointInstancerDeformer::~PointInstancerDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston