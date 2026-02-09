#include "simple_profiler.h"
#include "guide_curves_deformer.h"

#include "common.h"
#include "kdtree.hpp"
#include "geometry_tools.h"

#include <pxr/base/gf/matrix4f.h>

#include <random>
#include <algorithm>
#include <mutex>
#include <atomic>


namespace Piston {

GuideCurvesDeformer::GuideCurvesDeformer(const std::string& name): BaseCurvesDeformer(BaseCurvesDeformer::Type::GUIDES, name) {
	dbg_printf("GuideCurvesDeformer::GuideCurvesDeformer(%s)\n", name.c_str());
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

void GuideCurvesDeformer::setGuideIDPrimAttrName(const std::string& name) {
	if(mGuideIDPrimAttrName == name) return;
	mGuideIDPrimAttrName = name;
	makeDirty();

	dbg_printf("Guide ID attribute name is set to: %s\n", name.c_str());
}

void GuideCurvesDeformer::setGuidesSkinPrimAttrName(const std::string& name) {
	if(mGuidesSkinPrimAttrName == name) return;
	mGuidesSkinPrimAttrName = name;
	makeDirty();

	dbg_printf("Guide skin prim attribute name is set to: %s\n", name.c_str());
}

void GuideCurvesDeformer::setGuidesSkinPrimRestAttrName(const std::string& name) {
	if(mGuidesSkinPrimRestAttrName == name) return;
	mGuidesSkinPrimRestAttrName = name;
	makeDirty();

	dbg_printf("Guide skin prim rest attribute name is set to: %s\n", name.c_str());
}

void GuideCurvesDeformer::setGuidesSkinPrim(const pxr::UsdPrim& geoPrim) {
	if(mGuidesSkinGeoPrimHandle == geoPrim) return;
	
	if(!validateDeformerGeoPrim(geoPrim)) {
		mGuidesSkinGeoPrimHandle.clear();
		std::cerr << "Invalid guides skin geometry prim " <<  geoPrim.GetPath().GetText() << " type!" << std::endl;
		return;
	}

	mGuidesSkinGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	dbg_printf("Guides skin geometry prim is set to: %s\n", mGuidesSkinGeoPrimHandle.getPath().GetText());
}

bool GuideCurvesDeformer::deformImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, false, time_code);
}

bool GuideCurvesDeformer::deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	return __deform__(points, true, time_code);
}

bool GuideCurvesDeformer::__deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code) {
	if(!mpGuideCurvesContainer->update(mDeformerGeoPrimHandle, time_code)) {
		std::cerr << "Error updating guide curves from prim" << mDeformerGeoPrimHandle.getPath().GetText() << " !" << std::endl;
		return false;
	}

	assert(points.size() == mpGuideCurvesDeformerData->getPointBinds().size());
	const auto bind_mode = getBindMode();

	switch(bind_mode) {
		case BindMode::SPACE:
			return deformImpl_SpaceMode(multi_threaded, points, time_code);
		case BindMode::ANGLE:
			return deformImpl_AngleMode(multi_threaded, points, time_code);
		case BindMode::NTB:
			return deformImpl_NTBMode(multi_threaded, points, time_code);
		default:
			std::cerr << "Unimplemented bind mode " << to_string(bind_mode) << " !!!" << std::endl;
			break;
	}

	return false;
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

	const auto total_guides_count = mpGuideCurvesContainer->getCurvesCount();
	const auto& guides_live_points = mpGuideCurvesContainer->getLiveCurvePoints();

	const auto& pointBinds = mpGuideCurvesDeformerData->getPointBinds();
	const std::vector<uint32_t>& root_triface_binds = mpGuideCurvesDeformerData->getRootTrifaceBinds();

	std::vector<NTBFrame> live_guide_frames(points.size());

	auto frame_func = [&](const std::size_t start, const std::size_t end) {
		const pxr::VtArray<pxr::GfVec3f>& skin_live_points = pSkinPhantomTrimesh->getLivePositions();

		for(auto guide_id = start; guide_id < end; ++guide_id) {
			assert(guide_id < total_guides_count);

			const size_t guide_vertex_offset = mpGuideCurvesContainer->getCurveVertexOffset(guide_id);
			const size_t curve_points_count = mpGuideCurvesContainer->getCurveVertexCount(guide_id);
			const pxr::GfVec3f* pCurveRootPt = guides_live_points.data() + guide_vertex_offset;
			const pxr::GfVec3f& root_tangent = *(pCurveRootPt + 1) - *pCurveRootPt; 

			const uint32_t face_id = root_triface_binds[guide_id];
			const PhantomTrimesh::TriFace& face = pSkinPhantomTrimesh->getFace(face_id);

			const pxr::GfVec3f up_vector = pxr::GfGetNormalized(skin_live_points[face.indices[0]] - skin_live_points[face.indices[1]]);
			// TODO: build proper root normal form this up vector

			std::vector<NTBFrame>::iterator it_begin = live_guide_frames.begin() + guide_vertex_offset;
			std::vector<NTBFrame>::iterator it_end = it_begin + curve_points_count;
			buildRotationMinimizingFrames(pCurveRootPt, curve_points_count, root_tangent, up_vector, it_begin, it_end);
		}
	};

	auto func = [&](const std::size_t start, const std::size_t end) {
		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			if(bind.encoded_id == PointBindData::kInvalid) continue;

			uint32_t frame_id;
			uint8_t axis_id;
			bind.decodeID_modeNTB(frame_id, axis_id);
			assert(frame_id < live_guide_frames.size());
			const std::array<float, 3>& barycentricCoord = bind.getData();

			points[i] = guides_live_points[frame_id] + live_guide_frames[frame_id] * barycentricCoord;
		}
	};

	if(multi_threaded) {
		mPool.detach_blocks(0u, total_guides_count, frame_func);
		mPool.wait();
		mPool.detach_blocks(0u, pointBinds.size(), func);
		mPool.wait();
	} else {
		frame_func(0u, total_guides_count);
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
					dbg_printf("24bit\n");
					continue;
				} else {
					bind.getData(u, v, w);
					x = 1.f - (u + v + w);
				}
				points[i] = pPhantomTrimesh->getPointPositionFromBarycentricTetrahedronLiveCoords(tetra, u, v, w, x);
			} else {
				// bound to triface
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

bool GuideCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	assert(mpGuideCurvesDeformerData);
	assert(mpGuideCurvesContainer);

	if(!mpGuideCurvesContainer->init(mDeformerGeoPrimHandle, rest_time_code)) {
		std::cerr << "Error initializing guide curves container !" << std::endl;
		return false;
	}

	const auto total_guides_count = mpGuideCurvesContainer->getCurvesCount();
	const auto total_curves_count = mpCurvesContainer->getCurvesCount();

	if(mGuideIDPrimAttrName.empty() || getBindMode() == BindMode::SPACE) {
		return buildDeformerDataSpaceMode(rest_time_code, multi_threaded);
	} 

	pxr::UsdGeomPrimvarsAPI curvesPrimvarsApi = mCurvesGeoPrimHandle.getPrimvarsAPI();
	pxr::UsdGeomPrimvar guideIDSPrimVar = curvesPrimvarsApi.GetPrimvar(pxr::TfToken(mGuideIDPrimAttrName));

	if(!guideIDSPrimVar) {
		std::cerr << "Error getting guide(clump) id prmitive attribute \"" << mGuideIDPrimAttrName << "\" !" << std::endl;
		return false;
	}

	if(!guideIDSPrimVar.GetAttr().Get(&mGuideIndices, rest_time_code)) {
		std::cerr << "Error getting curves " << mCurvesGeoPrimHandle.getPath() << " \"" << mGuideIDPrimAttrName << "\" guide indices !" << std::endl;
		return false;
	}
	assert(mGuideIndices.size() == total_curves_count);

	// check guide indices are not out of range
	int max_guide_index = 0;
	for(const int idx: mGuideIndices) {
		max_guide_index = std::max(max_guide_index, idx);
	}

	if(max_guide_index >= total_guides_count) {
		std::cerr << "Curves " << mCurvesGeoPrimHandle.getPath() << " guide indices are out of range !" << std::endl; 
		return false;
	}

	switch(getBindMode()) {
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
			std::cerr << "Error building phantom mesh data!" << std::endl;
			return false;
		}
	}

	PhantomTrimesh* pPhantomTrimesh = mpGuidesPhantomTrimeshData->getTrimesh();
	assert(pPhantomTrimesh);

	if(!pPhantomTrimesh->hasTetrahedrons()) {
		if(!pPhantomTrimesh->buildTetrahedrons()) {
			std::cerr << "Error building tetrahedrons!" << std::endl;
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

	dbg_printf("Total points: %zu\n", total_points.load());
	dbg_printf("Bound points: %zu\n", bound_points.load());
	dbg_printf("Unbound points: %zu\n", unboud_points.load());

	return true;
}

bool GuideCurvesDeformer::buildDeformerDataNTBMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!mGuidesSkinPrimAttrName.empty()) {
		std::cout << "No skin prim attribute provided !" << std::endl;
	}

	if(mGuidesSkinGeoPrimHandle.isValid()) {
		mpGuideCurvesDeformerData->setSkinPrimPath(mGuidesSkinGeoPrimHandle.getName());
	} else {
		mpGuideCurvesDeformerData->setSkinPrimPath("");
	}

	return true;
}

bool GuideCurvesDeformer::buildDeformerDataAngleMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	const size_t guides_count = mpGuideCurvesContainer->getCurvesCount();

	std::vector<std::mutex> kdtrees_mutexes(guides_count);  // protects kdree initialisation
	std::vector<std::unique_ptr<neighbour_search::KDTree<float, 3>>> kdtrees(guides_count);

	const size_t curves_count = mpCurvesContainer->getCurvesCount();

	assert(curves_count == mGuideIndices.size());

	auto& pointBinds = mpGuideCurvesDeformerData->mPointBinds;
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	auto func = [&](const size_t curve_index) {
		assert(mGuideIndices[curve_index] >= 0);
    	const uint32_t guide_id = (uint32_t)mGuideIndices[curve_index];
    	assert(guide_id < guides_count);
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
		std::cerr << "Unable to build guides skin primtive data. No primitive \"" << mGuidesSkinGeoPrimHandle.getFullName() << "\" found!" << std::endl;
		return false;
	}

	if(!mDirty && mpSkinAdjacencyData && mpSkinPhantomTrimeshData && 
		mpSkinAdjacencyData->isValid() && 
		mpSkinPhantomTrimeshData->isValid()) {
		return true;
	}

	if(!mpSkinAdjacencyData) {
		mpSkinAdjacencyData = std::make_unique<SerializableUsdGeomMeshFaceAdjacency>();
	}

	if(!getReadJsonDataState() || !mGuidesSkinGeoPrimHandle.getDataFromBson(mpSkinAdjacencyData.get())) {
		if(!mpSkinAdjacencyData->buildInPlace(mGuidesSkinGeoPrimHandle)) {
			std::cerr << "Error building guides skin adjacency data!" << std::endl;
			return false;
		}
	}

	if(!mpSkinPhantomTrimeshData) {
		mpSkinPhantomTrimeshData = std::make_unique<SerializablePhantomTrimesh>();
	}

	if(!getReadJsonDataState() || !mGuidesSkinGeoPrimHandle.getDataFromBson(mpSkinPhantomTrimeshData.get())) {
		if(!mpSkinPhantomTrimeshData->buildInPlace(mGuidesSkinGeoPrimHandle, mGuidesSkinPrimRestAttrName)) {
			std::cerr << "Error building guides skin trimesh data!" << std::endl;
			return false;
		}
	}

	return true;
}

bool GuideCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(mpGuidesPhantomTrimeshData && !mDeformerGeoPrimHandle.writeDataToBson(mpGuidesPhantomTrimeshData.get())) {
		std::cerr << "Error writing " << mpGuidesPhantomTrimeshData->typeName() << " deformer data to json !" << std::endl;
		return false;
	}

	if(mpGuideCurvesDeformerData && !mCurvesGeoPrimHandle.writeDataToBson(mpGuideCurvesDeformerData.get())) {
		std::cerr << "Error writing " << mpGuideCurvesDeformerData->typeName() << " deformer data to json !" << std::endl;
		return false;
	}

	if(mpSkinAdjacencyData && mGuidesSkinGeoPrimHandle.writeDataToBson(mpSkinAdjacencyData.get())) {
		std::cerr << "Error writing " << mpSkinAdjacencyData->typeName() << " deformer data to json !" << std::endl;
		return false;
	}

	if(mpSkinPhantomTrimeshData && !mGuidesSkinGeoPrimHandle.writeDataToBson(mpSkinPhantomTrimeshData.get())) {
		std::cerr << "Error writing " << mpSkinPhantomTrimeshData->typeName() << " curves data to json !" << std::endl;
		return false;
	}

	return true;
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