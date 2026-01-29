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
		default:
			std::cerr << "Unimplemented bind mode " << to_string(bind_mode) << " !!!" << std::endl;
			break;
	}

	return false;
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
		for(size_t i = start; i < end; ++i) {
			const auto& bind = pointBinds[i];
			if(bind.encoded_id == PointBindData::kInvalid) continue;

			const auto& tetra = pPhantomTrimesh->getTetrahedron(bind.encoded_id);
			points[i] = pPhantomTrimesh->getPointPositionFromBarycentricTetrahedronLiveCoords(tetra, bind.vec[0], bind.vec[1], bind.vec[2], (1 - (bind.vec[0] + bind.vec[1] + bind.vec[2])));
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

	if(guideIDSPrimVar.GetAttr().Get(&mGuideIndices, rest_time_code)) {
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
	neighbour_search::KDTree<float, 3> restpoints_kdtree = buildRestPositionsKDTree(multi_threaded);

	const size_t curves_count = mpCurvesContainer->getCurvesCount();

	assert(mpGuideCurvesDeformerData);
	auto& pointBinds = mpGuideCurvesDeformerData->mPointBinds;
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	std::atomic<size_t> total_points = 0;
	std::atomic<size_t> bound_points = 0;
	std::atomic<size_t> proximity_bound_points = 0;
	std::atomic<size_t> unboud_points = 0;
	
	auto bind_func = [&](const std::size_t start, const std::size_t end) {
    	if(multi_threaded) {
    		const std::optional<std::size_t> thread_index = BS::this_thread::get_index();
    		dbg_printf("Binding curves from %zu to %zu by thread id #%zu\n", start, end, *thread_index);
    	}

    	std::vector<neighbour_search::KDTree<float, 3>::ReturnType> sorted_tetra_centorids; // sorted tetrahedron centroids for brute force search
    	sorted_tetra_centorids.reserve(tetrahedrons.size());

		for(size_t curve_idx = start; curve_idx < end; ++curve_idx) {
    		const PxrCurvesContainer::CurveDataPtr curve_data_ptr = mpCurvesContainer->getCurveDataPtr(curve_idx);

    		const uint32_t curve_vertices_count = static_cast<uint32_t>(curve_data_ptr.first);

        	const pxr::GfVec3f& curve_root_pt = mpCurvesContainer->getCurveRootPoint(curve_idx);
        	const uint32_t curve_vertex_offset = mpCurvesContainer->getCurveVertexOffset(curve_idx);

        	for(uint32_t i = 0; i < curve_vertices_count; ++i) {
        		total_points++;

        		bool bound = false;
        		const size_t curr_point_index = curve_vertex_offset + i;
        		auto& bind = pointBinds[curr_point_index];
        		bind.encoded_id = PointBindData::kInvalid;

        		const pxr::GfVec3f curr_pt = curve_root_pt + *(curve_data_ptr.second + i); 

        		//findKNearestNeighboursWithinRadiusSquared(const PointType &target, std::size_t k, CoordinateType radius_squared, std::vector<ReturnType> &result)

        		// First try to find tetrahedron is connected to cleset point in point cloud
        		const neighbour_search::KDTree<float, 3>::ReturnType nearest_pt = restpoints_kdtree.findNearestNeighbour(curr_pt);
        		const size_t point_connected_tetras_count = pPhantomTrimesh->getPointConnectedTetrahedronsCount(nearest_pt.first);

        		for(size_t lti = 0; lti < point_connected_tetras_count; ++lti) {
        			uint32_t tetra_index = pPhantomTrimesh->getPointConnectedTetrahedronIndex(nearest_pt.first, lti);

        			float x;
        			pPhantomTrimesh->barycentricTetrahedronRestCoords(tetra_index, curr_pt, bind.vec[0], bind.vec[1], bind.vec[2], x);

        			if(bind.vec[0] < 0.0f || bind.vec[1] < 0.0f || bind.vec[2] < 0.0f || x < 0.0f ||
        			   bind.vec[0] > 1.0f || bind.vec[1] > 1.0f || bind.vec[2] > 1.0f || x > 1.0f ) {
        				// neighbour test failed;
        				continue;
        			}

        			// bound by nerest point cloud vertex;
        			bind.encoded_id = tetra_index;
        			bound_points++;
        			break;
        		}

        		// If point os not boud by previous method we test it against closest tetraheron centroid
        		if(bind.encoded_id == PointBindData::kInvalid) {
        			const neighbour_search::KDTree<float, 3>::ReturnType nearest_centroid = centroids_kdtree.findNearestNeighbour(curr_pt);
					
					float x;
					pPhantomTrimesh->barycentricTetrahedronRestCoords(nearest_centroid.first, curr_pt, bind.vec[0], bind.vec[1], bind.vec[2], x);
        			if(!(bind.vec[0] < 0.0f || bind.vec[1] < 0.0f || bind.vec[2] < 0.0f || x < 0.0f ||
        			   	bind.vec[0] > 1.0f || bind.vec[1] > 1.0f || bind.vec[2] > 1.0f || x > 1.0f )) {
        				// bound by nearest tetra centroid
        				bind.encoded_id = nearest_centroid.first;
        				bound_points++;
        			}
        		}

        		// Brute force search for contaning centroid
        		if(bind.encoded_id == PointBindData::kInvalid) {
        			const bool sort = false;
        			centroids_kdtree.findAllNearestNeighboursWithinRadiusSquared(curr_pt, sorted_tetra_centorids, sort);
					
					float x;
        			for(const auto& centroid: sorted_tetra_centorids) {
						pPhantomTrimesh->barycentricTetrahedronRestCoords(centroid.first, curr_pt, bind.vec[0], bind.vec[1], bind.vec[2], x);
        				if(!(bind.vec[0] < 0.0f || bind.vec[1] < 0.0f || bind.vec[2] < 0.0f || x < 0.0f ||
        			   	    bind.vec[0] > 1.0f || bind.vec[1] > 1.0f || bind.vec[2] > 1.0f || x > 1.0f)) {
        				   	// bound by brute force search
        					bind.encoded_id = centroid.first;
        					bound_points++;
        					break;
        				}
        			}
        		}

        		if(bind.encoded_id == PointBindData::kInvalid) {
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
	dbg_printf("Proximity bound points: %zu\n", proximity_bound_points.load());
	dbg_printf("Unbound points: %zu\n", unboud_points.load());

	return true;
}

bool GuideCurvesDeformer::buildDeformerDataNTBMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!mGuidesSkinPrimAttrName.empty()) {

	}

	if(mGuidesSkinGeoPrimHandle.isValid()) {
		mpGuideCurvesDeformerData->setSkinPrimPath(mGuidesSkinGeoPrimHandle.getName());
	} else {
		mpGuideCurvesDeformerData->setSkinPrimPath("");
	}
}

bool GuideCurvesDeformer::buildDeformerDataAngleMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	const size_t guides_count = mpGuideCurvesContainer->getCurvesCount();

	std::vector<std::mutex> kdtrees_mutexes(guides_count);  // protects kdree initialisation
	std::vector<std::unique_ptr<neighbour_search::KDTree<float, 3>>> kdtrees(guides_count);

	const size_t curves_count = mpCurvesContainer->getCurvesCount();

	auto& pointBinds = mpGuideCurvesDeformerData->mPointBinds;
	pointBinds.resize(mpCurvesContainer->getTotalVertexCount());

	auto func = [&](const size_t curve_index) {
		assert(mGuideIndices[curve_index] >= 0);
    	const uint32_t guide_id = (uint32_t)mGuideIndices[curve_index];
    	assert(guide_id < guides_count);
        const neighbour_search::KDTree<float, 3>* pKDTree;
        
        {
        	// build guide kdtree if needed
        	const std::lock_guard<std::mutex> lock(kdtrees_mutexes[guide_id]);
        	if(!kdtrees[guide_id]) {
        		kdtrees[guide_id] = std::make_unique<neighbour_search::KDTree<float, 3>>(mpGuideCurvesContainer->getRestCurvePoints(), false /*threaded*/);
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
			
			const uint32_t guide_vertex_id = nearest_pt.first;

			const pxr::GfVec3f guide_pt = mpGuideCurvesContainer->getGuideRestPoint(guide_id, guide_vertex_id);
			bind.vec = curr_pt - guide_pt;
			bind.encoded_id = GuideCurvesDeformerData::PointBindData::encodeID_modeANGLE(guide_id, guide_vertex_id);
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