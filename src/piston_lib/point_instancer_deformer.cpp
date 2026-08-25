#include "simple_profiler.h"
#include "point_instancer_deformer.h"

#include "common.h"
#include "logging.h"
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

void PointInstancerDeformer::invalidateData(DeformerDataCache& cache) {
	BaseDeformer::invalidateData(cache);
	cache.invalidate(mpPointInstancerDeformerData);
}

bool PointInstancerDeformer::deformImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	PROFILE("PointInstancerDeformer::deformImpl");
	return __deform__(points, false, time_code);
}

bool PointInstancerDeformer::deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) {
	PROFILE("PointInstancerDeformer::deformMtImpl");
	return __deform__(points, true, time_code);
}

bool PointInstancerDeformer::__deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code) {
	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();

	if(!pPhantomTrimesh || !pPhantomTrimesh->isValid()) {
		return false;
	}

	assert(points.size() == mpPointInstancerDeformerData->getPointBinds().size());
	assert(mpAdjacencyData);
	assert(mpDeformerMeshContainer);

	buildVertexNormals(mpAdjacencyData->getAdjacency(), pPhantomTrimesh, mLiveVertexNormals, mpDeformerMeshContainer->getLivePositions(), (multi_threaded ? &mPool : nullptr));

	bool result = false;

	assert(false);
	
	return result;
}

static inline bool saturate(bool a) {
	return a < 0.f ? 0.f : (a > 1.f ? 1.f : a);
}

bool PointInstancerDeformer::writeJsonDataToPrimImpl() const {
	if(!BaseDeformer::writeJsonDataToPrimImpl()) {
		return false;
	}

	if(mpPointInstancerDeformerData && !mInstancerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpPointInstancerDeformerData.get())) {
		DLOG_ERR << "Error writing " << mpPointInstancerDeformerData->typeName() << " deformer data to json !";	
		return false;
	}
	return true;
}

bool PointInstancerDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	if(!BaseDeformer::buildDeformerDataImpl(rest_time_code, multi_threaded)) {
		return false;
	}

	// Data validity was checked in BaseDeformer::buildDeformerDataImpl()
	const auto* pAdjacency = mpAdjacencyData->getAdjacency();
	auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();

	DeformerDataCache& dataCache = DeformerDataCache::getInstance();
	bool deformer_data_created;
	if(!mpPointInstancerDeformerData) {
		mpPointInstancerDeformerData = dataCache.getOrCreateData<PointInstancerDeformerData>(this, {&mDeformerGeoPrimHandle, &mInstancerGeoPrimHandle}, rest_time_code, deformer_data_created);
		mpPointInstancerDeformerData->setBindMode(mBindMode);
	}

	if(deformer_data_created || !mpPointInstancerDeformerData->isValid() || !mpPhantomTrimeshData->isValid()) {
		if(!getReadJsonDataState() || !mInstancerGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpPointInstancerDeformerData.get())) {
			// Build deformer data in place if no json data present or not needed

			// First triangulate using simple "fan" triangulation
			const uint32_t src_mesh_face_count = pAdjacency->getFaceCount();

			for(uint32_t face_id = 0; face_id < src_mesh_face_count; ++face_id) {
				const uint32_t face_vertex_count = pAdjacency->getFaceVertexCount(face_id);
				
				if(face_vertex_count < 3 ) {
					DLOG_ERR << "Source mesh polygon " << face_id << " is invalid !!!";
					continue;
				}
				
				const uint32_t face_vertex_offset = pAdjacency->getFaceVertexOffset(face_id);

				switch(face_vertex_count) {
					case 3:
						pPhantomTrimesh->getOrCreateFaceID(
							pAdjacency->getFaceVertex(face_id, 0), 
							pAdjacency->getFaceVertex(face_id, 1),
							pAdjacency->getFaceVertex(face_id, 2)
						);
						break;
					default:
						for(uint32_t ii = 1; ii < (face_vertex_count - 1); ++ii) {
							pPhantomTrimesh->getOrCreateFaceID(
								pAdjacency->getFaceVertex(face_id, 0), 
								pAdjacency->getFaceVertex(face_id, ii % face_vertex_count),
								pAdjacency->getFaceVertex(face_id, (ii + 1) % face_vertex_count)
							);
						}
						break;
				}
			}

			const size_t tri_face_count = pPhantomTrimesh->getFaceCount();

			DLOG_DBG << src_mesh_face_count << " source mesh faces triangulated to " << tri_face_count << " triangles.";

			std::vector<pxr::GfVec3f> rest_vertex_normals;
			buildVertexNormals(pAdjacency, pPhantomTrimesh, rest_vertex_normals, mpDeformerMeshContainer->getRestPositions(), (multi_threaded ? &mPool : nullptr));
			mLiveVertexNormals.resize(rest_vertex_normals.size());

			// Bind curve points
			DLOG_DBG << "Binding " << mpInstacerContainer->getPointsCount() << " instancer points.";	

			bool result = false;
			auto threads_timer = Timer();
			threads_timer.start();

			// Build bind data
			switch(mpWrapCurvesDeformerData->getBindMode()) {
				case BindMode::SPACE:
					result = buildDeformerData_SpaceMode(multi_threaded, rest_vertex_normals, rest_time_code);
					break;
				default:
					result = buildDeformerData_DistMode(multi_threaded, rest_vertex_normals, rest_time_code);
					break;
			}

			auto& face_flags = pPhantomTrimesh->getFaceFlags(); 
			assert(face_flags.size() == pPhantomTrimesh->getFaceCount());

			for(const auto& bind: mpPointInstancerDeformerData->getPointBinds()) {
				if(bind.isValid()) face_flags[bind.face_id] = PhantomTrimesh::TriFace::Flags::Bound;
			}

			threads_timer.stop();
			if(multi_threaded) {
				DLOG_TRC << "PointInstancerDeformer::buildDeformerDataImpl() " << mPool.get_thread_count() << " threads finished in " << threads_timer.toString();
			} else {
				DLOG_TRC << "PointInstancerDeformer::buildDeformerDataImpl() finished in " << threads_timer.toString();
			}

			mpPointInstancerDeformerData->setValid(true);
		}
		mpPhantomTrimeshData->setValid(mpPointInstancerDeformerData->isValid());
	}

	return mpPointInstancerDeformerData->isValid();
}

PointInstancerDeformer::~PointInstancerDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston