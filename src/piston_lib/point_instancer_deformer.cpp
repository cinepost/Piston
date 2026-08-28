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

bool PointInstancerDeformer::deform_dbg(pxr::UsdTimeCode time_code, bool ignoreVelocities) {	
	return deform(time_code, false, ignoreVelocities);
}

bool PointInstancerDeformer::deform(pxr::UsdTimeCode time_code, bool multi_threaded, bool ignoreVelocities) {
	PROFILE("PointInstancerDeformer::deform");

	assert(mpPointInstancerDeformerData && mpPointInstancerDeformerData->isValid());
	assert(mpAdjacencyData);
	assert(mpDeformerMeshContainer);

	assert(mpPhantomTrimeshData);
	const auto* pPhantomTrimesh = mpPhantomTrimeshData->getTrimesh();

	if(!pPhantomTrimesh || !pPhantomTrimesh->isValid()) {
		return false;
	}

	buildVertexNormals(mpAdjacencyData->getAdjacencyFinal(), pPhantomTrimesh, mLiveVertexNormals, mpDeformerMeshContainer->getLivePositions(), (multi_threaded ? &mPool : nullptr));

	if(mShowDebugGeometry) {
		drawDebugSubdivDeformerGeometry(time_code);
	}

	bool result = false;

	assert(false);
	
	return result;
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
	const auto* pAdjacency = mpAdjacencyData->getAdjacencyFinal();
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

			std::vector<pxr::GfVec3f> rest_vertex_normals;
			buildVertexNormals(pAdjacency, pPhantomTrimesh, rest_vertex_normals, mpDeformerMeshContainer->getRestPositions(), (multi_threaded ? &mPool : nullptr));
			mLiveVertexNormals.resize(rest_vertex_normals.size());

			// Bind curve points
			DLOG_DBG << "Binding " << mpInstancerContainer->getInstanceCount() << " instancer points.";	

			bool result = false;
			auto threads_timer = Timer();
			threads_timer.start();

			// Build bind data
			switch(mpPointInstancerDeformerData->getBindMode()) {
				case BindMode::SIMPLE:
				default:
					result = buildDeformerData_SimpleMode(multi_threaded, rest_vertex_normals, rest_time_code);
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

bool PointInstancerDeformer::buildDeformerData_SimpleMode(bool multi_threaded, const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code) {
	return false;
}

PointInstancerDeformer::~PointInstancerDeformer() {
	PROFILE_PRINT();
}

} // namespace Piston