#include "deformer_factory.h"
#include "base_mesh_curves_deformer.h"
#include "geometry_tools.h"
#include "pxr_points_lru_cache.h"
#include "deformer_data_cache.h"
#include "logging.h"

#include <thread>


namespace Piston {

BaseMeshCurvesDeformer::BaseMeshCurvesDeformer(const BaseCurvesDeformer::Type t, const std::string& name): BaseCurvesDeformer(t, name) {
	DLOG_TRC << "BaseMeshCurvesDeformer::BaseMeshCurvesDeformer()";
}

bool BaseMeshCurvesDeformer::validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	return isMeshGeoPrim(geoPrim);
}

bool BaseMeshCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(mpAdjacencyData && !mDeformerGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpAdjacencyData.get())) {
		DLOG_ERR << "Error writing " << mpAdjacencyData->typeName() << " deformer data to json !";
		return false;
	}

	if(mpPhantomTrimeshData && !mCurvesGeoPrimHandle.writeDataToBson(getDataPrimPath(), mpPhantomTrimeshData.get())) {
		DLOG_ERR << "Error writing " << mpPhantomTrimeshData->typeName() << " curves data to json !";
		return false;
	}
	return true;
}

bool BaseMeshCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded) {
	DeformerDataCache& dataCache = DeformerDataCache::getInstance();

	bool adjacency_data_created;
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

	bool trimesh_data_created;
	if(!mpPhantomTrimeshData) {
		mpPhantomTrimeshData = dataCache.getOrCreateData<SerializablePhantomTrimesh>(this, {&mDeformerGeoPrimHandle, &mCurvesGeoPrimHandle}, rest_time_code, trimesh_data_created);
	}

	// Get phantom mesh json data if present
	if(trimesh_data_created || !getReadJsonDataState() || !mCurvesGeoPrimHandle.getDataFromBson(getDataPrimPath(), mpPhantomTrimeshData.get())) {
		// Build in place if no json data present or not needed
		if(!mpPhantomTrimeshData->buildInPlace(mDeformerGeoPrimHandle, getDeformerRestAttrName())) {
			DLOG_ERR << "Error building phantom mesh data!";
			return false;
		}
	}
	
	assert(mpAdjacencyData);
	assert(mpPhantomTrimeshData);

	return true;
}

void BaseMeshCurvesDeformer::invalidateData(DeformerDataCache& cache) {
	//if(mpAdjacencyData && mpAdjacencyData->isValid()) cache.invalidate<SerializableUsdGeomMeshFaceAdjacency>(mDeformerGeoPrimHandle);
	//if(mpPhantomTrimeshData && mpPhantomTrimeshData->isValid()) cache.invalidate<SerializablePhantomTrimesh>({&mDeformerGeoPrimHandle, &mCurvesGeoPrimHandle});
	cache.invalidate(mpAdjacencyData);
	cache.invalidate(mpPhantomTrimeshData);
}

const std::string& BaseMeshCurvesDeformer::toString() const {
	static const std::string kBaseMeshDeformerString = "BaseMeshCurvesDeformer";
	return kBaseMeshDeformerString;
}

} // namespace Piston
