#include "curves_deformer_factory.h"
#include "base_mesh_curves_deformer.h"
#include "geometry_tools.h"
#include "pxr_points_lru_cache.h"

#include <thread>


namespace Piston {

BaseMeshCurvesDeformer::BaseMeshCurvesDeformer(const BaseCurvesDeformer::Type t, const std::string& name): BaseCurvesDeformer(t, name) {
	dbg_printf("BaseMeshCurvesDeformer::BaseMeshCurvesDeformer()\n");
}

bool BaseMeshCurvesDeformer::validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) {
	return isMeshGeoPrim(geoPrim);
}

bool BaseMeshCurvesDeformer::writeJsonDataToPrimImpl() const {
	if(!mDeformerGeoPrimHandle.writeDataToBson(mpAdjacencyData.get())) {
		std::cerr << "Error writing " << mpAdjacencyData->typeName() << " deformer data to json !" << std::endl;
		return false;
	}

	if(!mCurvesGeoPrimHandle.writeDataToBson(mpPhantomTrimeshData.get())) {
		std::cerr << "Error writing " << mpPhantomTrimeshData->typeName() << " curves data to json !" << std::endl;
		return false;
	}
	return true;
}

bool BaseMeshCurvesDeformer::buildDeformerDataImpl(pxr::UsdTimeCode reference_time_code, bool multi_threaded) {
	if(!mpAdjacencyData) {
		mpAdjacencyData = std::make_unique<SerializableUsdGeomMeshFaceAdjacency>();
	}

	// Get primitive adjacency json data if present
	if(!getReadJsonDataState() || !mDeformerGeoPrimHandle.getDataFromBson(mpAdjacencyData.get())) {
		// Build in place if no json data present or not needed
		if(!mpAdjacencyData->buildInPlace(mDeformerGeoPrimHandle)) {
			std::cerr << "Error building mesh adjacency data!" << std::endl;
			return false;
		}
	}

	if(!mpPhantomTrimeshData) {
		mpPhantomTrimeshData = std::make_unique<SerializablePhantomTrimesh>();
	}

	// Get phantom mesh json data if present
	if(!getReadJsonDataState() || !mCurvesGeoPrimHandle.getDataFromBson(mpPhantomTrimeshData.get())) {
		// Build in place if no json data present or not needed
		if(!mpPhantomTrimeshData->buildInPlace(mDeformerGeoPrimHandle, getDeformerRestAttrName())) {
			std::cerr << "Error building phantom mesh data!" << std::endl;
			return false;
		}
	}
	
	return true;
}

void BaseMeshCurvesDeformer::setSkinPrimAttrName(const std::string& name) {
	if(mSkinPrimAttrName == name) return;
	mSkinPrimAttrName = name;
	makeDirty();

	dbg_printf("Skin prim ID attribute name is set to: %s\n", mSkinPrimAttrName.c_str());
}


const std::string& BaseMeshCurvesDeformer::toString() const {
	static const std::string kBaseMeshDeformerString = "BaseMeshCurvesDeformer";
	return kBaseMeshDeformerString;
}

} // namespace Piston
