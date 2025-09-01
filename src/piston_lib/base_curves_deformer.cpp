#include "base_curves_deformer.h"
#include "geometry_tools.h"

#include <thread>


namespace Piston {

BaseCurvesDeformer::BaseCurvesDeformer(): mPool(std::thread::hardware_concurrency() - 1) {
	dbg_printf("BaseCurvesDeformer::BaseCurvesDeformer()\n");

	makeDirty();
	mpTempStage = pxr::UsdStage::CreateInMemory();
}

void BaseCurvesDeformer::setMeshGeoPrim(const pxr::UsdPrim& geoPrim) {
	//assert(pGeoPrim);
	if(mMeshGeoPrimHandle == geoPrim) return;
	if(!isMeshGeoPrim(geoPrim)) {
		mMeshGeoPrimHandle.clear();
		std::cerr << "Mesh geometry prim is not \"Mesh\"!" << std::endl;
		return;
	}

	mMeshGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	dbg_printf("Mesh geometry prim is set to: %s\n", mMeshGeoPrimHandle.getPath().GetText());
}

void BaseCurvesDeformer::setCurvesGeoPrim(const pxr::UsdPrim& geoPrim) {
	if(mCurvesGeoPrimHandle == geoPrim) return;
	if(!isCurvesGeoPrim(geoPrim)) {
		mCurvesGeoPrimHandle.clear();
		std::cerr << "Curves geometry prim is not \"BasisCurves\"!" << std::endl;
		return;
	}

	mCurvesGeoPrimHandle = UsdPrimHandle(geoPrim);
	makeDirty();

	dbg_printf("Curves geometry prim is set to: %s\n", mCurvesGeoPrimHandle.getPath().GetText());
}

void BaseCurvesDeformer::setReadJsonDataFromPrim(bool state) {
	if(mReadJsonDeformerData == state) return;
	mDirty = true;
	mReadJsonDeformerData = state;
}

void BaseCurvesDeformer::writeJsonDataToPrim(pxr::UsdTimeCode time_code) {
	// Write json data if needed
	if(!buildDeformerData(time_code)) {
		std::cerr << "Error building deformer data !" << std::endl;
		return;
	}

	if(!mMeshGeoPrimHandle.writeDataToBson(mpAdjacencyData.get())) {
		std::cerr << "Error writing " << mpAdjacencyData->typeName() << " deformer mesh data to json !";	
	}

	if(!mMeshGeoPrimHandle.writeDataToBson(mpPhantomTrimeshData.get())) {
		std::cerr << "Error writing " << mpPhantomTrimeshData->typeName() << " deformer curves data to json !";	
	}

	writeJsonDataToPrimImpl();
}

bool BaseCurvesDeformer::buildDeformerData(pxr::UsdTimeCode reference_time_code) {
	if(!mDirty) return true;

	if(!mMeshGeoPrimHandle || !mCurvesGeoPrimHandle) {
		std::cerr << "No mesh or curves UsdPrim is set !" << std::endl;
		return false;
	}

	if(!mpAdjacencyData) {
		mpAdjacencyData = std::make_unique<SerializableUsdGeomMeshFaceAdjacency>();
	}

	// Get primitive adjacency json data if present
	if(!mReadJsonDeformerData || !mMeshGeoPrimHandle.getDataFromBson(mpAdjacencyData.get())) {
		// Build in place if no json data present or not needed
		if(!mpAdjacencyData->buildInPlace(mMeshGeoPrimHandle)) {
			std::cerr << "Error building mesh adjacency data!" << std::endl;
			return false;
		}
	}

	if(!mpPhantomTrimeshData) {
		mpPhantomTrimeshData = std::make_unique<SerializablePhantomTrimesh>();
	}

	// Get phantom mesh json data if present
	if(!mReadJsonDeformerData || !mCurvesGeoPrimHandle.getDataFromBson(mpPhantomTrimeshData.get())) {
		// Build in place if no json data present or not needed
		if(!mpPhantomTrimeshData->buildInPlace(mMeshGeoPrimHandle, getMeshRestPositionAttrName())) {
			std::cerr << "Error building phantom mesh data!" << std::endl;
			return false;
		}
	}
	
	mpCurvesContainer = PxrCurvesContainer::create(mCurvesGeoPrimHandle, reference_time_code);
	if(!mpCurvesContainer) {
		std::cerr << "Error creating curves container !" << std::endl;
		return false;
	}

	if(!buildDeformerDataImpl(reference_time_code)) {
		std::cerr << "Error building deform data !" << std::endl;
		return false;
	}

	mDirty = false;
	return true;
}

bool BaseCurvesDeformer::deform(pxr::UsdTimeCode time_code) {	
	if(!buildDeformerData(time_code)) {
		return false;
	}

	return deformImpl(time_code);
}

bool BaseCurvesDeformer::deform_mt(pxr::UsdTimeCode time_code) {	
	if(!buildDeformerData(time_code)) {
		return false;
	}

	return deformMtImpl(time_code);
}

void BaseCurvesDeformer::setMeshRestPositionAttrName(const std::string& name) {
	if(mMeshRestPositionAttrName == name) return;
	mMeshRestPositionAttrName = name;
	makeDirty();
}

void BaseCurvesDeformer::setСurvesSkinPrimAttrName(const std::string& name) {
	if(mСurvesSkinPrimAttrName == name) return;
	mСurvesSkinPrimAttrName = name;
	makeDirty();
}

void BaseCurvesDeformer::makeDirty() {
	mStats.clear();
	mDirty = true;
}

const std::string& BaseCurvesDeformer::toString() const {
	static const std::string kBaseDeformerString = "BaseCurvesDeformer";
	return kBaseDeformerString;
}

} // namespace Piston
