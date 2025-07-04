#include "base_curves_deformer.h"
#include "geometry_tools.h"

#include <thread>


namespace Piston {

BaseCurvesDeformer::BaseCurvesDeformer(): mPool(std::thread::hardware_concurrency() - 1) {
	dbg_printf("BaseCurvesDeformer::BaseCurvesDeformer()\n");

	makeDirty();
	mpTempStage = pxr::UsdStage::CreateInMemory();
}

void BaseCurvesDeformer::setMeshGeoPrim(pxr::UsdPrim* pGeoPrim) {
	assert(pGeoPrim);
	if(pGeoPrim && mMeshGeoPrimHandle == pGeoPrim) return;
	if(!isMeshGeoPrim(pGeoPrim)) {
		mMeshGeoPrimHandle.clear();
		std::cerr << "Mesh geometry prim is not \"Mesh\"!" << std::endl;
		return;
	}

	mMeshGeoPrimHandle = {pGeoPrim};
	makeDirty();

	dbg_printf("Mesh geometry prim is set to: %s\n", mMeshGeoPrimHandle.getPath().GetText());
}

void BaseCurvesDeformer::setCurvesGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim && mCurvesGeoPrimHandle == pGeoPrim) return;
	if(!isCurvesGeoPrim(pGeoPrim)) {
		mCurvesGeoPrimHandle.clear();
		std::cerr << "Curves geometry prim is not \"BasisCurves\"!" << std::endl;
		return;
	}

	mCurvesGeoPrimHandle = {pGeoPrim};
	makeDirty();

	dbg_printf("Curves geometry prim is set to: %s\n", mCurvesGeoPrimHandle.getPath().GetText());
}

bool BaseCurvesDeformer::deform(pxr::UsdTimeCode time_code) {
	if(!mMeshGeoPrimHandle || !mCurvesGeoPrimHandle) {
		std::cerr << "No mesh or curves UsdPrim is set !" << std::endl;
		return false;
	}
	
	if(mDirty) {
		pxr::UsdTimeCode rest_time_code = pxr::UsdTimeCode::Default();

		mpCurvesContainer = PxrCurvesContainer::create(mCurvesGeoPrimHandle, rest_time_code);
		if(!mpCurvesContainer) {
			std::cerr << "Error creating curves container !" << std::endl;
			return false;
		}

		if(!buildDeformerData(rest_time_code)) {
			std::cerr << "Error building deform data !" << std::endl;
			return false;
		}
		mDirty = false;
	}

	return deformImpl(time_code);
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
