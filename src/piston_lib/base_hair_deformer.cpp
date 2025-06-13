#include "base_hair_deformer.h"
#include "geometry_tools.h"


namespace Piston {

BaseHairDeformer::BaseHairDeformer(): mDirty(true) {
	dbg_printf("BaseHairDeformer::BaseHairDeformer()\n");

	mpTempStage = pxr::UsdStage::CreateInMemory();
}

void BaseHairDeformer::setMeshGeoPrim(pxr::UsdPrim* pGeoPrim) {
	assert(pGeoPrim);
	if(pGeoPrim && mMeshGeoPrimHandle == pGeoPrim) return;
	if(!isMeshGeoPrim(pGeoPrim)) {
		mMeshGeoPrimHandle.clear();
		std::cerr << "Mesh geometry prim is not \"Mesh\"!" << std::endl;
		return;
	}

	mMeshGeoPrimHandle = {pGeoPrim};
	mDirty = true;

	dbg_printf("Mesh geometry prim is set to: %s\n", mMeshGeoPrimHandle.getPath().GetText());
}

void BaseHairDeformer::setHairGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim && mHairGeoPrimHandle == pGeoPrim) return;
	if(!isCurvesGeoPrim(pGeoPrim)) {
		mHairGeoPrimHandle.clear();
		std::cerr << "Hair geometry prim is not \"BasisCurves\"!" << std::endl;
		return;
	}

	mHairGeoPrimHandle = {pGeoPrim};
	mDirty = true;

	dbg_printf("Hair geometry prim is set to: %s\n", mHairGeoPrimHandle.getPath().GetText());
}

bool BaseHairDeformer::deform(pxr::UsdTimeCode time_code) {
	if(!mMeshGeoPrimHandle || !mHairGeoPrimHandle) {
		std::cerr << "No mesh or curves UsdPrim is set !" << std::endl;
		return false;
	}
	
	if(mDirty) {
		pxr::UsdTimeCode rest_time_code = pxr::UsdTimeCode::Default();

		mpCurvesContainer = PxrCurvesContainer::create(mHairGeoPrimHandle, rest_time_code);
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

bool deform_mp(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default()) {
	
}


void BaseHairDeformer::setMeshRestPositionAttrName(const std::string& name) {
	if(mRestPositionAttrName == name) return;
	mRestPositionAttrName = name;
	mDirty = true;
}

const std::string& BaseHairDeformer::toString() const {
	static const std::string kBaseDeformerString = "BaseHairDeformer";
	return kBaseDeformerString;
}

} // namespace Piston
