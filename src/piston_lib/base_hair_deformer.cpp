#include "base_hair_deformer.h"
#include "geometry_tools.h"

namespace Piston {

inline static bool isMeshGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim->GetTypeName() != "Mesh") return false;
	return true;
}

inline static bool isHairGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim->GetTypeName() != "BasisCurves") return false;
	return true;
}

UsdPrimHandle::UsdPrimHandle(): mpStage(nullptr) {}

UsdPrimHandle::UsdPrimHandle(pxr::UsdStageWeakPtr pStage, const pxr::SdfPath& path): mpStage(pStage), mPath(path) {
	assert(mpStage);
}

UsdPrimHandle::UsdPrimHandle(const pxr::UsdPrim* pPrim) {
	assert(pPrim);

	mpStage = pPrim->GetStage();
	mPath = pPrim->GetPath();
}

pxr::UsdPrim UsdPrimHandle::getPrim() const {
	return mpStage ? mpStage->GetPrimAtPath(mPath) : pxr::UsdPrim();
}

void UsdPrimHandle::clear() {
	mpStage = nullptr;
}

bool UsdPrimHandle::operator==(const pxr::UsdPrim* pPrim) const {
	if(!pPrim) return false;
	return mpStage == pPrim->GetStage() && mPath == pPrim->GetPath();
}

BaseHairDeformer::BaseHairDeformer(): mDirty(true) {
	printf("BaseHairDeformer::BaseHairDeformer()\n");

	mpTempStage = pxr::UsdStage::CreateInMemory();
}

void BaseHairDeformer::setMeshGeoPrim(pxr::UsdPrim* pGeoPrim) {
	assert(pGeoPrim);
	if(pGeoPrim && mMeshGeoPrimHandle == pGeoPrim) return;
	if(!isMeshGeoPrim(pGeoPrim)) {
		mMeshGeoPrimHandle.clear();
		printf("Mesh geometry prim is not \"Mesh\"!\n");
		return;
	}

	mMeshGeoPrimHandle = {pGeoPrim};
	mDirty = true;

	printf("Mesh geometry prim is set to: %s\n", mMeshGeoPrimHandle.getPath().GetText());
	//printf("Address of mesh geometry prim is %p\n", (void *)pGeoPrim);  
}

void BaseHairDeformer::setHairGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim && mHairGeoPrimHandle == pGeoPrim) return;
	if(!isHairGeoPrim(pGeoPrim)) {
		mHairGeoPrimHandle.clear();
		printf("Hair geometry prim is not \"BasisCurves\"!\n");
		return;
	}

	mHairGeoPrimHandle = {pGeoPrim};
	mDirty = true;

	printf("Hair geometry prim is set to: %s\n", mHairGeoPrimHandle.getPath().GetText());
	//printf("Address of hair geometry prim is %p\n", (void *)pGeoPrim); 
}

bool BaseHairDeformer::deform(pxr::UsdTimeCode time_code) {
	if(!mMeshGeoPrimHandle || !mHairGeoPrimHandle) {
		printf("No mesh or hair UsdPrim is set !\n");
		return false;
	}
	
	if(mDirty) {
		pxr::UsdTimeCode rest_time_code = pxr::UsdTimeCode::Default();

		mpCurvesContainer = PxrCurvesContainer::create(mHairGeoPrimHandle, rest_time_code);
		if(!mpCurvesContainer) {
			printf("Error creating curves container !\n");
			return false;
		}

		if(!buildDeformerData(rest_time_code)) {
			printf("Error building deform data !\n");
			return false;
		}
		mDirty = false;
	}

	return deformImpl(time_code);
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
