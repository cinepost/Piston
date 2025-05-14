#include "base_hair_deformer.h"

inline static bool isMeshGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim->GetTypeName() != "Mesh") return false;
	return true;
}

BaseHairDeformer::BaseHairDeformer(): mType(Type::UNKNOWN) {
	mpRestGeoPrim = nullptr;
	mpDeformedGeoPrim = nullptr;
	mpHairPrim = nullptr;

	printf("BaseHairDeformer::BaseHairDeformer()\n");
}

void BaseHairDeformer::setRestGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim && mpRestGeoPrim == pGeoPrim) return;
	if(!isMeshGeoPrim(pGeoPrim)) {
		printf("Rest prim is not mesh!\n");
		return;
	}

	mpRestGeoPrim = pGeoPrim;
	printf("Rest position geomtery prim is set to: %s\n", mpRestGeoPrim->GetName().GetText());
	printf("Address of rest position geomtery prim is %p\n", (void *)mpRestGeoPrim);  
}

void BaseHairDeformer::setDeformedGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim && mpDeformedGeoPrim == pGeoPrim) return;
	if(!isMeshGeoPrim(pGeoPrim)) {
		printf("Deformed prim is not mesh!\n");
		return;
	}

	mpDeformedGeoPrim = pGeoPrim;
	printf("Rest position geomtery prim is set to: %s\n", mpDeformedGeoPrim->GetName().GetText());
}

const std::string& BaseHairDeformer::greet() const {
	static const std::string kBaseDeformerString = "BaseHairDeformer";
	return kBaseDeformerString;
}