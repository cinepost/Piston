#include "base_hair_deformer.h"

BaseHairDeformer::BaseHairDeformer(): mType(Type::UNKNOWN) {
	mpRestGeoPrim = nullptr;
	mpDeformedGeoPrim = nullptr;
	mpHairPrim = nullptr;

	printf("BaseHairDeformer::BaseHairDeformer()\n");
}

void BaseHairDeformer::setRestGeoPrim(pxr::UsdPrim* pRestGeoPrim) {
	assert(pRestGeoPrim);
	mpRestGeoPrim = pRestGeoPrim;
}

const std::string& BaseHairDeformer::greet() const {
	static const std::string kBaseDeformerString = "BaseHairDeformer";
	return kBaseDeformerString;
}