#include "fast_hair_deformer.h"

FastHairDeformer::FastHairDeformer(): BaseHairDeformer() {
	printf("FastHairDeformer::FastHairDeformer()\n");
}

FastHairDeformer::SharedPtr FastHairDeformer::create() {
	return SharedPtr(new FastHairDeformer());
}

const std::string& FastHairDeformer::toString() const {
	static const std::string kFastDeformerString = "FastHairDeformer";
	return kFastDeformerString;
}

bool FastHairDeformer::deform() {
	printf("FastHairDeformer::deform()\n");
	return true;
}

bool FastHairDeformer::buildDeformerData() {
	printf("FastHairDeformer::buildDeformerData()\n");
	return true;
}