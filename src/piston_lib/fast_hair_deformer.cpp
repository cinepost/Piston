#include "fast_hair_deformer.h"

FastHairDeformer::FastHairDeformer(): BaseHairDeformer() {
	printf("FastHairDeformer::FastHairDeformer()\n");
}

FastHairDeformer::SharedPtr FastHairDeformer::create() {
	return SharedPtr(new FastHairDeformer());
}

const std::string& FastHairDeformer::greet() const {
	static const std::string kFastDeformerString = "FastHairDeformer";
	return kFastDeformerString;
}

bool FastHairDeformer::deform() {
	return true;
}