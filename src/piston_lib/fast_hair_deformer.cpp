#include "fast_hair_deformer.h"

namespace Piston {

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

bool FastHairDeformer::deformImpl() {
	printf("FastHairDeformer::deformImpl()\n");
	return true;
}

bool FastHairDeformer::buildDeformerData() {
	printf("FastHairDeformer::buildDeformerData()\n");

	pxr::UsdGeomMesh mesh(mMeshGeoPrimHandle.getPrim());

	printf("Mesh face count: %zu\n", mesh.GetFaceCount());

	return true;
}

} // namespace Piston