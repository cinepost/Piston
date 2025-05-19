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

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = mMeshGeoPrimHandle.getPrimvarsAPI();

	pxr::UsdGeomMesh mesh(mMeshGeoPrimHandle.getPrim());

	// Store rest positions
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(mRestPositionAttrName));
	if(!restPositionPrimVar) {
		printf("No valid primvar \"%s\" exists in mesh !\n", mRestPositionAttrName.c_str());
		return false;
	}

	const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
	if(!restPosAttr.Get(&mMeshRestPositions, pxr::UsdTimeCode::Default())) {
		printf("Error getting mesh rest positions !\n");
		return false;
	}

	printf("Mesh face count: %zu\n", mesh.GetFaceCount());

	pxr::UsdTimeCode time = pxr::UsdTimeCode::Default();
	pxr::VtArray<pxr::GfVec3f> points;

	if(!mesh.GetPointsAttr().Get(&points, time)) {
		return false;
	}

	// Hair to mesh binding data
	{
		pxr::UsdGeomPrimvarsAPI hairPrimvarsApi = mMeshGeoPrimHandle.getPrimvarsAPI();

		pxr::UsdGeomPrimvar hairToMeshBindingPrimVar = hairPrimvarsApi.GetPrimvar(pxr::TfToken(mHairToMeshBindingAttrName));
		if(!hairToMeshBindingPrimVar) {
			printf("No valid hair primvar \"%s\" exists. Building now.\n", mHairToMeshBindingAttrName.c_str());
			if(!buildHairToMeshBindingData()) {
				return false;
			}
		}

	}

	return true;
}

} // namespace Piston