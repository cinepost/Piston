#include "phantom_trimesh.h"
#include "base_hair_deformer.h"
#include "geometry_tools.h"

namespace Piston {

template<typename IndexType>
typename PhantomTrimesh<IndexType>::SharedPtr create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default()) {
	assert(prim_handle.isMeshGeoPrim());

	typename PhantomTrimesh<IndexType>::SharedPtr pPhantomTrimesh = std::make_shared<PhantomTrimesh<IndexType>>();

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomMesh mesh(prim_handle.getPrim());

	// Keep rest position for further calculations
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(rest_p_name));
	if(!restPositionPrimVar) {
		printf("No valid primvar \"%s\" exists in mesh !\n", rest_p_name.c_str());
		return nullptr;
	}

	const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
	if(!restPosAttr.Get(&pPhantomTrimesh->mUsdMeshRestPositions, time_code)) {
		printf("Error getting mesh rest positions !\n");
		return nullptr;
	}

	pPhantomTrimesh->mValid = true;
	return pPhantomTrimesh;
}

template<typename IndexType>
typename PhantomTrimesh<IndexType>::TrifaceRetType PhantomTrimesh<IndexType>::getOrCreate(IndexType a, IndexType b, IndexType c) {	
	auto it = mFaceMap.find({a, b, c});
	if(it != mFaceMap.end()) {
		return {&mFaces[it.second], it.second};
	}

	const size_t idx = mFaces.size_t();

	mFaces.push_back({a, b, c});
	mFaces.back().restNormal = pxr::GfNormalize(pxr::GfCross(mUsdMeshRestPositions[b] - mUsdMeshRestPositions[a], mUsdMeshRestPositions[c] - mUsdMeshRestPositions[a]));

	return {&mFaces.back(), idx};
}

} // namespace Piston
