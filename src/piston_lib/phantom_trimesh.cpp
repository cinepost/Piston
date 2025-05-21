#include "phantom_trimesh.h"
#include "base_hair_deformer.h"
#include "geometry_tools.h"

namespace Piston {

template<typename T>
PhantomTrimesh<T>::PhantomTrimesh(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code): PhantomTrimesh() {
	assert(prim_handle.isMeshGeoPrim());

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomMesh mesh(prim_handle.getPrim());

	// Keep rest position for further calculations
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(rest_p_name));
	if(!restPositionPrimVar) {
		printf("No valid primvar \"%s\" exists in mesh !\n", rest_p_name.c_str());
		return;
	}

	const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
	if(!restPosAttr.Get(&mUsdMeshRestPositions, time_code)) {
		printf("Error getting mesh rest positions !\n");
		return;
	}

	mValid = true;
}

template<typename T>
void PhantomTrimesh<T>::calculateRestTrifaceNormals() const {
	mRestNormals.resize(1);
}

} // namespace Piston
