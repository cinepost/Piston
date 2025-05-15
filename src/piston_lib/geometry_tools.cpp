#include "geometry_tools.h"

namespace Piston {

bool buildGeometryFaceNormals(const pxr::UsdGeomMesh& mesh, std::vector<glm::vec3>& normals, pxr::UsdTimeCode time) {
	size_t points_count;
	
	std::vector<pxr::GfVec3f> points;

	if(!mesh.GetPointsAttr().Get<pxr::GfVec3f>(points.data(), time)) {
		return false;
	}

	//if (UsdAttribute attr = prim.GetAttribute(TfToken("myAttr"))){

	return true;
}

} // namespace Piston