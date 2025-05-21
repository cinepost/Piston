#include "geometry_tools.h"

#include <pxr/base/gf/math.h>


namespace Piston {

bool buildUsdGeomMeshFaceNormals(const pxr::UsdGeomMesh& mesh, std::vector<glm::vec3>& normals, pxr::UsdTimeCode time) {
	size_t points_count;
	
	std::vector<pxr::GfVec3f> points;

	if(!mesh.GetPointsAttr().Get<pxr::GfVec3f>(points.data(), time)) {
		return false;
	}

	//if (UsdAttribute attr = prim.GetAttribute(TfToken("myAttr"))){

	return true;
}

pxr::GfMatrix3f rotateAlign( pxr::GfVec3f v1, pxr::GfVec3f v2) {
    pxr::GfVec3f axis = pxr::GfCross( v1, v2 );

    const float cosA = pxr::GfDot( v1, v2 );
    const float k = 1.0f / (1.0f + cosA);

    pxr::GfMatrix3f result( (axis[0] * axis[0] * k) + cosA,
		(axis[1] * axis[0] * k) - axis[2], 
		(axis[2] * axis[0] * k) + axis[1],
		(axis[0] * axis[1] * k) + axis[2],  
		(axis[1] * axis[1] * k) + cosA,      
		(axis[2] * axis[1] * k) - axis[0],
		(axis[0] * axis[2] * k) - axis[1],  
		(axis[1] * axis[2] * k) + axis[0],  
		(axis[2] * axis[2] * k) + cosA 
	);

    return result;
}

glm::mat3 rotateAlign( glm::vec3 v1, glm::vec3 v2) {
    glm::vec3 axis = cross( v1, v2 );

    const float cosA = dot( v1, v2 );
    const float k = 1.0f / (1.0f + cosA);

    glm::mat3 result( (axis.x * axis.x * k) + cosA,
		(axis.y * axis.x * k) - axis.z, 
		(axis.z * axis.x * k) + axis.y,
		(axis.x * axis.y * k) + axis.z,  
		(axis.y * axis.y * k) + cosA,      
		(axis.z * axis.y * k) - axis.x,
		(axis.x * axis.z * k) - axis.y,  
		(axis.y * axis.z * k) + axis.x,  
		(axis.z * axis.z * k) + cosA 
	);

    return result;
}

} // namespace Piston