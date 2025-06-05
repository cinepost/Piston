#include "geometry_tools.h"

#include <pxr/base/gf/math.h>

#include <limits>

#define CHECK_PARALLEL

namespace Piston {

static const pxr::GfMatrix3f kRotMat180 = {
	1.f, 0.f, 0.f,
	0.f,-1.f, 0.f,
	0.f, 0.f,-1.f
};

bool buildUsdGeomMeshFaceNormals(const pxr::UsdGeomMesh& mesh, std::vector<glm::vec3>& normals, pxr::UsdTimeCode time) {
	size_t points_count;
	
	std::vector<pxr::GfVec3f> points;

	if(!mesh.GetPointsAttr().Get<pxr::GfVec3f>(points.data(), time)) {
		return false;
	}

	return true;
}

pxr::GfMatrix3f rotateAlign(const pxr::GfVec3f& v1, const pxr::GfVec3f& v2) {
    const float cosA = pxr::GfDot( v1, v2 );
    const pxr::GfVec3f axis = pxr::GfCross( v1, v2 );
    const float k = (cosA < -1.f) ? (1.0f / (1.0f + cosA)) : 0.0f;

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

glm::mat3 rotateAlign(const glm::vec3& v1, const glm::vec3& v2) {
    const float cosA = dot( v1, v2 );
    const glm::vec3 axis = cross( v1, v2 );
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

bool pointTriangleProject(const pxr::GfVec3f &pt, const pxr::GfVec3f &n, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &t, float &u, float &v) {
    static constexpr float kEpsilon = std::numeric_limits<float>::epsilon();

    pxr::GfVec3f v0v1 = v1 - v0;
    pxr::GfVec3f v0v2 = v2 - v0;
    pxr::GfVec3f pvec = pxr::GfCross(-n, v0v2);
    float det = pxr::GfDot(v0v1, pvec);

#ifdef CHECK_CULLING
    // If the determinant is negative, the triangle is back-facing.
    // If the determinant is close to 0, the ray misses the triangle.
    if (det < kEpsilon) return false;
#else
#ifdef CHECK_PARALLEL
    // If det is close to 0, the ray and triangle are parallel.
    if (fabs(det) < kEpsilon) return false;
#endif // CHECK_PARALLEL
#endif // CHECK_CULLING
    float invDet = 1.f / det;

    pxr::GfVec3f tvec = pt - v0;
    u = pxr::GfDot(tvec, pvec) * invDet;
    if ((u < 0.f) || (u > 1.f)) {
    	return false;
    }

    pxr::GfVec3f qvec = pxr::GfCross(tvec, v0v1);
    v = pxr::GfDot(-n, qvec) * invDet;
    if ((v < 0.f) || ((u + v) > 1.f)) {
    	return false;
    }

    t = pxr::GfDot(v0v2, qvec) * invDet;
    return true;
}

} // namespace Piston