#ifndef PISTON_LIB_GEOMETRY_TOOLS_H_
#define PISTON_LIB_GEOMETRY_TOOLS_H_

#include "framework.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>

#include <glm/vec3.hpp>
#include <glm/glm.hpp>  

#include <pxr/base/gf/matrix3f.h>


namespace Piston {

class UsdGeomMeshFaceAdjacency;

template<typename IndexType> 
class PhantomTrimesh;

struct Plane {
	Plane(const glm::vec3& p, const glm::vec3& n): point{p[0], p[1], p[2]}, normal{n[0], n[1], n[2]} {};
	Plane(const pxr::GfVec3f& p, const pxr::GfVec3f& n): point{p[0], p[1], p[2]}, normal{n[0], n[1], n[2]} {};
	std::array<float, 3> point;
	std::array<float, 3> normal;
};

inline float distance(const glm::vec3& point, const Plane& plane) {
	return dot(glm::vec3{plane.normal[0], plane.normal[1], plane.normal[2]}, glm::vec3{point[0] - plane.point[0], point[1] - plane.point[1], point[2] - plane.point[2]});
}

inline float distance(const Plane& plane, const glm::vec3& point) { return distance(point, plane); } 

inline float distance(const pxr::GfVec3f& point, const Plane& plane) {
	return pxr::GfDot(pxr::GfVec3f{plane.normal[0], plane.normal[1], plane.normal[2]}, pxr::GfVec3f{point[0] - plane.point[0], point[1] - plane.point[1], point[2] - plane.point[2]});
}

inline float distance(const Plane& plane, const pxr::GfVec3f& point) { return distance(point, plane); } 

inline float distanceSquared(const pxr::GfVec3f &p1, const pxr::GfVec3f &p2) {
	const auto dx = p1[0] - p2[0];
	const auto dy = p1[1] - p2[1];
	const auto dz = p1[2] - p2[2];
	return (dx * dx) + (dy * dy) + (dz * dz);
}

inline float distance(const pxr::GfVec3f &p1, const pxr::GfVec3f &p2) {
	return sqrt(distanceSquared(p1, p2));
}


glm::mat3 rotateAlign(const glm::vec3& n1, const glm::vec3& n2);

pxr::GfMatrix3f rotateAlign2(const pxr::GfVec3f& n1, const pxr::GfVec3f& n2);
pxr::GfMatrix3f rotateAlign(const pxr::GfVec3f& n1, const pxr::GfVec3f& n2);

bool pointTriangleProject(const pxr::GfVec3f &pt, const pxr::GfVec3f &n, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &u, float &v);
bool pointTriangleProject(const pxr::GfVec3f &pt, const pxr::GfVec3f &n, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &dist, float &u, float &v);

bool rayTriangleIntersect(const pxr::GfVec3f &orig, const pxr::GfVec3f &dir, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &u, float &v);
bool rayTriangleIntersect(const pxr::GfVec3f &orig, const pxr::GfVec3f &dir, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &dist, float &u, float &v);

template<typename IndexType> 
void buildVertexNormals(const UsdGeomMeshFaceAdjacency* pAdjacency, const PhantomTrimesh<IndexType>* pTrimesh, std::vector<pxr::GfVec3f>& vertex_normals, bool build_live);

} // namespace Piston

#endif // PISTON_LIB_GEOMETRY_TOOLS_H_