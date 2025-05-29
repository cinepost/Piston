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

#include <base_hair_deformer.h>


namespace Piston {

bool buildUsdGeomMeshFaceNormals(const pxr::UsdGeomMesh& mesh, std::vector<glm::vec3>& normals, pxr::UsdTimeCode timeCode = pxr::UsdTimeCode::Default());

glm::mat3 rotateAlign(const glm::vec3& v1, const glm::vec3& v2);

pxr::GfMatrix3f rotateAlign(const pxr::GfVec3f& v1, const pxr::GfVec3f& v2);

bool pointTriangleProject(const pxr::GfVec3f &pt, const pxr::GfVec3f &n, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &t, float &u, float &v);

} // namespace Piston

#endif // PISTON_LIB_GEOMETRY_TOOLS_H_