#ifndef PISTON_LIB_GEOMETRY_TOOLS_H_
#define PISTON_LIB_GEOMETRY_TOOLS_H_

#include "framework.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>

#include <glm/vec3.hpp> // glm::vec3

#include <base_hair_deformer.h>


namespace Piston {

bool buildGeometryFaceNormals(const pxr::UsdGeomMesh& mesh, std::vector<glm::vec3>& normals, pxr::UsdTimeCode time = pxr::UsdTimeCode::Default());

bool buildPhantomTriMesh(const pxr::UsdGeomMesh& mesh, std::vector<PhantomMeshTriface>& phantomMesh, pxr::UsdTimeCode time = pxr::UsdTimeCode::Default());

} // namespace Piston

#endif // PISTON_LIB_GEOMETRY_TOOLS_H_