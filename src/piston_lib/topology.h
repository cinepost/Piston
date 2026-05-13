#ifndef PISTON_LIB_TOPOLOGY_H_
#define PISTON_LIB_TOPOLOGY_H_

#include "framework.h"
#include "common.h"
#include "logging.h"

#include <memory>
#include <string>
#include <variant>
#include <algorithm> 

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/basisCurves.h>

namespace Piston {

pxr::HdMeshTopology computeMeshTopology(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode time);
pxr::HdBasisCurvesTopology computeCurvesTopology(const pxr::UsdGeomBasisCurves& curves, pxr::UsdTimeCode time);

size_t computeMeshTopologyHash(const pxr::UsdGeomMesh& mesh, const pxr::HdMeshTopology& topology);
size_t computeCurvesTopologyHash(const pxr::UsdGeomBasisCurves& curves, const pxr::HdBasisCurvesTopology& topology);

bool isSameTopology(const pxr::UsdPrim& prim_l, const pxr::UsdPrim& prim_r, pxr::UsdTimeCode time_code);
bool isSameTopology(const UsdPrimHandle& handle, const pxr::UsdPrim& prim, pxr::UsdTimeCode time_code);
bool isSameTopology(const UsdPrimHandle& handle_l, const UsdPrimHandle& handle_r, pxr::UsdTimeCode time_code);  

} // namespace Piston

#endif // PISTON_LIB_TOPOLOGY_H_