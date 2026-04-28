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
#include <pxr/imaging/hd/meshTopology.h>
#include <pxr/imaging/hd/basisCurvesTopology.h>

PXR_NAMESPACE_OPEN_SCOPE

bool operator<(const pxr::HdMeshTopology& lhs, const pxr::HdMeshTopology& rhs);
bool operator<(const HdBasisCurvesTopology& lhs, const HdBasisCurvesTopology& rhs);

PXR_NAMESPACE_CLOSE_SCOPE

namespace Piston {

pxr::HdMeshTopology computeMeshTopology(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode time = pxr::UsdTimeCode::Default());
pxr::HdBasisCurvesTopology computeCurvesTopology(const pxr::UsdGeomBasisCurves& curves, pxr::UsdTimeCode time = pxr::UsdTimeCode::Default());

size_t computeMeshTopologyHash(const pxr::UsdGeomMesh& mesh, const pxr::HdMeshTopology& topology);
size_t computeCurvesTopologyHash(const pxr::UsdGeomBasisCurves& curves, const pxr::HdBasisCurvesTopology& topology);

bool isSameTopology(const pxr::UsdPrim& prim_l, const pxr::UsdPrim& prim_r);
bool isSameTopology(const UsdPrimHandle& handle, const pxr::UsdPrim& prim);
bool isSameTopology(const UsdPrimHandle& handle_l, const UsdPrimHandle& handle_r);  

} // namespace Piston

#endif // PISTON_LIB_TOPOLOGY_H_