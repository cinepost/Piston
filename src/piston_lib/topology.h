#ifndef PISTON_LIB_TOPOLOGY_H_
#define PISTON_LIB_TOPOLOGY_H_

#include "framework.h"
#include "common.h"
#include "logging.h"

#include <memory>
#include <string>

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/imaging/hd/meshTopology.h>
#include <pxr/imaging/hd/basisCurvesTopology.h>


namespace Piston {

struct UsdPrimTopologyHash {
    size_t hash;
};

pxr::HdMeshTopology computeMeshTopology(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode time = pxr::UsdTimeCode::Default());
pxr::HdBasisCurvesTopology computeCurvesTopology(const pxr::UsdGeomBasisCurves& curves, pxr::UsdTimeCode time = pxr::UsdTimeCode::Default());

size_t computeMeshTopologyHash(const pxr::UsdGeomMesh& mesh, const pxr::HdMeshTopology& topology);
size_t computeCurvesTopologyHash(const pxr::UsdGeomBasisCurves& curves, const pxr::HdBasisCurvesTopology& topology);

bool isSameTopology(const pxr::UsdPrim& prim_l, const pxr::UsdPrim& prim_r);
 

} // namespace Piston

#endif // PISTON_LIB_TOPOLOGY_H_