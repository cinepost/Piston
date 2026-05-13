#include "topology.h"


namespace Piston {

pxr::HdMeshTopology computeMeshTopology(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode time) {
    pxr::VtIntArray faceVertexCounts, faceVertexIndices;
    
    mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts, time);
    mesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices, time);
    
    pxr::TfToken scheme;
    mesh.GetSubdivisionSchemeAttr().Get(&scheme);

    return pxr::HdMeshTopology(scheme, pxr::HdTokens->rightHanded, faceVertexCounts, faceVertexIndices);
}

pxr::HdBasisCurvesTopology computeCurvesTopology(const pxr::UsdGeomBasisCurves& curves, pxr::UsdTimeCode time) {
    pxr::VtIntArray curveVertexCounts, curveIndices;
    curves.GetCurveVertexCountsAttr().Get(&curveVertexCounts, time);

    pxr::UsdAttribute indicesAttr = curves.GetPrim().GetAttribute(pxr::TfToken("curveIndices"));
    
    if (indicesAttr.IsValid()) {
        indicesAttr.Get(&curveIndices, time);
    }

    pxr::TfToken type, basis, wrap;
    curves.GetTypeAttr().Get(&type);
    curves.GetBasisAttr().Get(&basis);
    curves.GetWrapAttr().Get(&wrap);

    return pxr::HdBasisCurvesTopology(type, basis, wrap, curveVertexCounts, curveIndices);
}

size_t computeMeshTopologyHash(const pxr::UsdGeomMesh& mesh, const pxr::HdMeshTopology& topology) {
    return topology.ComputeHash();
}

size_t computeCurvesTopologyHash(const pxr::UsdGeomBasisCurves& curves, const pxr::HdBasisCurvesTopology& topology) {
    return topology.ComputeHash();
}

bool isSameTopology(const pxr::UsdPrim& prim_l, const pxr::UsdPrim& prim_r, pxr::UsdTimeCode time_code) {
    assert(prim_l.IsValid() && "Invalid prim_l");
    assert(prim_r.IsValid() && "Invalid prim_r");
    if(!isSameType(prim_l, prim_r)) return false;

    if(isMeshGeoPrim(prim_r)){
        return computeMeshTopology(pxr::UsdGeomMesh(prim_r), time_code) == computeMeshTopology(pxr::UsdGeomMesh(prim_l), time_code);
    } else if(isBasisCurvesGeoPrim(prim_r)) {
        return computeCurvesTopology(pxr::UsdGeomBasisCurves(prim_r), time_code) == computeCurvesTopology(pxr::UsdGeomBasisCurves(prim_l), time_code);
    }

    return false;
}

bool isSameTopology(const UsdPrimHandle& handle, const pxr::UsdPrim& prim, pxr::UsdTimeCode time_code) {
    assert(handle.isValid() && "Invalid prim handle");
    assert(prim.IsValid() && "Invalid prim");
    if(!isSameType(handle.getPrim(), prim)) return false;

    size_t handle_topology_hash;
    const auto& handle_topology = handle.getTopology(time_code);
 
    size_t prim_topology_hash;
    PxrTopologyVariant prim_topology_variant;
    if(isMeshGeoPrim(prim)){
        const auto topology = computeMeshTopology(pxr::UsdGeomMesh(prim), time_code);
        prim_topology_hash = topology.ComputeHash();
        prim_topology_variant = std::move(topology);
    } else if(isBasisCurvesGeoPrim(prim)) {
        const auto topology = computeCurvesTopology(pxr::UsdGeomBasisCurves(prim), time_code);
        prim_topology_hash = topology.ComputeHash();
        prim_topology_variant = std::move(topology);
    }

    if(handle_topology.topology_hash != prim_topology_hash) return false;

    return handle_topology.topology_variant == prim_topology_variant;
}

bool isSameTopology(const UsdPrimHandle& handle_l, const UsdPrimHandle& handle_r, pxr::UsdTimeCode time_code) {
    assert(handle_l.isValid() && "Invalid prim handle_l");
    assert(handle_r.isValid() && "Invalid prim handle_r");
    if(!isSameType(handle_l.getPrim(), handle_r.getPrim())) return false;

    return handle_l.getTopology(time_code) == handle_r.getTopology(time_code);    
}

} // namespace Piston