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

bool isSameTopology(const pxr::UsdPrim& prim_l, const pxr::UsdPrim& prim_r) {
    if(prim_l.GetTypeName() != prim_r.GetTypeName()) return false;

    if(isMeshGeoPrim(prim_r)){
        return computeMeshTopology(pxr::UsdGeomMesh(prim_r)) == computeMeshTopology(pxr::UsdGeomMesh(prim_l));
    }

    if(isBasisCurvesGeoPrim(prim_r)) {
        return computeCurvesTopology(pxr::UsdGeomBasisCurves(prim_r)) == computeCurvesTopology(pxr::UsdGeomBasisCurves(prim_l));
    }

    return false;
}

} // namespace Piston