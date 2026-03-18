#include "debug_drawing.h"


namespace Piston {

/*
static void CreateQuadMesh(pxr::UsdStageRefPtr stage) {
    // 1. Define the Mesh prim at a specific path
    pxr::SdfPath meshPath("/MyMesh");
    pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh::Define(stage, meshPath);

    // 2. Define the vertices (Points)
    pxr::VtArray<pxr::GfVec3f> points = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}
    };
    mesh.GetPointsAttr().Set(points);

    // 3. Define vertex counts per face (e.g., 4 for a quad)
    pxr::VtArray<int> faceCounts = {4};
    mesh.GetFaceVertexCountsAttr().Set(faceCounts);

    // 4. Define vertex indices for the face
    pxr::VtArray<int> faceIndices = {0, 1, 2, 3};
    mesh.GetFaceVertexIndicesAttr().Set(faceIndices);

    // 5. Add colors
    pxr::UsdGeomPrimvar colorPrimvar = mesh.CreateDisplayColorPrimvar(pxr::UsdGeomTokens->vertex);
    pxr::VtArray<pxr::GfVec3f> colors = {
        {1, 0, 0}, // Red
        {0, 1, 0}, // Green
        {0, 0, 1}, // Blue
        {1, 1, 0}  // Yellow
    };
    colorPrimvar.Set(colors);
}
*/

static void removePrimIfExist(pxr::UsdStageRefPtr pStage, const pxr::SdfPath& path) {
    if(pStage->GetPrimAtPath(path)) {
        pStage->RemovePrim(path);
    }
}

DebugGeo::UniquePtr DebugGeo::create(const std::string& name) {
    return std::make_unique<DebugGeo>(name);
}

bool DebugGeo::build(const std::string& path, pxr::UsdStageRefPtr pStage) {
    static const std::string kSimpleLinesPostfix = "/simpleLines";

    const std::lock_guard<std::mutex> lock(mMutex);

    // Simple colored lines
    if(!mLines.empty()) {
        pxr::SdfPath simpleLinesPath(path + kSimpleLinesPostfix);
        removePrimIfExist(pStage, simpleLinesPath);
    
        pxr::UsdGeomBasisCurves curves = pxr::UsdGeomBasisCurves::Define(pStage, simpleLinesPath);
        curves.GetTypeAttr().Set(pxr::UsdGeomTokens->linear);

        pxr::VtArray<pxr::GfVec3f> points;
        pxr::VtArray<pxr::GfVec3f> colors;
        pxr::VtArray<float> widths;

        for(const auto& l: mLines) {
            points.push_back(l.p0);
            points.push_back(l.p1);
            colors.push_back(l.c0);
            colors.push_back(l.c1);
            widths.push_back(l.w0);
            widths.push_back(l.w1);
        }

        assert(points.size() == widths.size());

        pxr::VtArray<int> curveVertexCounts(mLines.size());
        for(size_t i = 0; i < curveVertexCounts.size(); ++i) {
            curveVertexCounts[i] = 2;
        }
        curves.GetCurveVertexCountsAttr().Set(curveVertexCounts);
    
        curves.GetPointsAttr().Set(points);
        curves.GetWidthsAttr().Set(widths);

        if(points.size() == colors.size()) {
            pxr::UsdGeomPrimvar colorPrimvar = curves.CreateDisplayColorPrimvar(pxr::UsdGeomTokens->vertex);
            colorPrimvar.Set(colors);
        }
    }

    return true;
}

void DebugGeo::clear() {
    const std::lock_guard<std::mutex> lock(mMutex);

    mLines.clear();
}

} // namespace Piston