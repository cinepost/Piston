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
    static const std::string kWireBoxesPostfix = "/simpleWireBoxes";
    static const std::string kTetrasPostfix = "/simpleTetras";
    static const std::string kPointsPostfix = "/simplePoints";

    const std::lock_guard<std::mutex> lock(mMutex);

    pxr::VtArray<pxr::GfVec3f> points;
    pxr::VtArray<pxr::GfVec3f> colors;
    pxr::VtArray<float> widths;

    auto pushLine = [&](const pxr::GfVec3f& p0, const pxr::GfVec3f& p1, const pxr::GfVec3f& color, float width) {
        points.push_back(p0);
        points.push_back(p1);
        colors.push_back(color);
        colors.push_back(color);
        widths.push_back(width);
        widths.push_back(width);
    };

     auto pushLineMC = [&](const pxr::GfVec3f& p0, const pxr::GfVec3f& p1, const pxr::GfVec3f& c0, const pxr::GfVec3f& c1, float w0, float w1) {
        points.push_back(p0);
        points.push_back(p1);
        colors.push_back(c0);
        colors.push_back(c1);
        widths.push_back(w0);
        widths.push_back(w1);
    };

    // Simple colored lines
    pxr::SdfPath simpleLinesPath(path + kSimpleLinesPostfix);
    removePrimIfExist(pStage, simpleLinesPath);

    if(!mLines.empty()) {    
        pxr::UsdGeomBasisCurves curves = pxr::UsdGeomBasisCurves::Define(pStage, simpleLinesPath);
        curves.GetTypeAttr().Set(pxr::UsdGeomTokens->linear);

        points.clear();
        colors.clear();
        widths.clear();

        for(const auto& l: mLines) {
            pushLineMC(l.p0, l.p1 ,l.c0, l.c1, l.w0, l.w1);
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

    // Wireframe boxes
    pxr::SdfPath simpleWireBoxesPath(path + kWireBoxesPostfix);
    removePrimIfExist(pStage, simpleWireBoxesPath);

    if(!mWireBoxes.empty()) {
        pxr::UsdGeomBasisCurves curves = pxr::UsdGeomBasisCurves::Define(pStage, simpleWireBoxesPath);
        curves.GetTypeAttr().Set(pxr::UsdGeomTokens->linear);

        points.clear();
        colors.clear();
        widths.clear();

        for(const auto& box: mWireBoxes) {
            pxr::GfVec3f extents = box.max - box.min;
            pushLine(box.min, box.min + pxr::GfVec3f(extents[0], 0.0f, 0.0f), box.color, box.width);
            pushLine(box.min, box.min + pxr::GfVec3f(0.0f, extents[1], 0.0f), box.color, box.width);
            pushLine(box.min, box.min + pxr::GfVec3f(0.0f, 0.0f, extents[2]), box.color, box.width);

            pushLine(box.max, box.max - pxr::GfVec3f(extents[0], 0.0f, 0.0f), box.color, box.width);
            pushLine(box.max, box.max - pxr::GfVec3f(0.0f, extents[1], 0.0f), box.color, box.width);
            pushLine(box.max, box.max - pxr::GfVec3f(0.0f, 0.0f, extents[2]), box.color, box.width);
        }

        pxr::VtArray<int> curveVertexCounts(mWireBoxes.size() * 6);
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

    // Wire tetras
    pxr::SdfPath simpleTetrasPath(path + kTetrasPostfix);
    removePrimIfExist(pStage, simpleTetrasPath);

    if(!mWireTetras.empty()) {
        pxr::UsdGeomBasisCurves curves = pxr::UsdGeomBasisCurves::Define(pStage, simpleTetrasPath);
        curves.GetTypeAttr().Set(pxr::UsdGeomTokens->linear);

        points.clear();
        colors.clear();
        widths.clear();

        for(const auto& tetra: mWireTetras) {
            pushLine(tetra.p0, tetra.p1, tetra.color, tetra.width);
            pushLine(tetra.p0, tetra.p2, tetra.color, tetra.width);
            pushLine(tetra.p0, tetra.p3, tetra.color, tetra.width);
            pushLine(tetra.p1, tetra.p2, tetra.color, tetra.width);
            pushLine(tetra.p2, tetra.p3, tetra.color, tetra.width);
            pushLine(tetra.p3, tetra.p1, tetra.color, tetra.width);
        }

        pxr::VtArray<int> curveVertexCounts(mWireTetras.size() * 6);
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

    // Point cloud
    pxr::SdfPath ptCloudPath(path + kPointsPostfix);
    removePrimIfExist(pStage, ptCloudPath);

    if(!mPoints.empty()) {
        pxr::VtArray<pxr::GfVec3f> positions(mPoints.size());
        pxr::VtArray<pxr::GfVec3f> colors(mPoints.size());
        pxr::VtArray<float> widths(mPoints.size());

        for(size_t i = 0; i < mPoints.size(); ++i) {
            const auto& pt = mPoints[i];

            positions[i] = pt.pos;
            colors[i] = pt.col;
            widths[i] = pt.w;
        }

        pxr::UsdGeomPoints pointCloud = pxr::UsdGeomPoints::Define(pStage, ptCloudPath);
        pointCloud.CreatePointsAttr(pxr::VtValue(positions));

        pxr::UsdAttribute widthsAttr = pointCloud.CreateWidthsAttr(pxr::VtValue(widths));
        widthsAttr.SetMetadata(pxr::UsdGeomTokens->interpolation, pxr::UsdGeomTokens->constant);

        pxr::UsdAttribute colorAttr = pointCloud.CreateDisplayColorAttr(pxr::VtValue(colors));
        colorAttr.SetMetadata(pxr::UsdGeomTokens->interpolation, pxr::UsdGeomTokens->vertex);
    }

    return true;
}

void DebugGeo::clear() {
    const std::lock_guard<std::mutex> lock(mMutex);

    mLines.clear();
    mPoints.clear();
    mWireBoxes.clear();
    mWireTetras.clear();
}

} // namespace Piston