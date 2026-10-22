#include "framework.h"
#include "common.h"
#include "adjacency.h"
#include "logging.h"

#include "mesh_subdiv.h"

namespace Piston {

OsdFaceCoord calculateOsdCoordinates(uint32_t prim_id, const pxr::GfVec3f& pt, const UsdGeomMeshFaceAdjacency& adjacency, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, const OpenSubdiv::Bfr::Parameterization& faceParam) {
    OsdFaceCoord result;
    result.face_id = prim_id;
    size_t vertex_count = adjacency.getPrimVertexCount(prim_id);

    auto projectPointToPlane = [](const pxr::GfVec3f& p, const pxr::GfVec3f& planePoint, const pxr::GfVec3f& planeNormal, float& dist) {
        pxr::GfVec3f v = p - planePoint;
        dist = pxr::GfDot(v, planeNormal);
        return p - (dist * planeNormal);
    };

    auto invertBilinearQuad = [&projectPointToPlane](const pxr::GfVec3f& p, const pxr::GfVec3f& p0, const pxr::GfVec3f& p1, const pxr::GfVec3f& p2, const pxr::GfVec3f& p3, float& dist) {
        // Compute face normal to perform calculations on a 2D projection plane
        pxr::GfVec3f normal = pxr::GfCross(p1 - p0, p3 - p0).GetNormalized();
        pxr::GfVec3f projP = projectPointToPlane(p, p0, normal, dist);

        // Iterative Gauss-Newton solver for finding U and V
        float u = 0.5f, v = 0.5f; // Initial guess at center
        for (int iter = 0; iter < 6; ++iter) {
            pxr::GfVec3f guess = (1.0f - u) * (1.0f - v) * p0 + u * (1.0f - v) * p1 + u * v * p2 + (1.0f - u) * v * p3;
            pxr::GfVec3f dGdu = -(1.0f - v) * p0 + (1.0f - v) * p1 + v * p2 - v * p3;
            pxr::GfVec3f dGdv = -(1.0f - u) * p0 - u * p1 + u * p2 + (1.0f - u) * p3;
            pxr::GfVec3f delta = projP - guess;
            
            // Update steps using dot products (pseudo-inverse projection)
            float lenSqU = dGdu.GetLengthSq();
            float lenSqV = dGdv.GetLengthSq();
            
            if (lenSqU > 1e-6f) u += pxr::GfDot(delta, dGdu) / lenSqU;
            if (lenSqV > 1e-6f) v += pxr::GfDot(delta, dGdv) / lenSqV;
            
            u = std::clamp(u, 0.0f, 1.0f);
            v = std::clamp(v, 0.0f, 1.0f);
        }
        return pxr::GfVec2f(u, v);
    };

    if (vertex_count == 3 && !faceParam.HasSubFaces()) {
        // Barycentric coordinates apply only to triangular parameterizations.
        // Catmull-Clark triangles use the quadrangulated sub-face path below.
        const pxr::GfVec3f p0 = meshPointPositions[adjacency.getPrimVertex(prim_id, 0)];
        const pxr::GfVec3f p1 = meshPointPositions[adjacency.getPrimVertex(prim_id, 1)];
        const pxr::GfVec3f p2 = meshPointPositions[adjacency.getPrimVertex(prim_id, 2)];

        pxr::GfVec3f normal = pxr::GfCross(p1 - p0, p2 - p0).GetNormalized();
        pxr::GfVec3f projP = projectPointToPlane(pt, p0, normal, result.dist);
        
        float totalArea = pxr::GfDot(pxr::GfCross(p1 - p0, p2 - p0), normal);
        float areaV = pxr::GfDot(pxr::GfCross(projP - p0, p2 - p0), normal);
        float areaU = pxr::GfDot(pxr::GfCross(p1 - p0, projP - p0), normal);
        
        result.uv[0] = std::clamp(areaU / totalArea, 0.01f, 0.99f);
        result.uv[1] = std::clamp(areaV / totalArea, 0.01f, 0.99f);
        return result;
    }

    if (vertex_count == 4) {
        // Quad
        result.uv = invertBilinearQuad(pt, 
            meshPointPositions[adjacency.getPrimVertex(prim_id, 0)], 
            meshPointPositions[adjacency.getPrimVertex(prim_id, 1)], 
            meshPointPositions[adjacency.getPrimVertex(prim_id, 2)], 
            meshPointPositions[adjacency.getPrimVertex(prim_id, 3)],
            result.dist
        );

        result.uv[0] = std::clamp(result.uv[0], 0.01f, 0.99f);
        result.uv[1] = std::clamp(result.uv[1], 0.01f, 0.99f);

        return result;
    }

    // Faces with quadrangulated sub-faces, including Catmull-Clark triangles.
    pxr::GfVec3f center(0.0f, 0.0f, 0.0f); // Virtual center point exactly like Catmull-Clark/OpenSubdiv does
    for(size_t i = 0; i < vertex_count; ++i) {
        center += meshPointPositions[adjacency.getPrimVertex(prim_id, i)];
    }
    center /= static_cast<float>(vertex_count);

    // Reconstruct edge midpoints
    std::vector<pxr::GfVec3f> midpoints(vertex_count);
    for(size_t i = 0; i < vertex_count; ++i) {
        midpoints[i] = (meshPointPositions[adjacency.getPrimVertex(prim_id, i)] + meshPointPositions[adjacency.getPrimVertex(prim_id, (i + 1) % vertex_count)]) * 0.5f;
    }

    auto isPointInTriangle2D = [](const pxr::GfVec3f& p, const pxr::GfVec3f& a, const pxr::GfVec3f& b, const pxr::GfVec3f& c) {
        pxr::GfVec3f normal = pxr::GfCross(b - a, c - a).GetNormalized();
        float areaABC = pxr::GfDot(pxr::GfCross(b - a, c - a), normal);
        float areaPBC = pxr::GfDot(pxr::GfCross(c - b, p - b), normal);
        float areaPCA = pxr::GfDot(pxr::GfCross(a - c, p - c), normal);
        
        if (areaABC == 0.0f) return false;
        
        float u = areaPBC / areaABC;
        float v = areaPCA / areaABC;
        float w = 1.0f - u - v;
        
        return (u >= -1e-4f && v >= -1e-4f && w >= -1e-4f);
    };

    // Find which sub-quad quadrant the point falls into
    // We check this by evaluating which sub-quad's diagonal triangle splitting bounds the projected point P
    int subFaceId = 0;
    for (size_t i = 0; i < vertex_count; ++i) {
        size_t prev_idx = (i == 0) ? (vertex_count - 1) : (i - 1);
        
        // Vertices tracking the i-th sub-quad
        const pxr::GfVec3f& pi = meshPointPositions[adjacency.getPrimVertex(prim_id, i)];
        if (isPointInTriangle2D(pt, pi, midpoints[i], center) || 
            isPointInTriangle2D(pt, pi, center, midpoints[prev_idx])) {
            subFaceId = static_cast<int>(i);
            break;
        }
    }

    // Extract the 4 vertices defining the matched sub-quadrant [1]
    size_t i = static_cast<size_t>(subFaceId);
    size_t prev_idx = (i == 0) ? (vertex_count - 1) : (i - 1);
    
    pxr::GfVec3f q0 = meshPointPositions[adjacency.getPrimVertex(prim_id, i)];
    pxr::GfVec3f q1 = midpoints[i];
    pxr::GfVec3f q2 = center;
    pxr::GfVec3f q3 = midpoints[prev_idx];

    // Solve the local bilinear mapping inside the chosen sub-quad
    pxr::GfVec2f localUV = invertBilinearQuad(pt, q0, q1, q2, q3, result.dist);
    
    // Shift and tile the sub-face UV into the unified coordinate matrix space
    faceParam.ConvertNormalizedSubFaceToCoord(subFaceId, &localUV[0], &result.uv[0]);        

    return result;
}

// Mesh evaluator

PersistentMeshEvaluator::UniquePtr PersistentMeshEvaluator::create() {
    return PersistentMeshEvaluator::UniquePtr(new PersistentMeshEvaluator());
}

PersistentMeshEvaluator::~PersistentMeshEvaluator() {

}

bool PersistentMeshEvaluator::init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code) {
    mIsInitialized = false;
    
    mSurfaceCache.clear();

    auto mesh = pxr::UsdGeomMesh(prim_handle.getPrim());

    if(!isValidMesh(mesh)) {
        LOG_ERR << "Error initializing PersistentMeshEvaluator for " << prim_handle;
        return false;
    }

    pxr::UsdAttribute pointsAttr = mesh.GetPointsAttr();

    size_t pointCount = 0;
    pxr::VtVec3fArray usdPoints;
    pxr::VtIntArray usdCounts;
    pxr::VtIntArray usdIndices;

    if (pointsAttr.Get(&usdPoints, rest_time_code)) {
        pointCount = usdPoints.size();
    } else {
        LOG_ERR << "Points attribute is unauthored or empty on " << prim_handle;
        return false;
    }

    mesh.GetFaceVertexCountsAttr().Get(&usdCounts);
    mesh.GetFaceVertexIndicesAttr().Get(&usdIndices);

    mFaceCount = usdCounts.size();

    OpenSubdiv::Far::TopologyDescriptor desc;
    desc.numVertices = pointCount;
    desc.numFaces = mFaceCount;
    desc.numVertsPerFace = usdCounts.data();
    desc.vertIndicesPerFace = usdIndices.data();

    OpenSubdiv::Sdc::SchemeType osdScheme = OpenSubdiv::Sdc::SCHEME_CATMARK;

    OpenSubdiv::Sdc::Options options;
    options.SetVtxBoundaryInterpolation(OpenSubdiv::Sdc::Options::VTX_BOUNDARY_EDGE_ONLY);

    mpRefiner = std::unique_ptr<OpenSubdiv::Far::TopologyRefiner>(
        OpenSubdiv::Far::TopologyRefinerFactory<OpenSubdiv::Far::TopologyDescriptor>::Create(desc, OpenSubdiv::Far::TopologyRefinerFactory<OpenSubdiv::Far::TopologyDescriptor>::Options(osdScheme, options)
    ));

    if (!mpRefiner) {
        LOG_ERR << "Failed to construct OpenSubdiv Evaluator. Check " << prim_handle << " topology.";
        return false;
    }

    // Pixar recommends a value between 5 and 10 for production meshes
    OpenSubdiv::Far::TopologyRefiner::AdaptiveOptions adaptiveOptions(5);
    mpRefiner->RefineAdaptive(adaptiveOptions);

    mpSurfaceFactory = std::make_unique<OpenSubdiv::Bfr::RefinerSurfaceFactory<>>(*mpRefiner);

    mIsInitialized = true;
    return mIsInitialized;
}

const std::vector<OpenSubdiv::Bfr::Surface<float>>& PersistentMeshEvaluator::getSurfaceCache() const {
    const std::lock_guard<std::mutex> lock(mSurfaceMutex);

    assert(mIsInitialized);
    assert(mpRefiner);
    assert(mpSurfaceFactory);
    assert(mFaceCount > 0u);

    if(mSurfaceCache.empty() || static_cast<uint32_t>(mSurfaceCache.size()) != mFaceCount) {
        mMaxPatchPoints = 0;
        mSurfaceCache.resize(mFaceCount);

        for (uint32_t i = 0; i < mFaceCount; ++i) {
            mpSurfaceFactory->InitVertexSurface(i, &mSurfaceCache[i]);

            //if(!mSurfaceCache[i].IsValid()) continue;

            assert(mSurfaceCache[i].IsValid()); // TODO: handle invalid surfaces
            
            if (mSurfaceCache[i].IsValid()) {
                volatile auto warmUp = mSurfaceCache[i].GetParameterization();
            }
            mMaxPatchPoints = std::max(mMaxPatchPoints, mSurfaceCache[i].GetNumPatchPoints());
        }
    }

    return mSurfaceCache;
}

void PersistentMeshEvaluator::evaluateSurface(const OsdFaceCoord& coord, const std::vector<OpenSubdiv::Bfr::Surface<float>>& surfaceCache, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, pxr::GfVec3f& limitPos, pxr::GfVec3f& limitTangentU, pxr::GfVec3f& limitTangentV) {
    static const OpenSubdiv::Bfr::Surface<float>::PointDescriptor meshPointDesc(3, 3); // The point has 3 float components, the memory stride to the next vertex is 3 floats
    assert(coord.face_id < surfaceCache.size());
    
    const std::lock_guard<std::mutex> lock(mSurfaceMutex);
    surfaceCache[coord.face_id].Evaluate(&coord.uv[0], meshPointPositions.cdata()->data(), meshPointDesc, &limitPos[0], &limitTangentU[0], &limitTangentV[0]);
}

void PersistentMeshEvaluator::evaluateSurface(const std::vector<OsdFaceCoord>& coords, const std::vector<OpenSubdiv::Bfr::Surface<float>>& surfaceCache, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, std::vector< pxr::GfVec3f>& limitPositions, std::vector< pxr::GfVec3f>& limitTangentUs, std::vector<pxr::GfVec3f>& limitTangentVs) {
    std::vector<float> faceScratchBuffer(mMaxPatchPoints * 3);
    return evaluateSurface(coords, surfaceCache, meshPointPositions, limitPositions, limitTangentUs, limitTangentVs, faceScratchBuffer);
}

void PersistentMeshEvaluator::evaluateSurface(const std::vector<OsdFaceCoord>& coords, const std::vector<OpenSubdiv::Bfr::Surface<float>>& surfaceCache, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, std::vector< pxr::GfVec3f>& limitPositions, std::vector< pxr::GfVec3f>& limitTangentUs, std::vector<pxr::GfVec3f>& limitTangentVs, std::vector<float>& faceScratchBuffer) {
    static const OpenSubdiv::Bfr::Surface<float>::PointDescriptor meshPointDesc(3, 3); // The point has 3 float components, the memory stride to the next vertex is 3 floats
    static const OpenSubdiv::Bfr::Surface<float>::PointDescriptor patchPointDesc(3, 3);

    const std::lock_guard<std::mutex> lock(mSurfaceMutex);

    if(coords.empty()) {
        LOG_ERR << "PersistentMeshRefiner::evaluateSurface(...) coords target vector is empty !";
        return;
    }

    limitPositions.resize(coords.size());
    limitTangentUs.resize(coords.size());
    limitTangentVs.resize(coords.size());

    std::map<uint32_t, std::vector<size_t>> surface_map;
    for(size_t i = 0; i < coords.size(); ++i) {
        auto& vec = surface_map[coords[i].face_id];
        vec.push_back(i);
    }

    for(const auto& [prim_id, queries]: surface_map) {
        assert(prim_id < surfaceCache.size());
        const auto& surface = surfaceCache[prim_id];
    
        assert(surface.IsValid());
        surface.PreparePatchPoints(reinterpret_cast<const float*>(meshPointPositions.cdata()->data()), meshPointDesc, faceScratchBuffer.data(), patchPointDesc);

        for (const size_t index : queries) {
            const OsdFaceCoord& coord = coords[index];
            surface.Evaluate(&coord.uv[0], faceScratchBuffer.data(), patchPointDesc, &limitPositions[index][0], &limitTangentUs[index][0], &limitTangentVs[index][0]);
        }
    }
}

void PersistentMeshEvaluator::getPrimControlPoints(uint32_t prim_id, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, std::vector<pxr::GfVec3f>& faceControlPoints) const {
    assert(mpRefiner);

    const OpenSubdiv::Far::TopologyLevel& baseLevel = mpRefiner->GetLevel(0);

    auto faceVertIndices = baseLevel.GetFaceVertices(prim_id);
    int numFaceVertices = faceVertIndices.size();

    faceControlPoints.resize(numFaceVertices);
    if(faceControlPoints.empty()) return;

    for (int i = 0; i < numFaceVertices; ++i) {
        int meshVertIndex = faceVertIndices[i];
        faceControlPoints[i] = meshPointPositions[meshVertIndex];
    }
}

// Mesh refiner

PersistentMeshRefiner::UniquePtr PersistentMeshRefiner::create() {
    return PersistentMeshRefiner::UniquePtr(new PersistentMeshRefiner());
}

PersistentMeshRefiner::~PersistentMeshRefiner() {

}

void PersistentMeshRefiner::clear() {
    mIsInitialized = false;
    mOutputMesh.GetPointsAttr().Clear();
    mOutputMesh.GetFaceVertexCountsAttr().Clear();
    mOutputMesh.GetFaceVertexIndicesAttr().Clear();
    mOutputMesh.GetExtentAttr().Clear();
}

const pxr::UsdGeomMesh& PersistentMeshRefiner::getOutputMesh() const {
    if(mIsInitialized && mMaxLevel > 0) {
        return mOutputMesh;
    }   

    return pxr::UsdGeomMesh();
}

bool PersistentMeshRefiner::isValidSourceMesh() const { 
    return isValidMesh(mSourceMesh); 
}

bool PersistentMeshRefiner::isValidOutputMesh() const { 
    return isValidMesh(mOutputMesh); 
}

bool PersistentMeshRefiner::init(const UsdPrimHandle& prim_handle, uint8_t maxLevel, const std::string& rest_p_name, pxr::UsdTimeCode rest_time_code, bool linear_rest_interp) {
    mIsInitialized = false;

    maxLevel = std::min(maxLevel, kMaxSubdivLevel);
    if(maxLevel == 0) {
        return false;
    }

    if(mIsInitialized && mRestPosName == rest_p_name && (mRestTimeCode == rest_time_code || rest_time_code.IsDefault()) && mMaxLevel == maxLevel) return true;

    auto sourceMesh = pxr::UsdGeomMesh(prim_handle.getPrim());

    if(!isValidMesh(sourceMesh)) {
        LOG_ERR << "Error initializing PersistentMeshRefiner";
        return false;
    }

    mMaxLevel = maxLevel;
    mRestPosName = rest_p_name;
    mRestTimeCode = rest_time_code;
    mSourceMesh = pxr::UsdGeomMesh(sourceMesh);

    pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = pxr::UsdGeomPrimvarsAPI::Get(mSourceMesh.GetPrim().GetStage(), mSourceMesh.GetPrim().GetPath());
    pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(mRestPosName));

    pxr::VtVec3fArray usdPoints;
    pxr::VtIntArray usdCounts;
    pxr::VtIntArray usdIndices;

    if(!restPositionPrimVar) {
        if(!mSourceMesh.GetPointsAttr().Get(&usdPoints, rest_time_code)) {
            LOG_ERR << "Error getting mesh " << mSourceMesh.GetPrim().GetPath() << " positions !";
            return false; 
        }
    } else {
        const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
    
        if(!restPosAttr.Get(&usdPoints, rest_time_code)) {
            LOG_ERR << "Error getting mesh " << mSourceMesh.GetPrim().GetPath() << " rest positions \"" << mRestPosName << "\" !";
            return false;
        }
    }

    mSourceMesh.GetFaceVertexCountsAttr().Get(&usdCounts);
    mSourceMesh.GetFaceVertexIndicesAttr().Get(&usdIndices);

    int numVertices = usdPoints.size();
    int numFaces = usdCounts.size();

    OpenSubdiv::Far::TopologyDescriptor desc;
    desc.numVertices = numVertices;
    desc.numFaces = numFaces;
    desc.numVertsPerFace = usdCounts.data();
    desc.vertIndicesPerFace = usdIndices.data();

    pxr::TfToken schemeToken;
    mSourceMesh.GetSubdivisionSchemeAttr().Get(&schemeToken);
    OpenSubdiv::Sdc::SchemeType osdScheme = OpenSubdiv::Sdc::SCHEME_CATMARK;
    if (schemeToken == pxr::UsdGeomTokens->loop) {
        osdScheme = OpenSubdiv::Sdc::SCHEME_LOOP;
    } else if (schemeToken == pxr::UsdGeomTokens->bilinear) {
        osdScheme = OpenSubdiv::Sdc::SCHEME_BILINEAR;
    }

    mpRefiner = std::unique_ptr<OpenSubdiv::Far::TopologyRefiner>(
        OpenSubdiv::Far::TopologyRefinerFactory<OpenSubdiv::Far::TopologyDescriptor>::Create(desc, OpenSubdiv::Far::TopologyRefinerFactory<OpenSubdiv::Far::TopologyDescriptor>::Options(osdScheme)
    ));

    if (!mpRefiner) {
        LOG_ERR << "Failed to construct OpenSubdiv Refiner. Check " << prim_handle << " topology.";
        return false;
    }

    std::unique_ptr<OpenSubdiv::Far::TopologyRefiner> pRestRefiner;

    if(linear_rest_interp && (osdScheme != OpenSubdiv::Sdc::SCHEME_BILINEAR)) {
        pRestRefiner = std::unique_ptr<OpenSubdiv::Far::TopologyRefiner>(
            OpenSubdiv::Far::TopologyRefinerFactory<OpenSubdiv::Far::TopologyDescriptor>::Create(desc, OpenSubdiv::Far::TopologyRefinerFactory<OpenSubdiv::Far::TopologyDescriptor>::Options(OpenSubdiv::Sdc::SCHEME_BILINEAR))
        );
    }

    auto* pRefiner = (pRestRefiner && linear_rest_interp) ? pRestRefiner.get() : mpRefiner.get();
    
    pRefiner->RefineUniform(OpenSubdiv::Far::TopologyRefiner::UniformOptions(mMaxLevel));
    if(pRefiner != mpRefiner.get()) mpRefiner->RefineUniform(OpenSubdiv::Far::TopologyRefiner::UniformOptions(mMaxLevel));

    int totalNumVertices = pRefiner->GetNumVerticesTotal();
    mTmpVertexBuffer.resize(totalNumVertices);

    // fill level 0 with the baseline USD positions
    for (int i = 0; i < numVertices; ++i) {
        mTmpVertexBuffer[i].position = usdPoints[i];
    }

    OpenSubdiv::Far::PrimvarRefiner primvarRefiner(*pRefiner);
    
    OpenSubdVertex* srcPoints = &mTmpVertexBuffer[0];
    for (int level = 1; level <= mMaxLevel; ++level) {
        OpenSubdVertex* dstPoints = srcPoints + pRefiner->GetLevel(level - 1).GetNumVertices();

        // Linearly compute intermediate structures for the current refinement step
        primvarRefiner.Interpolate(level, srcPoints, dstPoints);
        srcPoints = dstPoints;
    }

    const OpenSubdiv::Far::TopologyLevel& refLevel = pRefiner->GetLevel(mMaxLevel);
    int refinedNumFaces = refLevel.GetNumFaces();
    int refinedNumVertices = refLevel.GetNumVertices();
    int targetLevelVertexOffset = pRefiner->GetNumVerticesTotal() - refinedNumVertices;

    mTmpOutPoints.reserve(refinedNumVertices);
    
    for (int i = 0; i < refinedNumVertices; ++i) {
        const auto& pt = mTmpVertexBuffer[targetLevelVertexOffset + i];
        mTmpOutPoints.push_back(pt.position);
    }

    // extract and format the subdivided face counts and face indices
    pxr::VtIntArray outCounts;
    pxr::VtIntArray outIndices;
    outCounts.reserve(refinedNumFaces);

    for (int face = 0; face < refinedNumFaces; ++face) {
        OpenSubdiv::Far::ConstIndexArray faceVerts = refLevel.GetFaceVertices(face);
        
        outCounts.push_back(faceVerts.size());

        for (int i = 0; i < faceVerts.size(); ++i) {
            outIndices.push_back(faceVerts[i]);
        }
    }

    if(!mpStage) {
        mpStage = pxr::UsdStage::CreateInMemory();
    }

    const std::string newPrimName = "/" + mSourceMesh.GetPrim().GetName().GetString() + "_subd";

    pxr::SdfPath path(newPrimName);
    mOutputMesh = pxr::UsdGeomMesh::Define(mpStage, path);

    mOutputMesh.GetPointsAttr().Set(mTmpOutPoints);
    mOutputMesh.GetFaceVertexCountsAttr().Set(outCounts);
    mOutputMesh.GetFaceVertexIndicesAttr().Set(outIndices);
    mOutputMesh.GetSubdivisionSchemeAttr().Set(pxr::UsdGeomTokens->none);

    //////
    mLastUpdateTimeCode = rest_time_code;
    mIsInitialized = true;

    return true;
}

bool PersistentMeshRefiner::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) const {
    assert(mIsInitialized);
    if(mLastUpdateTimeCode == time_code) return true;

    pxr::VtVec3fArray usdPoints;

    if(!mSourceMesh) {
        mSourceMesh = pxr::UsdGeomMesh(prim_handle.getPrim());
    }

    //mSourceMesh.GetPointsAttr().Get(&usdPoints, time_code);

    pxr::UsdEditTarget currentEditTarget = prim_handle.getStage()->GetEditTarget();
    pxr::UsdResolveTarget resolveTarget = prim_handle.getPrim().MakeResolveTargetUpToEditTarget(currentEditTarget);
    auto pointsQuery = pxr::UsdAttributeQuery(mSourceMesh.GetPointsAttr(), resolveTarget);

    if(!pointsQuery || !pointsQuery.Get(&usdPoints, time_code)) {
        LOG_WRN << "Error getting mesh point positions from " << prim_handle << " using UsdAttributeQuery. Reading using UsdAttribute::Get().";
        if(!prim_handle.getPoints(usdPoints, time_code)) {
            LOG_ERR << "Error getting mesh point positions from " << prim_handle << " !";
            return false;
        }
    }

    int numBaseVertices = mpRefiner->GetLevel(0).GetNumVertices();
    if (usdPoints.size() != numBaseVertices) {
        LOG_ERR << "Topology mismatch: Vertex counts changed at time " << time_code << " !";
        return false;
    }

    int totalNumVertices = mpRefiner->GetNumVerticesTotal();

    if(mTmpVertexBuffer.size() < totalNumVertices) {
        mTmpVertexBuffer.resize(totalNumVertices);
    }

    // fill base points
    for (int i = 0; i < numBaseVertices; ++i) {
        mTmpVertexBuffer[i].position = usdPoints[i];
    }

    OpenSubdiv::Far::PrimvarRefiner primvarRefiner(*mpRefiner);
    
    OpenSubdVertex* srcPoints = mTmpVertexBuffer.data();
    for (int level = 1; level <= mMaxLevel; ++level) {
        OpenSubdVertex* dstPoints = srcPoints + mpRefiner->GetLevel(level - 1).GetNumVertices();
        
        // re-evaluate positions using fixed weighted structures
        primvarRefiner.Interpolate(level, srcPoints, dstPoints);
        srcPoints = dstPoints;
    }

    //
    const OpenSubdiv::Far::TopologyLevel& refLevel = mpRefiner->GetLevel(mMaxLevel);
    int refinedNumFaces = refLevel.GetNumFaces();
    int refinedNumVertices = refLevel.GetNumVertices();
    int targetLevelVertexOffset = mpRefiner->GetNumVerticesTotal() - refinedNumVertices;

    assert(mTmpOutPoints.size() >= refinedNumVertices);

    mTmpOutPoints.resize(refinedNumVertices);
    for (int i = 0; i < refinedNumVertices; ++i) {
        const auto& pt = mTmpVertexBuffer[targetLevelVertexOffset + i];
        mTmpOutPoints[i] = pt.position;
    }

    LOG_DBG << "PersistentMeshRefiner::update(...) time_code is " << time_code.GetValue();
    LOG_DBG << "PersistentMeshRefiner::update(...) refinedNumFaces count is " << refinedNumFaces;
    LOG_DBG << "PersistentMeshRefiner::update(...) refinedNumVertices count is " << refinedNumVertices;

    LOG_DBG << "PersistentMeshRefiner::update(...) mTmpOutPoints size is " << mTmpOutPoints.size();

    mOutputMesh.GetPointsAttr().Set(mTmpOutPoints);

    mLastUpdateTimeCode = time_code;

    return true;
}

void PersistentMeshRefiner::getSubdividedPrimsFromSource(int sourceFaceId, std::vector<int>& outFaceIds) const  {
    outFaceIds.clear();
    
    if (!mpRefiner || sourceFaceId < 0 || sourceFaceId >= mpRefiner->GetLevel(0).GetNumFaces()) {
        return;
    }

    const OpenSubdiv::Far::TopologyLevel& refLevel = mpRefiner->GetLevel(mMaxLevel);
    int refinedNumFaces = refLevel.GetNumFaces();
    
    // Trace faces instantly via cached pointer maps
    for (int face = 0; face < refinedNumFaces; ++face) {
        int parentFace = face;
        for (int l = mMaxLevel; l > 0; --l) {
            parentFace = mpRefiner->GetLevel(l).GetFaceParentFace(parentFace);
        }

        if (parentFace == sourceFaceId) {
            outFaceIds.push_back(face);
        }
    }
}

void PersistentMeshRefiner::getSubdividedPrimsAndVerticesFromSource(int sourceFaceId, std::vector<int>& outFaceIds, std::vector<int>& outVertexIds) const  {
    outFaceIds.clear();
    outVertexIds.clear();

    if (!mpRefiner || sourceFaceId < 0 || sourceFaceId >= mpRefiner->GetLevel(0).GetNumFaces()) {
        return;
    }

    const OpenSubdiv::Far::TopologyLevel& refLevel = mpRefiner->GetLevel(mMaxLevel);
    int refinedNumFaces = refLevel.GetNumFaces();
    std::set<int> uniqueVerts;

    // Trace faces instantly via cached pointer maps
    for (int face = 0; face < refinedNumFaces; ++face) {
        int parentFace = face;
        for (int l = mMaxLevel; l > 0; --l) {
            parentFace = mpRefiner->GetLevel(l).GetFaceParentFace(parentFace);
        }

        if (parentFace == sourceFaceId) {
            outFaceIds.push_back(face);
            
            OpenSubdiv::Far::ConstIndexArray faceVerts = refLevel.GetFaceVertices(face);
            for (int i = 0; i < faceVerts.size(); ++i) {
                uniqueVerts.insert(faceVerts[i]);
            }
        }
    }

    outVertexIds.assign(uniqueVerts.begin(), uniqueVerts.end());
}

void PersistentMeshRefiner::querySubdividedPoints(int sourceFaceId, std::vector<pxr::GfVec3f>& points, uint32_t& count) const {
    std::vector<int> subFaceIds;
    std::vector<int> subVertexIds;
    
    getSubdividedPrimsAndVerticesFromSource(sourceFaceId, subFaceIds, subVertexIds);

    pxr::VtVec3fArray outPoints;
    mOutputMesh.GetPointsAttr().Get(&outPoints);

    LOG_DBG << "Target Face ID " << sourceFaceId << " contains:\n";
    LOG_DBG << "  - " << subFaceIds.size() << " subdivided faces.\n";
    LOG_DBG << "  - " << subVertexIds.size() << " unique vertex positions.\n";

    for (int vId : subVertexIds) {
        if (vId < outPoints.size()) {
            const pxr::GfVec3f& pt = outPoints[vId];
            LOG_DBG << "    * Output Vertex [" << vId << "]: (" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")\n";
        }
    }
}


} // namespace Piston
