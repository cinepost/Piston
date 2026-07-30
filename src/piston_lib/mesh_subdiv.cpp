#include "framework.h"
#include "common.h"
#include "logging.h"

#include "mesh_subdiv.h"

namespace Piston {

PersistentMeshRefiner::UniquePtr PersistentMeshRefiner::create() {
    return PersistentMeshRefiner::UniquePtr(new PersistentMeshRefiner());
}

void PersistentMeshRefiner::clear() {
    mIsInitialized = false;
    mOutputMesh.GetPointsAttr().Clear();
    mOutputMesh.GetFaceVertexCountsAttr().Clear();
    mOutputMesh.GetFaceVertexIndicesAttr().Clear();
    mOutputMesh.GetExtentAttr().Clear();
}

const pxr::UsdGeomMesh& PersistentMeshRefiner::getSubdividedMesh() const {
    if(mIsInitialized && mMaxLevel > 0) {
        return mOutputMesh;
    }   

    return mSourceMesh;
}

bool PersistentMeshRefiner::isValidSourceMesh() const { 
    return isValidMesh(mSourceMesh); 
}

bool PersistentMeshRefiner::isValidOutputMesh() const { 
    return isValidMesh(mOutputMesh); 
}

void PersistentMeshRefiner::init(const pxr::UsdGeomMesh& sourceMesh, uint8_t maxLevel, const std::string& rest_p_name, pxr::UsdTimeCode rest_time_code) {
    maxLevel = std::min(maxLevel,(uint8_t)1);
    if(maxLevel == 0) {
        return;
    }

    if(mIsInitialized && mRestPosName == rest_p_name && mRestTimeCode == rest_time_code && mMaxLevel == maxLevel) return;

    if(!isValidMesh(sourceMesh)) {
        LOG_ERR << "Error initializing PersistentMeshRefiner";
        return;
    }

    mMaxLevel = maxLevel;
    mRestPosName = rest_p_name;
    mRestTimeCode = rest_time_code;
    mSourceMesh = sourceMesh;

    pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = pxr::UsdGeomPrimvarsAPI::Get(mSourceMesh.GetPrim().GetStage(), mSourceMesh.GetPrim().GetPath());
    pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(mRestPosName));

    pxr::VtVec3fArray usdPoints;
    pxr::VtIntArray usdCounts;
    pxr::VtIntArray usdIndices;

    if(!restPositionPrimVar) {
        if(!mSourceMesh.GetPointsAttr().Get(&usdPoints, rest_time_code)) {
            LOG_ERR << "Error getting mesh " << mSourceMesh.GetPrim().GetPath() << " positions !";
            return;
        }
    } else {
        const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
    
        if(!restPosAttr.Get(&usdPoints, rest_time_code)) {
            LOG_ERR << "Error getting mesh " << mSourceMesh.GetPrim().GetPath() << " rest positions \"" << mRestPosName << "\" !";
            return;
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

    if(mpRefiner) {
        delete mpRefiner;
    }

    mpRefiner = OpenSubdiv::Far::TopologyRefinerFactory<OpenSubdiv::Far::TopologyDescriptor>::Create(
        desc, OpenSubdiv::Far::TopologyRefinerFactory<OpenSubdiv::Far::TopologyDescriptor>::Options(osdScheme)
    );

    if (!mpRefiner) {
        LOG_ERR << "Failed to construct OpenSubdiv Refiner. Check topology validity.";
        return;
    }
    
    mpRefiner->RefineUniform(OpenSubdiv::Far::TopologyRefiner::UniformOptions(mMaxLevel));

    int totalNumVertices = mpRefiner->GetNumVerticesTotal();
    mTmpVertexBuffer.resize(totalNumVertices);

    // fill level 0 with the baseline USD positions
    for (int i = 0; i < numVertices; ++i) {
        mTmpVertexBuffer[i].position = usdPoints[i];
    }

    OpenSubdiv::Far::PrimvarRefiner primvarRefiner(*mpRefiner);
    
    OpenSubdVertex* srcPoints = &mTmpVertexBuffer[0];
    for (int level = 1; level <= mMaxLevel; ++level) {
        OpenSubdVertex* dstPoints = srcPoints + mpRefiner->GetLevel(level - 1).GetNumVertices();

        // Linearly compute intermediate structures for the current refinement step
        primvarRefiner.Interpolate(level, srcPoints, dstPoints);
        srcPoints = dstPoints;
    }

    const OpenSubdiv::Far::TopologyLevel& refLevel = mpRefiner->GetLevel(mMaxLevel);
    int refinedNumFaces = refLevel.GetNumFaces();
    int refinedNumVertices = refLevel.GetNumVertices();
    int targetLevelVertexOffset = mpRefiner->GetNumVerticesTotal() - refinedNumVertices;

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
}

void PersistentMeshRefiner::update(pxr::UsdTimeCode time_code) const {
    if (!mpRefiner || mLastUpdateTimeCode == time_code) return;

    pxr::VtVec3fArray usdPoints;
    mSourceMesh.GetPointsAttr().Get(&usdPoints, time_code);

    int numBaseVertices = mpRefiner->GetLevel(0).GetNumVertices();
    if (usdPoints.size() != numBaseVertices) {
        LOG_ERR << "Topology mismatch: Vertex counts changed at time " << time_code << " !";
        return;
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

    for (int i = 0; i < refinedNumVertices; ++i) {
        const auto& pt = mTmpVertexBuffer[targetLevelVertexOffset + i];
        mTmpOutPoints[i] = pt.position;
    }

    mOutputMesh.GetPointsAttr().Set(mTmpOutPoints);

    mLastUpdateTimeCode = time_code;
}

void PersistentMeshRefiner::getSubdividedPrimsFromSource(int sourceFaceId, std::vector<int>& outFaceIds, std::vector<int>& outVertexIds) const  {
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
    
    getSubdividedPrimsFromSource(sourceFaceId, subFaceIds, subVertexIds);

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
