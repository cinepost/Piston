#include "adjacency.h"
#include "logging.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kAdjacencyDataVersion( 1u, 0u, 0u);

UsdGeomMeshFaceAdjacency::UsdGeomMeshFaceAdjacency(): mFaceCount(0), mVertexCount(0), mMaxFaceVertexCount(0), mValid(false), mHash(0) {};

UsdGeomMeshFaceAdjacency::UniquePtr UsdGeomMeshFaceAdjacency::create() {
	return std::make_unique<UsdGeomMeshFaceAdjacency>();
}

bool UsdGeomMeshFaceAdjacency::init(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode rest_time_code) {
	invalidate();

	mFaceCount = static_cast<uint32_t>(mesh.GetFaceCount(rest_time_code));
	if(mFaceCount == 0) {
		LOG_ERR << "Mesh " << mesh.GetPath() << " has no faces !";
		return false;
	}

	pxr::VtArray<PxrIndexType> _srcFaceVertexCounts;
	_srcFaceVertexCounts.reserve(mFaceCount);

	if(!mesh.GetFaceVertexCountsAttr().Get(&_srcFaceVertexCounts, rest_time_code)) {
		LOG_ERR << "Error getting face vertex counts for mesh " << mesh.GetPath() << " !";
		return false;
	}
	mSrcFaceVertexCounts.resize(_srcFaceVertexCounts.size());
	for(size_t i = 0; i < _srcFaceVertexCounts.size(); ++i) {
		mSrcFaceVertexCounts[i] = _srcFaceVertexCounts[i];
	}

	assert(mSrcFaceVertexCounts.size() == mFaceCount);

	pxr::VtArray<PxrIndexType> _srcFaceVertexIndices;
	_srcFaceVertexIndices.reserve(mFaceCount);

	if(!mesh.GetFaceVertexIndicesAttr().Get(&_srcFaceVertexIndices, rest_time_code)) {
		LOG_ERR << "Error getting face vertex indices for mesh " << mesh.GetPath() << " !";
		return false;
	}

	mSrcFaceVertexIndices.resize(_srcFaceVertexIndices.size());
	for(size_t i = 0; i < _srcFaceVertexIndices.size(); ++i) {
		mSrcFaceVertexIndices[i] = _srcFaceVertexIndices[i];
	}

	{
		// fill prim vertices offsets and calc mMaxFaceVertexCount
		uint32_t face_vertex_offset = 0;
		mSrcFaceVertexOffsets.resize(mFaceCount);
		for(uint32_t i = 0; i < mSrcFaceVertexCounts.size(); ++i) {
			mSrcFaceVertexOffsets[i] = face_vertex_offset;
			face_vertex_offset += mSrcFaceVertexCounts[i];
			mMaxFaceVertexCount = std::max(mMaxFaceVertexCount, static_cast<uint32_t>(mSrcFaceVertexCounts[i]));
		}
	}

	auto getIndex = [&] (size_t i) {
		return mSrcFaceVertexIndices[i];
	};

	size_t mesh_index_count = mSrcFaceVertexIndices.size();
	
	for(int c: mSrcFaceVertexIndices) {
		mVertexCount = std::max(mVertexCount, static_cast<uint32_t>(c));
	}
	
	mVertexCount += 1;

	mCounts.resize(mVertexCount);
	mOffsets.resize(mVertexCount);
	mVtxToFace.resize(mVertexCount);
	mPrimData.resize(mesh_index_count);
	mCornerVertexData.resize(mesh_index_count);

	// fill prim counts
	memset(mCounts.data(), 0, mVertexCount * sizeof(uint32_t));

	for (size_t i = 0; i < static_cast<size_t>(mesh_index_count); ++i) {
		assert(getIndex(i) < mVertexCount);
		mCounts[getIndex(i)]++;
	}

	// fill offset table
	uint32_t offset = 0;

	for (size_t i = 0; i < static_cast<size_t>(mVertexCount); ++i) {
		mOffsets[i] = offset;
		offset += mCounts[i];
	}

	assert(offset == mesh_index_count);

	// fill face data
	size_t curent_face_start_index = 0;
	for (size_t c = 0; c < mFaceCount; ++c) {
		for(int i = 0; i < mSrcFaceVertexCounts[c]; ++i) {
			mPrimData[mOffsets[getIndex(curent_face_start_index + i)]++] = uint32_t(c);
		}

		curent_face_start_index += mSrcFaceVertexCounts[c];
	}

	// fix offsets that have been disturbed by the previous pass
	for (size_t i = 0; i < static_cast<size_t>(mVertexCount); ++i) {
		assert(mOffsets[i] >= mCounts[i]);
		mOffsets[i] -= mCounts[i];
	}

	// neighbor indices data
	uint32_t face_vertex_offset = 0;
	std::vector<uint32_t> face_vertex_offsets(mFaceCount);
	for(size_t i = 0; i < mFaceCount; ++i) {
		face_vertex_offsets[i] = face_vertex_offset;
		face_vertex_offset += mSrcFaceVertexCounts[i];
	}

	std::vector<std::pair<PxrIndexType, PxrIndexType>> neighbor_vtx_pairs;
	neighbor_vtx_pairs.reserve(128);
	for(size_t i = 0; i < mCounts.size(); ++i) {
		uint32_t count = mCounts[i];
		uint32_t offset = mOffsets[i];
		neighbor_vtx_pairs.clear();
		// iterate neighbor prims
		for (uint32_t j = offset; j < (offset + count); ++j) {
			uint32_t prim_id = mPrimData[j];
			uint32_t prim_vtx_count = static_cast<uint32_t>(mSrcFaceVertexCounts[prim_id]);
			for(uint32_t k = 0; k < prim_vtx_count; ++k) {
				if(i == mSrcFaceVertexIndices[face_vertex_offsets[prim_id] + k]) {
					neighbor_vtx_pairs.emplace_back(
						mSrcFaceVertexIndices[face_vertex_offsets[prim_id] + ((static_cast<int>(k) + prim_vtx_count - 1) % prim_vtx_count)],
						mSrcFaceVertexIndices[face_vertex_offsets[prim_id] + ((static_cast<int>(k) + 1) % prim_vtx_count)]
					);
					break;
				}
			}
		}

		assert(neighbor_vtx_pairs.size() == count);
		mCornerVertexData.insert(mCornerVertexData.begin() + offset, neighbor_vtx_pairs.begin(), neighbor_vtx_pairs.end());
	}

	// build reverse vertex to face relations data 
	{
		for (uint32_t face_id = 0; face_id < mFaceCount; ++face_id) {
			for(uint32_t j = mSrcFaceVertexOffsets[face_id]; j < mSrcFaceVertexOffsets[face_id] + mSrcFaceVertexCounts[face_id]; ++j) {
				mVtxToFace[mSrcFaceVertexIndices[j]] = face_id;
			}
		}
	}

	LOG_DBG << "Adjacency face count is " << mFaceCount;

	mHash = calcHash();
	mValid = true;

	return mValid;
}

uint32_t UsdGeomMeshFaceAdjacency::getVertexFaceId(uint32_t vtx) const {
	assert(vtx < mVtxToFace.size());
	return mVtxToFace[vtx];
}

void UsdGeomMeshFaceAdjacency::invalidate() {
	mFaceCount = 0;
	mVertexCount = 0;
	mMaxFaceVertexCount = 0;

	mCounts.clear();
	mOffsets.clear();
	mPrimData.clear();
	mVtxToFace.clear();
    mCornerVertexData.clear();
    mSrcFaceVertexOffsets.clear();

    mSrcFaceVertexIndices.clear();
	mSrcFaceVertexCounts.clear();

	mValid = false;
	mHash = 0;
}

size_t UsdGeomMeshFaceAdjacency::calcHash() const {
	size_t hash = 0;

	for(size_t i = 0; i < mCounts.size(); ++i) hash += mCounts[i]*i;
	hash += mCounts.size();

	for(size_t i = 0; i < mOffsets.size(); ++i) hash += mOffsets[i]*i;
	hash += mOffsets.size();

	for(size_t i = 0; i < mPrimData.size(); ++i) hash += mPrimData[i]*i;
	hash += mPrimData.size();

	return hash;
}

bool UsdGeomMeshFaceAdjacency::isValid() const { 
	return mValid && !mCounts.empty() && !mOffsets.empty() && !mPrimData.empty(); 
}

std::vector<uint32_t> UsdGeomMeshFaceAdjacency::getNeighborPrims(uint32_t common_point_idx) const {
	std::vector<uint32_t> prim_indices;

	const uint32_t neighbors_count = getNeighborsCount(common_point_idx);
	if(neighbors_count > 0) {
		prim_indices.resize(neighbors_count);
		const uint32_t neighbors_offset = getNeighborsOffset(common_point_idx);

		for(uint32_t i = 0; i < neighbors_count; ++i){
			prim_indices[i] = getNeighborPrim(neighbors_offset + i);
		}
	}

	return prim_indices;
}

uint32_t UsdGeomMeshFaceAdjacency::findBestPrimFast(const pxr::GfVec3f target, uint32_t common_point_idx) const {
	const uint32_t neighbors_count = getNeighborsCount(common_point_idx);
	if(neighbors_count == 0) return kInvalidID;
	const uint32_t neighbors_offset = getNeighborsOffset(common_point_idx);
	if(neighbors_count == 1) return getNeighborPrim(neighbors_offset);

	std::vector<uint32_t> prim_indices(neighbors_count);

	for(uint32_t i = 0; i <neighbors_count; ++i){
		prim_indices[i] = getNeighborPrim(neighbors_offset + i);
	}

	auto min_it = std::min_element(prim_indices.begin(), prim_indices.end());
    return *min_it;
}
////////////
float UsdGeomMeshFaceAdjacency::evaluatePrimMatch(uint32_t prim_id, const pxr::GfVec3f& target, const pxr::VtArray<pxr::GfVec3f>& positions, float weightDistance, float weightOrientation) const {
    const uint32_t prim_vertex_count = getPrimVertexCount(prim_id);
    assert(prim_vertex_count > 2);
    if (prim_vertex_count < 3) return std::numeric_limits<float>::max(); // degenerate prim. 

    pxr::GfVec3f centroid(0.0, 0.0, 0.0);

    for (uint32_t i = 0; i < prim_vertex_count; ++i) {
    	centroid += positions[getPrimVertex(prim_id, i)];
    }
    centroid /= static_cast<float>(prim_vertex_count);

    pxr::GfVec3f targetDir = target - centroid;
    float targetDist = targetDir.Normalize(); // Normalizes in place, returns distance to centroid


   	// 2. Compute Orientation (Average Direction of this Prim)
    pxr::GfVec3f primAvgDir(0, 0, 0);
    for (uint32_t i = 0; i < prim_vertex_count; ++i) {
    	pxr::GfVec3f dir = positions[getPrimVertex(prim_id, i)] - centroid;
        dir.Normalize();
        primAvgDir += dir;
    }
    primAvgDir.Normalize();

    // Orientation Penalty: 1.0 means perfectly aligned, -1.0 means opposite direction
    float dotAlignment = GfDot(targetDir, primAvgDir);
    // Convert to a penalty where 0 is perfect alignment and 2 is worst
    float orientationPenalty = 1.0f - dotAlignment;

    // 3. Compute Shortest Distance to the Ray Edges radiating from the common point
    float minEdgeDistance = std::numeric_limits<float>::max();
    float totalEdgeDistance = 0.0f;
    uint32_t edgeCount = 0;

    // Evaluate outer lip edges connecting consecutive outer perimeter vertices
    for (uint32_t i = 0; i < prim_vertex_count; ++i) {
        pxr::GfLineSeg outerLip(positions[getPrimVertex(prim_id, i)], positions[getPrimVertex(prim_id, (i + 1) % prim_vertex_count)]);
        pxr::GfVec3f closestOnLip = pxr::GfVec3f(outerLip.FindClosestPoint(target));
        float dist = (closestOnLip - target).GetLength();

        minEdgeDistance = std::min(minEdgeDistance, dist);
        totalEdgeDistance += dist;
        edgeCount++;
    }

    float avgEdgeDistance = totalEdgeDistance / static_cast<float>(edgeCount);
    float blendedDistance = (minEdgeDistance * 0.4f) + (avgEdgeDistance * 0.6f);

    // 4. Combine Metrics into a Single Score (Lower is better/closer)
    // Adjust weights based on whether physical proximity or angular alignment matters more
    float finalScore = (weightDistance * blendedDistance) + (weightOrientation * orientationPenalty * targetDist);
    return finalScore;
}

float UsdGeomMeshFaceAdjacency::evaluatePrimMatch(uint32_t prim_id, const pxr::GfVec3f& target, uint32_t common_point_idx, const pxr::VtArray<pxr::GfVec3f>& positions, float weightDistance, float weightOrientation) const {
    const uint32_t prim_vertex_count = getPrimVertexCount(prim_id);
    assert(prim_vertex_count > 2);
    if (prim_vertex_count < 3) return std::numeric_limits<float>::max(); // degenerate prim. 

    assert(common_point_idx < positions.size());
    const pxr::GfVec3f& commonPoint = positions[common_point_idx];

   	// 1. Calculate Target Direction from the Common Apex
   	pxr::GfVec3f targetDir = target - commonPoint;
   	float targetDist = targetDir.Normalize(); // Normalizes in place, returns original length

   	// 2. Compute Orientation (Average Direction of this Prim)
    pxr::GfVec3f primAvgDir(0, 0, 0);
    for (uint32_t i = 0; i < prim_vertex_count; ++i) {
    	assert(getPrimVertex(prim_id, i) < positions.size());
        pxr::GfVec3f dir = positions[getPrimVertex(prim_id, i)] - commonPoint;
        dir.Normalize();
        primAvgDir += dir;
    }
    primAvgDir.Normalize();

    // Orientation Penalty: 1.0 means perfectly aligned, -1.0 means opposite direction
    float dotAlignment = GfDot(targetDir, primAvgDir);
    // Convert to a penalty where 0 is perfect alignment and 2 is worst
    float orientationPenalty = 1.0f - dotAlignment;

    // 3. Compute Shortest Distance to the Ray Edges radiating from the common point
    float minEdgeDistance = std::numeric_limits<float>::max();
    float totalEdgeDistance = 0.0f;
    uint32_t edgeCount = 0;

    for (uint32_t i = 0; i < prim_vertex_count; ++i) {
    	assert(getPrimVertex(prim_id, i) < positions.size());
        // Form a finite edge segment from the common apex to the perimeter boundary point
        pxr::GfLineSeg edgeSegment(commonPoint, positions[getPrimVertex(prim_id, i)]);
        pxr::GfVec3f closestPointOnEdge = pxr::GfVec3f(edgeSegment.FindClosestPoint(target));
        float dist = (closestPointOnEdge - target).GetLength();

        minEdgeDistance = std::min(minEdgeDistance, dist);
        totalEdgeDistance += dist;
        edgeCount++;
    }

    // Evaluate outer lip edges connecting consecutive outer perimeter vertices
    for (uint32_t i = 0; i < prim_vertex_count; ++i) {
        pxr::GfLineSeg outerLip(positions[getPrimVertex(prim_id, i)], positions[getPrimVertex(prim_id, (i + 1) % prim_vertex_count)]);
        pxr::GfVec3f closestOnLip = pxr::GfVec3f(outerLip.FindClosestPoint(target));
        float dist = (closestOnLip - target).GetLength();

        minEdgeDistance = std::min(minEdgeDistance, dist);
        totalEdgeDistance += dist;
        edgeCount++;
    }

    float avgEdgeDistance = totalEdgeDistance / static_cast<float>(edgeCount);
    float blendedDistance = (minEdgeDistance * 0.4f) + (avgEdgeDistance * 0.6f);

    // 4. Combine Metrics into a Single Score (Lower is better/closer)
    // Adjust weights based on whether physical proximity or angular alignment matters more
    float finalScore = (weightDistance * blendedDistance) + (weightOrientation * orientationPenalty * targetDist);
    return finalScore;
}

///////////
uint32_t UsdGeomMeshFaceAdjacency::findBestPrimOriented(const pxr::GfVec3f target, const std::vector<uint32_t>& prims, const pxr::VtArray<pxr::GfVec3f>& positions) const {
    uint32_t best_prim_id = kInvalidID;
    float lowestScore = std::numeric_limits<float>::max();

    for (const uint32_t prim_id : prims) {
        // Equal weighting for distance to boundary and orientation direction
        float score = evaluatePrimMatch(prim_id, target, positions, 0.6f, 0.4f);
        
        if (score < lowestScore) {
            lowestScore = score;
            best_prim_id = prim_id;
        }
    }

    return best_prim_id;
}

uint32_t UsdGeomMeshFaceAdjacency::findBestPrimOriented(const pxr::GfVec3f target, uint32_t common_point_idx, const pxr::VtArray<pxr::GfVec3f>& positions) const {
    uint32_t best_prim_id = kInvalidID;
    float lowestScore = std::numeric_limits<float>::max();

    for (const uint32_t prim_id : getNeighborPrims(common_point_idx)) {
        // Equal weighting for distance to boundary and orientation direction
        float score = evaluatePrimMatch(prim_id, target, common_point_idx, positions, 0.6f, 0.4f);
        
        if (score < lowestScore) {
            lowestScore = score;
            best_prim_id = prim_id;
        }
    }

    return best_prim_id;
}

std::string UsdGeomMeshFaceAdjacency::toString() const {
	if(!isValid()) return "Invalid UsdGeomMeshFaceAdjacency";

	std::string s = "[\n";
	for(size_t i = 0; i < mCounts.size(); ++i) {
		s += std::to_string(i) + ":[";
		uint32_t count = mCounts[i];
		uint32_t offset = mOffsets[i];
		for(uint32_t j = 0; j < count; ++j) {
			s += std::to_string(mPrimData[offset + j]) + " ";
		}
		s += "]\n";
	}

	s += "]\n";
	return s;
}

SerializableUsdGeomMeshFaceAdjacency::SerializableUsdGeomMeshFaceAdjacency(): SerializableDeformerDataBase(), mHasSubdividedAdjacencyData(false) {
	mpAdjacency = UsdGeomMeshFaceAdjacency::create();
	mpAdjacencySubd = nullptr;
}

void SerializableUsdGeomMeshFaceAdjacency::clearData() { 
	const std::lock_guard<std::mutex> lock(mMutex);

	if(mpAdjacency) {
		mpAdjacency->invalidate();

		if(mpAdjacencySubd) {
			mpAdjacencySubd->invalidate();
		}

		mpAdjacencySubd = nullptr;
	}

	mHasSubdividedAdjacencyData = false;
}

const UsdGeomMeshFaceAdjacency* SerializableUsdGeomMeshFaceAdjacency::getAdjacency() const {
	if(!isValid()) return nullptr;

	return mpAdjacency.get();
}

const UsdGeomMeshFaceAdjacency* SerializableUsdGeomMeshFaceAdjacency::getAdjacencySubd() const {
	return mpAdjacencySubd.get();
}

bool SerializableUsdGeomMeshFaceAdjacency::buildInPlace(const UsdPrimHandle& prim_handle) {
	if(isValid()) {
		// Data is valid. No need to rebuild it.
		return true;
	}

	clearData();

	const std::lock_guard<std::mutex> lock(mMutex);

	if(!prim_handle.isMeshGeoPrim()) return false;

	if(!mpAdjacency) {
		mpAdjacency = UsdGeomMeshFaceAdjacency::create();
	}

	assert(prim_handle.isMeshGeoPrim());
	pxr::UsdGeomMesh mesh(prim_handle.getPrim());

	if(!isValidMesh(mesh)) {
		return false;
	}

	bool result = mpAdjacency->init(mesh);
	if(!result) {
		mpAdjacency->invalidate();
		return false;
	}	
	
	// Init subdivided adjacency if needed
	const PersistentMeshRefiner* pRefiner = prim_handle.getMeshRefiner();
	if(pRefiner && pRefiner->isInitialized() && pRefiner->isValidOutputMesh()) {
		if(!mpAdjacencySubd) {
			mpAdjacencySubd = UsdGeomMeshFaceAdjacency::create();
		}

		assert(mpAdjacencySubd);

		if(!mpAdjacencySubd->init(pRefiner->getOutputMesh())) {
			mpAdjacencySubd->invalidate();
			LOG_ERR << "Error initializing adjacency data for subdivided mesh " << prim_handle;
		} else {
			mHasSubdividedAdjacencyData = true;
		}
	}
	
	return result;
}

static constexpr const char* kJFaceCount = "face_cnt";
static constexpr const char* kJVertexCount = "vtx_cnt";
static constexpr const char* kJMaxFaceCount = "max_face_cnt";
static constexpr const char* kJCounts = "counts";
static constexpr const char* kJOffsets = "offsets";
static constexpr const char* kJPrimData = "prm_data";
static constexpr const char* kJVtxToFace = "vtx2face";
static constexpr const char* kJCornerVertexData = "crnvtxdata";
static constexpr const char* kJSrcFaceVertexOffsets = "srcvfacetxoffset";

static constexpr const char* kJSrcFaceVertexIndices = "srcfacevtxindices";
static constexpr const char* kJSrcFaceVertexCounts = "srcfacevtxcounts";

static constexpr const char* kJDataHash = "data_hash";

bool SerializableUsdGeomMeshFaceAdjacency::dumpToJSON(json& j) const {
	const std::lock_guard<std::mutex> lock(mMutex);

	auto data_to_json = [](const UsdGeomMeshFaceAdjacency* pAdjacency) -> json {
		json j;

		if(pAdjacency && pAdjacency->isValid()) {
			j[kJFaceCount] = pAdjacency->mFaceCount;
			j[kJVertexCount] = pAdjacency->mVertexCount;
			j[kJMaxFaceCount] = pAdjacency->mMaxFaceVertexCount;

			j[kJCounts] = pAdjacency->mCounts;
			j[kJOffsets] = pAdjacency->mOffsets;
			j[kJPrimData] = pAdjacency->mPrimData;
			j[kJVtxToFace] = pAdjacency->mVtxToFace;
			j[kJCornerVertexData] = pAdjacency->mCornerVertexData;
			
			j[kJSrcFaceVertexOffsets] = pAdjacency->mSrcFaceVertexOffsets;
			j[kJSrcFaceVertexIndices] = pAdjacency->mSrcFaceVertexIndices;
			j[kJSrcFaceVertexCounts] = pAdjacency->mSrcFaceVertexCounts;

			j[kJDataHash] = pAdjacency->calcHash();
		}

        return j;
    };

	j["adj_data"] = data_to_json(mpAdjacency.get());
	if(mpAdjacencySubd && mpAdjacencySubd->isValid()) {
		j["adj_has_subd_data"] = true;
		j["adj_subd_data"] = data_to_json(mpAdjacencySubd.get());
	}

	return true;
}

bool SerializableUsdGeomMeshFaceAdjacency::readFromJSON(const json& j) {
	const std::lock_guard<std::mutex> lock(mMutex);

	auto json_to_data = [](const json& j, UsdGeomMeshFaceAdjacency* pAdjacency) -> bool {
		if(!pAdjacency) {
			LOG_ERR << "SerializableUsdGeomMeshFaceAdjacency::readFromJSON lambda error. No pAdjacency !";
			return false;
		}

		pAdjacency->mValid = false;

		pAdjacency->mFaceCount = j[kJFaceCount];
		pAdjacency->mVertexCount = j[kJVertexCount];
		pAdjacency->mMaxFaceVertexCount = j[kJMaxFaceCount];

		pAdjacency->mCounts = j[kJCounts].template get<std::vector<unsigned int>>();
		pAdjacency->mOffsets = j[kJOffsets].template get<std::vector<unsigned int>>();
		pAdjacency->mPrimData = j[kJPrimData].template get<std::vector<unsigned int>>();
		pAdjacency->mVtxToFace = j[kJVtxToFace].template get<std::vector<unsigned int>>();
		pAdjacency->mCornerVertexData = j[kJCornerVertexData];

		pAdjacency->mSrcFaceVertexOffsets = j[kJSrcFaceVertexOffsets].template get<std::vector<unsigned int>>();
		pAdjacency->mSrcFaceVertexIndices = j[kJSrcFaceVertexIndices].template get<std::vector<UsdGeomMeshFaceAdjacency::PxrIndexType>>();
		pAdjacency->mSrcFaceVertexCounts = j[kJSrcFaceVertexCounts].template get<std::vector<UsdGeomMeshFaceAdjacency::PxrIndexType>>();

		const size_t json_adjacency_data_hash = j[kJDataHash];
		const size_t calc_adjacency_data_hash = pAdjacency->calcHash();

		if(calc_adjacency_data_hash != json_adjacency_data_hash) {
			LOG_ERR << "SerializableUsdGeomMeshFaceAdjacency::readFromJSON lambda error. Data hash mismatch !";
			return false;
		}

		pAdjacency->mHash = calc_adjacency_data_hash;
		pAdjacency->mValid = true;

		return pAdjacency->mValid;
    };

    if(!json_to_data(j["adj_data"], mpAdjacency.get())) {
    	return false;
    }

    if(j["adj_has_subd_data"]) {
    	if(!mpAdjacencySubd) {
    		mpAdjacencySubd = UsdGeomMeshFaceAdjacency::create();
    	}

    	assert(mpAdjacencySubd);

    	if(!json_to_data(j["adj_subd_data"], mpAdjacencySubd.get())) {
    		LOG_WRN << "Error reading subdivided adjacency data !!!";
    		mpAdjacencySubd->invalidate();
    		return false;
    	}

    	LOG_DBG << "Subdivided adjacency data read from json payload.";
    }

	LOG_DBG << "Adjacency data read from json payload.";

	return true;
}

const std::string& SerializableUsdGeomMeshFaceAdjacency::typeName() const { 
	static const std::string kTypeName = "SerializableUsdGeomMeshFaceAdjacency";
	return kTypeName;
}

const std::string& SerializableUsdGeomMeshFaceAdjacency::jsonDataKey() const {
	static const std::string kDataKey = "piston_mesh_adjacency_data";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& SerializableUsdGeomMeshFaceAdjacency::jsonDataVersion() const {
	return kAdjacencyDataVersion;
}

} // namespace Piston