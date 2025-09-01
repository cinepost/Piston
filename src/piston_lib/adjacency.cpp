#include "adjacency.h"

namespace Piston {

static const SerializableDeformerDataBase::DataVersion kAdjacencyDataVersion( 0u, 0u, 0u);

UsdGeomMeshFaceAdjacency::UsdGeomMeshFaceAdjacency(): mFaceCount(0), mVertexCount(0), mMaxFaceVertexCount(0), mValid(false), mHash(0) {};

UsdGeomMeshFaceAdjacency::UniquePtr UsdGeomMeshFaceAdjacency::create() {
	return std::make_unique<UsdGeomMeshFaceAdjacency>();
}

bool UsdGeomMeshFaceAdjacency::init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code) {
	assert(prim_handle.isMeshGeoPrim());

	invalidate();

	if(!prim_handle.isMeshGeoPrim()) return false;

	pxr::UsdGeomMesh mesh(prim_handle.getPrim());

	mFaceCount = mesh.GetFaceCount(rest_time_code);
	if(mFaceCount == 0) {
		std::cerr << "Mesh " << mesh.GetPath() << " has no faces !" << std::endl;
		return false;
	}

	pxr::VtArray<PxrIndexType> _srcFaceVertexCounts;
	_srcFaceVertexCounts.reserve(mFaceCount);

	if(!mesh.GetFaceVertexCountsAttr().Get(&_srcFaceVertexCounts, rest_time_code)) {
		std::cerr << "Error getting face vertex counts for mesh " << mesh.GetPath() << " !" << std::endl;
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
		std::cerr << "Error getting face vertex indices for mesh " << mesh.GetPath() << " !" << std::endl;
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
			mMaxFaceVertexCount = std::max(mMaxFaceVertexCount, static_cast<size_t>(mSrcFaceVertexCounts[i]));
		}
	}

	auto getIndex = [&] (int i) {
		return mSrcFaceVertexIndices[i];
	};

	size_t mesh_index_count = mSrcFaceVertexIndices.size();
	
	for(int c: mSrcFaceVertexIndices) {
		mVertexCount = std::max(mVertexCount, static_cast<size_t>(c));
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
		for (size_t face_id = 0; face_id < mFaceCount; ++face_id) {
			for(uint32_t j = mSrcFaceVertexOffsets[face_id]; j < mSrcFaceVertexOffsets[face_id] + mSrcFaceVertexCounts[face_id]; ++j) {
				mVtxToFace[mSrcFaceVertexIndices[j]] = face_id;
			}
		}
	}

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

uint32_t UsdGeomMeshFaceAdjacency::getNeighborsCount(uint32_t idx) const { 
	assert(idx < mCounts.size());
	return mCounts[idx]; 
}

uint32_t UsdGeomMeshFaceAdjacency::getNeighborsOffset(uint32_t idx) const {
	assert(idx < mOffsets.size());
	return mOffsets[idx]; 
}

uint32_t UsdGeomMeshFaceAdjacency::getNeighborPrim(uint32_t prim_offset) const {
	assert(prim_offset < mPrimData.size());
	return mPrimData[prim_offset];
}

uint32_t UsdGeomMeshFaceAdjacency::getFaceVertexOffset(uint32_t face_idx) const {
	assert(face_idx < mSrcFaceVertexOffsets.size());
	return mSrcFaceVertexOffsets[face_idx];
}

uint32_t UsdGeomMeshFaceAdjacency::getFaceVertexCount(uint32_t face_idx) const {
	if(face_idx >= mSrcFaceVertexCounts.size()) return 0;
	return static_cast<uint32_t>(mSrcFaceVertexCounts[face_idx]);
}

UsdGeomMeshFaceAdjacency::PxrIndexType UsdGeomMeshFaceAdjacency::getFaceVertex(uint32_t vtx_idx) const {
	assert(vtx_idx < mSrcFaceVertexIndices.size());
	return mSrcFaceVertexIndices[vtx_idx];
}

UsdGeomMeshFaceAdjacency::PxrIndexType UsdGeomMeshFaceAdjacency::getFaceVertex(uint32_t face_idx, uint32_t local_vertex_index) const {
	assert(face_idx < mSrcFaceVertexCounts.size());
	assert(local_vertex_index < mSrcFaceVertexCounts[face_idx]);
	return mSrcFaceVertexIndices[mSrcFaceVertexOffsets[face_idx] + local_vertex_index];
}

const std::pair<UsdGeomMeshFaceAdjacency::PxrIndexType, UsdGeomMeshFaceAdjacency::PxrIndexType>& UsdGeomMeshFaceAdjacency::getCornerVertexPair(uint32_t offset) const {
	assert(offset < mCornerVertexData.size());
	return mCornerVertexData[offset];
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

SerializableUsdGeomMeshFaceAdjacency::SerializableUsdGeomMeshFaceAdjacency(): SerializableDeformerDataBase() {
	mpAdjacency = UsdGeomMeshFaceAdjacency::create();
}

void SerializableUsdGeomMeshFaceAdjacency::clearData() { 
	setPopulated(false); 
	if(mpAdjacency) {
		mpAdjacency->invalidate();
	}
}

const UsdGeomMeshFaceAdjacency* SerializableUsdGeomMeshFaceAdjacency::getAdjacency() const {
	if(!isPopulated()) return nullptr;

	return mpAdjacency.get();
}

bool SerializableUsdGeomMeshFaceAdjacency::buildInPlace(const UsdPrimHandle& prim_handle) {
	assert(prim_handle.isMeshGeoPrim());
	clearData();

	if(!mpAdjacency) {
		mpAdjacency = UsdGeomMeshFaceAdjacency::create();
	}

	bool result = mpAdjacency->init(prim_handle);

	if(!result) {
		mpAdjacency->invalidate();
	}

	setPopulated(result);
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
	if(!mpAdjacency || !mpAdjacency->isValid()) return false;

	j[kJFaceCount] = mpAdjacency->mFaceCount;
	j[kJVertexCount] = mpAdjacency->mVertexCount;
	j[kJMaxFaceCount] = mpAdjacency->mMaxFaceVertexCount;

	j[kJCounts] = mpAdjacency->mCounts;
	j[kJOffsets] = mpAdjacency->mOffsets;
	j[kJPrimData] = mpAdjacency->mPrimData;
	j[kJVtxToFace] = mpAdjacency->mVtxToFace;
	j[kJCornerVertexData] = mpAdjacency->mCornerVertexData;
	
	j[kJSrcFaceVertexOffsets] = mpAdjacency->mSrcFaceVertexOffsets;
	j[kJSrcFaceVertexIndices] = mpAdjacency->mSrcFaceVertexIndices;
	j[kJSrcFaceVertexCounts] = mpAdjacency->mSrcFaceVertexCounts;

	j[kJDataHash] = mpAdjacency->calcHash();

	return true;
}

bool SerializableUsdGeomMeshFaceAdjacency::readFromJSON(const json& j) {
	mpAdjacency->mFaceCount = j[kJFaceCount];
	mpAdjacency->mVertexCount = j[kJVertexCount];
	mpAdjacency->mMaxFaceVertexCount = j[kJMaxFaceCount];

	mpAdjacency->mCounts = j[kJCounts].template get<std::vector<unsigned int>>();
	mpAdjacency->mOffsets = j[kJOffsets].template get<std::vector<unsigned int>>();
	mpAdjacency->mPrimData = j[kJPrimData].template get<std::vector<unsigned int>>();
	mpAdjacency->mVtxToFace = j[kJVtxToFace].template get<std::vector<unsigned int>>();
	mpAdjacency->mCornerVertexData = j[kJCornerVertexData];

	mpAdjacency->mSrcFaceVertexOffsets = j[kJSrcFaceVertexOffsets].template get<std::vector<unsigned int>>();
	mpAdjacency->mSrcFaceVertexIndices = j[kJSrcFaceVertexIndices].template get<std::vector<UsdGeomMeshFaceAdjacency::PxrIndexType>>();
	mpAdjacency->mSrcFaceVertexCounts = j[kJSrcFaceVertexCounts].template get<std::vector<UsdGeomMeshFaceAdjacency::PxrIndexType>>();

	const size_t json_adjacency_data_hash = j[kJDataHash];
	const size_t calc_adjacency_data_hash = mpAdjacency->calcHash();


	if(calc_adjacency_data_hash != json_adjacency_data_hash) {
		return false;
	}

	mpAdjacency->mHash = calc_adjacency_data_hash;
	mpAdjacency->mValid = true;
	return true;
}

const std::string& SerializableUsdGeomMeshFaceAdjacency::typeName() const { 
	static const std::string kTypeName = "SerializableUsdGeomMeshFaceAdjacency";
	return kTypeName;
}

const std::string& SerializableUsdGeomMeshFaceAdjacency::jsonDataKey() const {
	static const std::string kDataKey = "_piston_mesh_adjacency_data_";
	return kDataKey;
}

const SerializableDeformerDataBase::DataVersion& SerializableUsdGeomMeshFaceAdjacency::jsonDataVersion() const {
	return kAdjacencyDataVersion;
}

} // namespace Piston