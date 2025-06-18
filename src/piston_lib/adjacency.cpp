#include "adjacency.h"


namespace Piston {

UsdGeomMeshFaceAdjacency::UsdGeomMeshFaceAdjacency(): mFaceCount(0), mVertexCount(0), mMaxFaceVertexCount(0), mValid(false), mHash(0) {};

UsdGeomMeshFaceAdjacency::SharedPtr UsdGeomMeshFaceAdjacency::create(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode rest_time_code) {
	auto pResult = UsdGeomMeshFaceAdjacency::SharedPtr(new UsdGeomMeshFaceAdjacency());
	if(!pResult->init(mesh, rest_time_code)) {
		return nullptr;
	}
	return pResult;
}

bool UsdGeomMeshFaceAdjacency::init(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode rest_time_code) {
	mFaceCount = mesh.GetFaceCount(rest_time_code);
	if(mFaceCount == 0) {
		std::cerr << "Mesh " << mesh.GetPath() << " has no faces !" << std::endl;
		return false;
	}

	mSrcFaceVertexCounts.reserve(mFaceCount);
	if(!mesh.GetFaceVertexCountsAttr().Get(&mSrcFaceVertexCounts, rest_time_code)) {
		std::cerr << "Error getting face vertex counts for mesh " << mesh.GetPath() << " !" << std::endl;
		return false;
	}

	assert(mSrcFaceVertexCounts.size() == mFaceCount);

	mSrcFaceVertexIndices.reserve(mFaceCount);
	if(!mesh.GetFaceVertexIndicesAttr().Get(&mSrcFaceVertexIndices, rest_time_code)) {
		std::cerr << "Error getting face vertex indices for mesh " << mesh.GetPath() << " !" << std::endl;
		return false;
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
	mMaxFaceVertexCount = 0;
	mCounts.clear();
	mOffsets.clear();
	mPrimData.clear();
	mValid = false;
	mHash = 0;
}

size_t UsdGeomMeshFaceAdjacency::calcHash() {
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

} // namespace Piston