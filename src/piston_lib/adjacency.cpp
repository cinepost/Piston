#include "adjacency.h"


namespace Piston {

UsdGeomMeshFaceAdjacency::UsdGeomMeshFaceAdjacency(): mValid(false), mHash(0) {};

UsdGeomMeshFaceAdjacency::SharedPtr UsdGeomMeshFaceAdjacency::create(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode rest_time_code) {
	auto pResult = UsdGeomMeshFaceAdjacency::SharedPtr(new UsdGeomMeshFaceAdjacency());
	if(!pResult->init(mesh, rest_time_code)) {
		return nullptr;
	}
	return pResult;
}

bool UsdGeomMeshFaceAdjacency::init(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode rest_time_code) {
	const size_t mesh_face_count = mesh.GetFaceCount(rest_time_code);
	if(mesh_face_count == 0) {
		std::cerr << "Mesh " << mesh.GetPath() << " has no faces !" << std::endl;
		return false;
	}

	mSrcFaceVertexCounts.reserve(mesh_face_count);
	if(!mesh.GetFaceVertexCountsAttr().Get(&mSrcFaceVertexCounts, rest_time_code)) {
		std::cerr << "Error getting face vertex counts for mesh " << mesh.GetPath() << " !" << std::endl;
		return false;
	}

	mSrcFaceVertexIndices.reserve(mesh_face_count);
	if(!mesh.GetFaceVertexIndicesAttr().Get(&mSrcFaceVertexIndices, rest_time_code)) {
		std::cerr << "Error getting face vertex indices for mesh " << mesh.GetPath() << " !" << std::endl;
		return false;
	}

	{
		// fill prim vertices offsets
		uint32_t face_vertex_offset = 0;
		mSrcFaceVertexOffsets.resize(mesh_face_count);
		for(uint32_t i = 0; i < mSrcFaceVertexCounts.size(); ++i) {
			mSrcFaceVertexOffsets[i] = face_vertex_offset;
			face_vertex_offset += mSrcFaceVertexCounts[i];
		}
	}

	auto getIndex = [&] (int i) {
		return mSrcFaceVertexIndices[i];
	};

	size_t mesh_index_count = mSrcFaceVertexIndices.size();
	
	size_t mesh_vertex_count = 0;

	for(int c: mSrcFaceVertexIndices) mesh_vertex_count = std::max(mesh_vertex_count, static_cast<size_t>(c));
	mesh_vertex_count += 1;

	mCounts.resize(mesh_vertex_count);
	mOffsets.resize(mesh_vertex_count);
	mFaceData.resize(mesh_index_count);
	mCornerVertexData.resize(mesh_index_count);

	// fill prim counts
	memset(mCounts.data(), 0, mesh_vertex_count * sizeof(uint32_t));

	for (size_t i = 0; i < static_cast<size_t>(mesh_index_count); ++i) {
		assert(getIndex(i) < mesh_vertex_count);
		mCounts[getIndex(i)]++;
	}

	// fill offset table
	uint32_t offset = 0;

	for (size_t i = 0; i < static_cast<size_t>(mesh_vertex_count); ++i) {
		mOffsets[i] = offset;
		offset += mCounts[i];
	}

	assert(offset == mesh_index_count);

	// fill face data
	size_t curent_face_start_index = 0;
	for (size_t c = 0; c < mesh_face_count; ++c) {
		for(int i = 0; i < mSrcFaceVertexCounts[c]; ++i) {
			mFaceData[mOffsets[getIndex(curent_face_start_index + i)]++] = uint32_t(c);
		}

		curent_face_start_index += mSrcFaceVertexCounts[c];
	}

	// fix offsets that have been disturbed by the previous pass
	for (size_t i = 0; i < static_cast<size_t>(mesh_vertex_count); ++i) {
		assert(mOffsets[i] >= mCounts[i]);
		mOffsets[i] -= mCounts[i];
	}

	// neighbor indices data
	uint32_t face_vertex_offset = 0;
	std::vector<uint32_t> face_vertex_offsets(mesh_face_count);
	for(size_t i = 0; i < mesh_face_count; ++i) {
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
			uint32_t prim_id = mFaceData[j];
			uint32_t prim_vtx_count = static_cast<uint32_t>(mSrcFaceVertexCounts[prim_id]);
			for(uint32_t k = 0; k < prim_vtx_count; ++k) {
				if(i == mSrcFaceVertexIndices[face_vertex_offsets[prim_id] + k]) {
					neighbor_vtx_pairs.emplace_back(
						mSrcFaceVertexIndices[face_vertex_offsets[prim_id] + ((static_cast<int>(k) - 1) % prim_vtx_count)],
						mSrcFaceVertexIndices[face_vertex_offsets[prim_id] + ((static_cast<int>(k) + 1) % prim_vtx_count)]
					);
					break;
				}
			}
		}

		assert(neighbor_vtx_pairs.size() == count);
		mCornerVertexData.insert(mCornerVertexData.begin() + offset, neighbor_vtx_pairs.begin(), neighbor_vtx_pairs.end());
	}

	mHash = calcHash();
	mValid = true;

	return mValid;
}

void UsdGeomMeshFaceAdjacency::invalidate() {
	mCounts.clear();
	mOffsets.clear();
	mFaceData.clear();
	mValid = false;
	mHash = 0;
}

size_t UsdGeomMeshFaceAdjacency::calcHash() {
	size_t hash = 0;

	for(size_t i = 0; i < mCounts.size(); ++i) hash += mCounts[i]*i;
	hash += mCounts.size();

	for(size_t i = 0; i < mOffsets.size(); ++i) hash += mOffsets[i]*i;
	hash += mOffsets.size();

	for(size_t i = 0; i < mFaceData.size(); ++i) hash += mFaceData[i]*i;
	hash += mFaceData.size();

	return hash;
}

bool UsdGeomMeshFaceAdjacency::isValid() const { 
	return mValid && !mCounts.empty() && !mOffsets.empty() && !mFaceData.empty(); 
}

uint32_t UsdGeomMeshFaceAdjacency::getNeighborsCount(uint32_t idx) const { 
	assert(idx < mCounts.size());
	return mCounts[idx]; 
}

uint32_t UsdGeomMeshFaceAdjacency::getNeighborsOffset(uint32_t idx) const {
	assert(idx < mOffsets.size());
	return mOffsets[idx]; 
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
			s += std::to_string(mFaceData[offset + j]) + " ";
		}
		s += "]\n";
	}

	s += "]\n";
	return s;
}

} // namespace Piston