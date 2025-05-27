#include "adjacency.h"


namespace Piston {

UsdGeomMeshFaceAdjacency::UsdGeomMeshFaceAdjacency(): mValid(false), mHash(0) {};

UsdGeomMeshFaceAdjacency UsdGeomMeshFaceAdjacency::create(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode timeCode) {
	UsdGeomMeshFaceAdjacency adjacency;

	const size_t mesh_face_count = mesh.GetFaceCount(timeCode);
	if(mesh_face_count == 0) {
		printf("Mesh has no faces !\n");
		return adjacency;
	}

	pxr::VtArray<int> face_vertex_counts;
	if(!mesh.GetFaceVertexCountsAttr().Get(&face_vertex_counts, timeCode)) {
		printf("Error getting face vertex counts for mesh !\n");
		return adjacency;
	}

	pxr::VtArray<int> face_vertex_indices;
	if(!mesh.GetFaceVertexIndicesAttr().Get(&face_vertex_indices, timeCode)) {
		printf("Error getting face vertex indices for mesh !\n");
		return adjacency;
	}

	auto getIndex = [&] (int i) {
		return face_vertex_indices[i];
	};

	size_t mesh_index_count = face_vertex_indices.size();
	
	size_t mesh_vertex_count = 0;

	for(int c: face_vertex_indices) mesh_vertex_count = std::max(mesh_vertex_count, static_cast<size_t>(c));
	mesh_vertex_count += 1;

	adjacency.mCounts.resize(mesh_vertex_count);
	adjacency.mOffsets.resize(mesh_vertex_count);
	adjacency.mFaceData.resize(mesh_index_count);

	// fill prim counts
	memset(adjacency.mCounts.data(), 0, mesh_vertex_count * sizeof(uint32_t));

	for (size_t i = 0; i < static_cast<size_t>(mesh_index_count); ++i) {
		assert(getIndex(i) < mesh_vertex_count);
		adjacency.mCounts[getIndex(i)]++;
	}

	// fill offset table
	uint32_t offset = 0;

	for (size_t i = 0; i < static_cast<size_t>(mesh_vertex_count); ++i) {
		adjacency.mOffsets[i] = offset;
		offset += adjacency.mCounts[i];
	}

	assert(offset == mesh_index_count);

	// fill face data
	size_t curent_face_start_index = 0;
	for (size_t c = 0; c < mesh_face_count; ++c) {
		for(int i = 0; i < face_vertex_counts[c]; ++i) {
			adjacency.mFaceData[adjacency.mOffsets[getIndex(curent_face_start_index + i)]++] = uint32_t(c);
		}

		curent_face_start_index += face_vertex_counts[c];
	}

	// fix offsets that have been disturbed by the previous pass
	for (size_t i = 0; i < static_cast<size_t>(mesh_vertex_count); ++i) {
		assert(adjacency.mOffsets[i] >= adjacency.mCounts[i]);
		adjacency.mOffsets[i] -= adjacency.mCounts[i];
	}

	// neighbor indices data
	for(size_t i = 0; i < adjacency.mCounts.size(); ++i) {
		uint32_t count = adjacency.mCounts[i];
		uint32_t offset = adjacency.mOffsets[i];

	}

	adjacency.mValid = true;

	return adjacency;
}

void UsdGeomMeshFaceAdjacency::invalidate() {
	mCounts.clear();
	mOffsets.clear();
	mFaceData.clear();
	mPointToVerticesMap.clear();
	mValid = false;
	mHash = 0;
}

size_t UsdGeomMeshFaceAdjacency::getHash() const {
	if(!isValid()) return 0;

	if(mHash == 0) {
		for(size_t i = 0; i < mCounts.size(); ++i) mHash += mCounts[i]*i;
		mHash += mCounts.size();

		for(size_t i = 0; i < mOffsets.size(); ++i) mHash += mOffsets[i]*i;
		mHash += mOffsets.size();

		for(size_t i = 0; i < mFaceData.size(); ++i) mHash += mFaceData[i]*i;
		mHash += mFaceData.size();
	}

	return mHash;
}

bool UsdGeomMeshFaceAdjacency::isValid() const { 
	return mValid && !mCounts.empty() && !mOffsets.empty() && !mFaceData.empty(); 
}

std::string UsdGeomMeshFaceAdjacency::toString() const {
	if(!isValid()) return "Invalid UsdGeomMeshFaceAdjacency";

	std::string s = "[\n";
	for(size_t i = 0; i < mCounts.size(); ++i) {
		s += std::to_string(i) + ":[";
		uint32_t count = mCounts[i];
		uint32_t offset = mOffsets[i];
		for(uint32_t j = 0; j < count; ++j) {
			s += std::to_string(mFaceData[j]) + " ";
		}
		s += "]\n";
	}

	s += "]\n";
	return s;
}

} // namespace Piston