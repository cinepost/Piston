#include "geometry_tools.h"

namespace Piston {

bool buildGeometryFaceNormals(const pxr::UsdGeomMesh& mesh, std::vector<glm::vec3>& normals, pxr::UsdTimeCode time) {
	size_t points_count;
	
	std::vector<pxr::GfVec3f> points;

	if(!mesh.GetPointsAttr().Get<pxr::GfVec3f>(points.data(), time)) {
		return false;
	}

	//if (UsdAttribute attr = prim.GetAttribute(TfToken("myAttr"))){

	return true;
}

bool buildPhantomTriMesh(const pxr::UsdGeomMesh& mesh, std::vector<PhantomMeshTriface>& phantomMesh, pxr::UsdTimeCode time) {
	const size_t input_mesh_face_count = mesh.GetFaceCount(time);
	phantomMesh.clear();
	phantomMesh.reserve(input_mesh_face_count);

	pxr::VtArray<int> face_vertex_counts;
	if(!mesh.GetFaceVertexCountsAttr().Get(&face_vertex_counts, time)) {
		printf("Error getting face vertex counts for mesh !\n");
		return false;
	}

	pxr::VtArray<int> face_vertex_indices;
	if(!mesh.GetFaceVertexIndicesAttr().Get(&face_vertex_indices, time)) {
		printf("Error getting face vertex indices for mesh !\n");
		return false;
	}

	// triangulate if necessary
	size_t idx = 0;
	for(const int indices_count: face_vertex_counts) {
		printf("%d\n", indices_count);
		size_t _i = idx;
		idx += indices_count;

		// no trinagulation
		if(indices_count == 3) {
			phantomMesh.push_back({face_vertex_indices[_i], face_vertex_indices[_i+1], face_vertex_indices[_i+2]});
			continue;
		}

		// simple ear-clipping triangulation
		if(indices_count > 3) {
			const bool edge_count_is_even = indices_count % 2;
			std::vector<uint32_t> inner_indices;

			for(uint32_t i = 0; i < (edge_count_is_even ? (indices_count - 1) : (indices_count - 2)); i+=2) {

			}

		}

	} 

	return true;
}


} // namespace Piston