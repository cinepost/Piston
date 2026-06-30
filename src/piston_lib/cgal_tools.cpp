#include "cgal_tools.h"


namespace Piston {

Surface_mesh build_cgal_mesh_from_usd(const pxr::UsdGeomMesh& usd_mesh, pxr::UsdTimeCode time_code) {
    Surface_mesh cgal_mesh;

    // 1. Fetch data attributes from the USD Mesh
    pxr::VtArray<pxr::GfVec3f> usd_points;
    pxr::VtArray<int> face_counts;
    pxr::VtArray<int> face_indices;

    usd_mesh.GetPointsAttr().Get(&usd_points, time_code);
    usd_mesh.GetFaceVertexCountsAttr().Get(&face_counts, time_code);
    usd_mesh.GetFaceVertexIndicesAttr().Get(&face_indices, time_code);

    // 2. Add Vertices to CGAL
    std::vector<Surface_mesh::Vertex_index> cgal_vertices;
    cgal_vertices.reserve(usd_points.size());

    for (const auto& pt : usd_points) {
        // Convert USD float vector (GfVec3f) to CGAL double-based Point_3
        cgal_vertices.push_back(cgal_mesh.add_vertex(Point_3(pt[0], pt[1], pt[2])));
    }

    // 3. Add Faces to CGAL
    size_t index_offset = 0;
    for (int count : face_counts) {
        std::vector<Surface_mesh::Vertex_index> face_vertices;
        face_vertices.reserve(count);

        for (int i = 0; i < count; ++i) {
            int vertex_idx = face_indices[index_offset + i];
            face_vertices.push_back(cgal_vertices[vertex_idx]);
        }
        
        // Add polygon face to the halfedge structure
        cgal_mesh.add_face(face_vertices);
        index_offset += count;
    }

    // 4. CRITICAL STEP: Triangulate the mesh if it contains quads or ngons
    // This is mandatory for using PMP::Side_of_triangle_mesh for the zorbing ball filtering
    CGAL::Polygon_mesh_processing::triangulate_faces(cgal_mesh);

    return cgal_mesh;
}

/*
// Example usage: Open a USD stage and extract a mesh prim
auto stage = pxr::UsdStage::OpenFromFile("my_scene.usd");
auto mesh_prim = pxr::UsdGeomMesh::Get(stage, pxr::SdfPath("/World/BaseMeshObject"));

if (mesh_prim) {
    Surface_mesh base_mesh = build_cgal_mesh_from_usd(mesh_prim);
    // 'base_mesh' is now ready for Side_of_triangle_mesh classification!
}
*/

}  // namespace Piston