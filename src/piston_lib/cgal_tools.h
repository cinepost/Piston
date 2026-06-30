#ifndef PISTON_LIB_CGAL_TOOLS_H_
#define PISTON_LIB_CGAL_TOOLS_H_

#define CGAL_DO_NOT_USE_BOOST_MP

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

// Pixar USD Includes
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/base/vt/array.h>

namespace Piston {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

// Helper function to build CGAL surface mesh from a USD Mesh Prim
Surface_mesh build_cgal_mesh_from_usd(const pxr::UsdGeomMesh& usd_mesh, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

}  // namespace Piston

#endif  // PISTON_LIB_CGAL_TOOLS_H_