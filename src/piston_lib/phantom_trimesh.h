#ifndef PISTON_LIB_PHANTOM_TRIMESH_H_
#define PISTON_LIB_PHANTOM_TRIMESH_H_

#include "framework.h"
#include "kdtree.hpp"
#include "base_hair_deformer.h"

#include <pxr/usd/usdGeom/mesh.h>

#include <limits>
#include <string>
#include <array>
#include <unordered_map>


namespace Piston {

// PhantomTrimesh is a special kind of structure that holds virtual triangles that references existing UsdGeomMesh vertices.
// We use this structure to bind hair curves to mesh regions that influences control points.

template<typename VertexIndexType> 
class PhantomTrimesh {
	public: 
		struct Triface {
			static constexpr VertexIndexType kInvalidID = std::numeric_limits<VertexIndexType>::max();

			Triface(): indices{kInvalidID} { }
			Triface(VertexIndexType a, VertexIndexType b, VertexIndexType c): indices{a, b, c} { }
			Triface(const std::array<VertexIndexType, 3>& d): indices{d} { }

			bool isValid() const { return indices[0] != kInvalidID && indices[1] != kInvalidID && indices[2] != kInvalidID; }

			std::array<VertexIndexType, 3> indices;
		};

	public:
		PhantomTrimesh() : mValid(false) {};
		PhantomTrimesh(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		size_t getOrCreate(VertexIndexType a, VertexIndexType b, VertexIndexType c);

	private:
		void calculateRestTrifaceNormals() const;

	private:
		pxr::VtArray<pxr::GfVec3f> 					mUsdMeshRestPositions;
		mutable pxr::VtArray<pxr::GfVec3f> 			mRestNormals;

		std::unordered_map<Triface, size_t> 		mFaceMap;
		std::vector<Triface> 						mFaces;

		bool                                        mValid;
};

} // namespace Piston

#endif // PISTON_LIB_PHANTOM_TRIMESH_H_