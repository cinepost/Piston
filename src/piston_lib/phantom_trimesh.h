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
#include <memory>


namespace Piston {

template<typename IndexType, size_t length> 
struct IndicesArrayHasher {
	using IndicesList = std::array<IndexType, length>;

	std::size_t operator()(const IndicesList& a) const {
		std::size_t h = 0;

		for (auto e : a) {
			h ^= std::hash<IndexType>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2); 
		}
		return h;
	}   
};

// PhantomTrimesh is a special kind of structure that holds virtual triangles that references existing UsdGeomMesh vertices.
// We use this structure to bind hair curves to mesh regions that influences control points.

template<typename IndexType> 
class PhantomTrimesh {
	public:
		static constexpr size_t kInvalidTrifaceID = std::numeric_limits<size_t>::max();

		using SharedPtr = std::shared_ptr<PhantomTrimesh>;

		struct Triface {
			using IndicesList = std::array<IndexType, 3>;
			static constexpr IndexType kInvalidVertexID = std::numeric_limits<IndexType>::max();

			Triface(): indices{kInvalidVertexID} { }
			Triface(IndexType a, IndexType b, IndexType c): indices{a, b, c} { }
			Triface(const std::array<IndexType, 3>& d): indices{d} { }

			bool isValid() const { return indices[0] != kInvalidVertexID && indices[1] != kInvalidVertexID && indices[2] != kInvalidVertexID; }

			IndicesList indices;

			pxr::GfVec3f restNormal;
		};

	private:
		PhantomTrimesh() : mValid(false) {};

	public:
		using TrifaceRetType = typename std::pair<Triface*, size_t>;

		static PhantomTrimesh::SharedPtr create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		const pxr::VtArray<pxr::GfVec3f>& getRestPositions() const { return mUsdMeshRestPositions; }

		TrifaceRetType getOrCreate(IndexType a, IndexType b, IndexType c);

		bool isValid() const { return mValid; }

	private:
		pxr::VtArray<pxr::GfVec3f> 								mUsdMeshRestPositions;

		std::unordered_map<std::array<IndexType, 3>, size_t, IndicesArrayHasher<IndexType, 3>> mFaceMap;
		std::vector<Triface> 									mFaces;

		bool                                        			mValid;
};

} // namespace Piston

#endif // PISTON_LIB_PHANTOM_TRIMESH_H_