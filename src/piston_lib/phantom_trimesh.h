#ifndef PISTON_LIB_PHANTOM_TRIMESH_H_
#define PISTON_LIB_PHANTOM_TRIMESH_H_

#include "framework.h"
#include "kdtree.hpp"
#include "base_hair_deformer.h"

#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/base/gf/matrix3f.h>

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
		static constexpr size_t kInvalidTriFaceID = std::numeric_limits<size_t>::max();

		using SharedPtr = std::shared_ptr<PhantomTrimesh>;

		struct TriFace {
			using IndicesList = std::array<IndexType, 3>;
			static constexpr IndexType kInvalidVertexID = std::numeric_limits<IndexType>::max();

			TriFace(): indices{kInvalidVertexID} { }
			TriFace(IndexType a, IndexType b, IndexType c): indices{a, b, c} { }
			TriFace(const std::array<IndexType, 3>& d): indices{d} { }

			bool isValid() const { return indices[0] != kInvalidVertexID && indices[1] != kInvalidVertexID && indices[2] != kInvalidVertexID; }

			const pxr::GfVec3f& getRestNormal() const { return restNormal; }

			const IndexType& operator[](size_t index) const { return indices[index]; }

			IndicesList indices;

			pxr::GfVec3f 	restNormal;
			pxr::GfMatrix3f	alignMat;
		};

	protected:
		PhantomTrimesh() : mValid(false) {};

	public:
		static PhantomTrimesh::SharedPtr create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		const pxr::VtArray<pxr::GfVec3f>& getRestPositions() const { return mUsdMeshRestPositions; }

		size_t getOrCreate(IndexType a, IndexType b, IndexType c);

		bool projectPoint(const pxr::GfVec3f& pt, size_t triface_id) const;

		bool isValid() const { return mValid; }

	private:
		pxr::VtArray<pxr::GfVec3f> 								mUsdMeshRestPositions;

		std::unordered_map<std::array<IndexType, 3>, size_t, IndicesArrayHasher<IndexType, 3>> mFaceMap;
		std::vector<TriFace> 									mFaces;

		bool                                        			mValid;
};

} // namespace Piston

#endif // PISTON_LIB_PHANTOM_TRIMESH_H_