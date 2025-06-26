#ifndef PISTON_LIB_PHANTOM_TRIMESH_H_
#define PISTON_LIB_PHANTOM_TRIMESH_H_

#include "framework.h"
#include "common.h"
#include "kdtree.hpp"

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

			const IndicesList&  getIndices() const { return indices; }

			const IndexType& operator[](size_t index) const { return indices[index]; }

			IndicesList indices;

			pxr::GfVec3f 	restNormal;
		};

	protected:
		PhantomTrimesh() : mValid(false) {};

	public:
		static PhantomTrimesh::SharedPtr create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		const pxr::VtArray<pxr::GfVec3f>& getRestPositions() const { return mUsdMeshRestPositions; }
		const pxr::VtArray<pxr::GfVec3f>& getLivePositions() const { return mUsdMeshLivePositions; }


		uint32_t getOrCreate(IndexType a, IndexType b, IndexType c);

		const std::vector<TriFace>& getFaces() const { return mFaces; }
		const TriFace& getFace(uint32_t id) const { return mFaces[id]; }
		size_t getFaceCount() const { return mFaces.size(); }

		bool projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v) const;
		bool projectPoint(const pxr::GfVec3f& pt, uint32_t face_id, float& u, float& v, float& dist) const;
		bool intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, uint32_t face_id, float& u, float& v) const;
		bool intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, uint32_t face_id, float& u, float& v, float& dist) const;

		pxr::GfVec3f getInterpolatedRestPosition(uint32_t face_id, float u, float v) const;
		pxr::GfVec3f getInterpolatedLivePosition(uint32_t face_id, float u, float v) const;

		bool isValid() const { return mValid; }
		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

	private:
		pxr::VtArray<pxr::GfVec3f> 								mUsdMeshRestPositions;
		pxr::VtArray<pxr::GfVec3f> 								mUsdMeshLivePositions;

		std::unordered_map<std::array<IndexType, 3>, size_t, IndicesArrayHasher<IndexType, 3>> mFaceMap;
		std::vector<TriFace> 									mFaces;

		bool                                        			mValid;
};

} // namespace Piston

#endif // PISTON_LIB_PHANTOM_TRIMESH_H_