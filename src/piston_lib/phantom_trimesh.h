#ifndef PISTON_LIB_PHANTOM_TRIMESH_H_
#define PISTON_LIB_PHANTOM_TRIMESH_H_

#include "framework.h"
#include "common.h"
#include "kdtree.hpp"
#include "serializable_data.h"

#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/base/gf/matrix3f.h>

#include <limits>
#include <string>
#include <array>
#include <unordered_map>
#include <memory>
#include <mutex>


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

class SerializablePhantomTrimesh;

// PhantomTrimesh is a special kind of structure that holds virtual triangles that references existing UsdGeomMesh vertices.
// We use this structure to bind hair curves to mesh regions that influences control points.

class PhantomTrimesh {
	public:
		static constexpr size_t kInvalidTriFaceID = std::numeric_limits<size_t>::max();
		using PxrIndexType = int;
		using UniquePtr = std::unique_ptr<PhantomTrimesh>;

		struct TriFace {
			using IndicesList = std::array<PxrIndexType, 3>;
			static constexpr PxrIndexType kInvalidVertexID = std::numeric_limits<PxrIndexType>::max();

			TriFace(): indices{kInvalidVertexID} { }
			TriFace(PxrIndexType a, PxrIndexType b, PxrIndexType c): indices{a, b, c} { }
			TriFace(const std::array<PxrIndexType, 3>& d): indices{d} { }

			bool isValid() const { return indices[0] != kInvalidVertexID && indices[1] != kInvalidVertexID && indices[2] != kInvalidVertexID; }

			const pxr::GfVec3f& getRestNormal() const { return restNormal; }

			const IndicesList&  getIndices() const { return indices; }

			const PxrIndexType& operator[](size_t index) const { return indices[index]; }

			size_t calcHash() const {
				size_t hash = 0;
				
				hash += indices[0] + indices[1]*2 + indices[2]*3;
				hash += size_t(restNormal[0] * 111.f) + size_t(restNormal[1] * 222.f) + size_t(restNormal[2] * 333.f);

				return hash;
			}

			IndicesList 	indices;
			pxr::GfVec3f 	restNormal;
		};

		PhantomTrimesh() : mValid(false) {};

	public:
		static PhantomTrimesh::UniquePtr create();

		bool init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		const pxr::VtArray<pxr::GfVec3f>& getRestPositions() const { return mUsdMeshRestPositions; }
		const pxr::VtArray<pxr::GfVec3f>& getLivePositions() const { return mUsdMeshLivePositions; }

		uint32_t getOrCreate(PxrIndexType a, PxrIndexType b, PxrIndexType c) const;

		const std::vector<TriFace>& getFaces() const { return mFaces; }
		const TriFace& getFace(const uint32_t id) const { return mFaces[id]; }
		size_t getFaceCount() const { return mFaces.size(); }

		bool projectPoint(const pxr::GfVec3f& pt, const uint32_t face_id, float& u, float& v) const;
		bool projectPoint(const pxr::GfVec3f& pt, const uint32_t face_id, float& u, float& v, float& dist) const;
		bool intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, uint32_t face_id, float& u, float& v) const;
		bool intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, uint32_t face_id, float& u, float& v, float& dist) const;

		pxr::GfVec3f getInterpolatedRestPosition(const uint32_t face_id, const float u, const float v) const;
		pxr::GfVec3f getInterpolatedLivePosition(const uint32_t face_id, const float u, const float v) const;

		pxr::GfVec3f getFaceRestCentroid(const uint32_t face_id) const;

		const pxr::GfVec3f& getFaceRestNormal(const uint32_t face_id) const;
		pxr::GfVec3f getFaceLiveNormal(const uint32_t face_id) const;

		bool isValid() const { return mValid; }
		void invalidate();

		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default()) const;

		size_t calcHash() const;

	private:
		mutable pxr::VtArray<pxr::GfVec3f> 						mUsdMeshRestPositions;
		mutable pxr::VtArray<pxr::GfVec3f> 						mUsdMeshLivePositions;

		mutable std::unordered_map<std::array<PxrIndexType, 3>, size_t, IndicesArrayHasher<PxrIndexType, 3>> mFaceMap;
		mutable std::vector<TriFace> 							mFaces;

		bool                                        			mValid;

		friend class SerializablePhantomTrimesh;
};

void to_json(json& j, const PhantomTrimesh::TriFace& face);
void from_json(const json& j, PhantomTrimesh::TriFace& face);


class SerializablePhantomTrimesh: public SerializableDeformerDataBase {
	public:
		using UniquePtr = std::unique_ptr<SerializablePhantomTrimesh>;

		SerializablePhantomTrimesh();

		bool buildInPlace(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		const PhantomTrimesh* getTrimesh() const;

		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		typename PhantomTrimesh::UniquePtr	mpTrimesh;
};


} // namespace Piston

#endif // PISTON_LIB_PHANTOM_TRIMESH_H_