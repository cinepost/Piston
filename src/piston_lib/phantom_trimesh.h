#ifndef PISTON_LIB_PHANTOM_TRIMESH_H_
#define PISTON_LIB_PHANTOM_TRIMESH_H_

#include "framework.h"
#include "common.h"
#include "kdtree.hpp"
#include "serializable_data.h"
#include "logging.h"

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
		static constexpr uint32_t kInvalidTriFaceID = std::numeric_limits<uint32_t>::max();
		using PxrIndexType = int;
		using UniquePtr = std::unique_ptr<PhantomTrimesh>;

		struct TriFace {
			using IndicesList = std::array<PxrIndexType, 3>;
			static constexpr PxrIndexType kInvalidVertexID = std::numeric_limits<PxrIndexType>::max();

			enum class Flags: uint8_t {
				None 	= 0x0,      ///< None
				Bound 	= 0x1,      ///< Some curves are bound to this face

				Default = None
			};

			TriFace(): indices{kInvalidVertexID} { }
			TriFace(PxrIndexType a, PxrIndexType b, PxrIndexType c): indices{a, b, c} { }
			TriFace(const std::array<PxrIndexType, 3>& d): indices{d} { }

			bool isValid() const { return indices[0] != kInvalidVertexID && indices[1] != kInvalidVertexID && indices[2] != kInvalidVertexID; }

			//const pxr::GfVec3f& getRestNormal() const { return restNormal; }

			const IndicesList&  getIndices() const { return indices; }

			const PxrIndexType& operator[](size_t index) const { return indices[index]; }

			size_t calcHash() const {
				return static_cast<size_t>(indices[0]) + (static_cast<size_t>(indices[1]) << 16) + (static_cast<size_t>(indices[2]) << 32);
			}

			IndicesList 	indices;
			pxr::GfVec3f 	restNormal;
		};

		struct Tetrahedron {
			using IndicesList = std::array<PxrIndexType, 4>;
			static constexpr PxrIndexType kInvalidVertexID = std::numeric_limits<PxrIndexType>::max();

			Tetrahedron(): indices{kInvalidVertexID} { }
			Tetrahedron(PxrIndexType a, PxrIndexType b, PxrIndexType c, PxrIndexType d): indices{a, b, c, d} { }

			bool isValid() const { return !(
				indices[0] == indices[1] || indices[0] == indices[2] || indices[0] == indices[3] ||
				indices[0] == kInvalidVertexID || indices[1] == kInvalidVertexID || indices[2] == kInvalidVertexID || indices[3] == kInvalidVertexID); 
			}

			IndicesList 	indices;
		};

		PhantomTrimesh();

	public:
		static PhantomTrimesh::UniquePtr create();

		bool init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		size_t getPointsCount() const { return mPointsCount; }

		std::vector<TriFace::Flags>& getFaceFlags() { return mFaceFlags; }
		const std::vector<TriFace::Flags>& getFaceFlags() const { return mFaceFlags; }

		TriFace::Flags getFaceFlag(const uint32_t face_id) { assert(face_id < mFaceFlags.size()); return mFaceFlags[face_id]; }
		void setFaceFlag(const uint32_t face_id, const TriFace::Flags flag) { assert(face_id < mFaceFlags.size()); mFaceFlags[face_id] = flag; }

		uint32_t getFaceIDByIndices(PxrIndexType a, PxrIndexType b, PxrIndexType c) const;
		uint32_t getOrCreateFaceID(PxrIndexType a, PxrIndexType b, PxrIndexType c);
		uint32_t getOrCreateFaceID(const std::array<PxrIndexType, 3>& a);

		const std::vector<TriFace>& getFaces() const { return mFaces; }
		const TriFace& getFace(const uint32_t id) const { 
			if(id >= mFaces.size() ) {
				LOG_WRN << "PhantomTrimesh::getFace(" << id << ") while mFace.size() is " << mFaces.size();
			} 
			assert(id < mFaces.size()); return mFaces[id]; 
		}
		uint32_t getFaceCount() const { return static_cast<uint32_t>(mFaces.size()); }

		const std::vector<PxrIndexType>& getVertices() const { return mVertices; }
		size_t getVertexCount() const { return mVertices.size(); }

		bool isValid() const;
		void invalidate();

		bool buildTetrahedrons(const pxr::VtArray<pxr::GfVec3f>& positions);
		bool hasTetrahedrons() const { return !mTetrahedrons.empty() && (mTetrahedronCounts.size() == mTetrahedronOffsets.size() == mPointsCount); }
		const std::vector<Tetrahedron>& getTetrahedrons() const { return mTetrahedrons; }
		const Tetrahedron& getTetrahedron(size_t i) const { assert(i < mTetrahedrons.size()); return mTetrahedrons[i]; }

		size_t getPointConnectedTetrahedronsCount(size_t pt_index) const { assert(pt_index < mTetrahedronCounts.size()); return mTetrahedronCounts[pt_index];}
		uint32_t getPointConnectedTetrahedronIndex(size_t pt_index, size_t tetra_local_index) const;

		size_t calcHash() const;

	private:
		size_t                                                  mPointsCount;

		// TriFaces part
		std::unordered_map<std::array<PxrIndexType, 3>, size_t, IndicesArrayHasher<PxrIndexType, 3>> mFaceMap;
		std::vector<TriFace> 									mFaces;
		std::vector<TriFace::Flags>								mFaceFlags;
		mutable std::mutex										mFaceMapMutex;

		// Tetrahedrons part
		std::vector<Tetrahedron> 								mTetrahedrons;
		// maps points to tetrahedrons
		std::vector<uint32_t> 									mTetrahedronCounts;
		std::vector<uint32_t>                                   mTetrahedronOffsets;
		std::vector<uint32_t>                                   mTetrahedronIndices;

		//
		std::vector<PxrIndexType> 								mVertices;
		std::unordered_set<PxrIndexType> 						mTmpVertices; // this is used only to insert unused vertices into mVertices

		bool                                        			mIsValid;

		friend class SerializablePhantomTrimesh;
};

void to_json(json& j, const PhantomTrimesh::TriFace& face);
void from_json(const json& j, PhantomTrimesh::TriFace& face);


class SerializablePhantomTrimesh: public SerializableDeformerDataBase {
	public:
		using UniquePtr = std::unique_ptr<SerializablePhantomTrimesh>;

		SerializablePhantomTrimesh();

		bool buildInPlace(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());
		virtual bool isValid() const override { const std::lock_guard<std::mutex> lock(mMutex); return mpTrimesh && mpTrimesh->isValid(); }
		
		PhantomTrimesh* getTrimesh();
		const PhantomTrimesh* getTrimesh() const;

		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

		void setValid(bool state);

	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		typename PhantomTrimesh::UniquePtr	mpTrimesh;
};

using TriFaceFlags = PhantomTrimesh::TriFace::Flags;

inline TriFaceFlags operator& (TriFaceFlags a, TriFaceFlags b) { return static_cast<TriFaceFlags>(static_cast<uint8_t>(a)& static_cast<uint8_t>(b)); } \
inline bool is_set(TriFaceFlags val, TriFaceFlags flag) { return (val & flag) != static_cast<TriFaceFlags>(0); } \

} // namespace Piston

#endif // PISTON_LIB_PHANTOM_TRIMESH_H_