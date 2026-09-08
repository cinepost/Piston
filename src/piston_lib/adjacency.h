#ifndef PISTON_LIB_ADJACENCY_H_
#define PISTON_LIB_ADJACENCY_H_

#include "framework.h"
#include "serializable_data.h"

#include <memory>
#include <string>
#include <mutex>
#include <algorithm>

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/base/gf/lineSeg.h>


namespace Piston {

class SerializableUsdGeomMeshFaceAdjacency;

class UsdGeomMeshFaceAdjacency {
	public:
		using UniquePtr = std::unique_ptr<UsdGeomMeshFaceAdjacency>;
		using PxrIndexType = int;
		static const uint32_t kInvalidID = std::numeric_limits<uint32_t>::max();

		UsdGeomMeshFaceAdjacency();

		static UniquePtr create();

		bool init(const pxr::UsdGeomMesh& mesh, pxr::UsdTimeCode rest_time_code = pxr::UsdTimeCode::Default());

		bool isValid() const;

		void invalidate();

		size_t getHash() const { return isValid() ? mHash : 0; }

		uint32_t getNeighborsCount(uint32_t idx) const {
			assert(idx < mCounts.size());
			return mCounts[idx]; 
		}
		
		uint32_t getNeighborsOffset(uint32_t idx) const {
			assert(idx < mOffsets.size());
			return mOffsets[idx]; 
		}
		
		uint32_t getNeighborPrim(uint32_t prim_offset) const {
			assert(prim_offset < mPrimData.size());
			return mPrimData[prim_offset];
		}


		uint32_t getPrimVertexOffset(uint32_t prim_idx) const {
			assert(prim_idx < mSrcFaceVertexOffsets.size());
			return mSrcFaceVertexOffsets[prim_idx];
		}
		
		uint32_t getPrimVertexCount(uint32_t prim_idx) const {
			if(prim_idx >= mSrcFaceVertexCounts.size()) return 0;
			return static_cast<uint32_t>(mSrcFaceVertexCounts[prim_idx]);
		}
		
		
		PxrIndexType getPrimVertex(uint32_t vtx_idx) const {
			assert(vtx_idx < mSrcFaceVertexIndices.size());
			return mSrcFaceVertexIndices[vtx_idx];
		}
		
		PxrIndexType getPrimVertex(uint32_t prim_idx, uint32_t local_vertex_index) const {
			assert(prim_idx < mSrcFaceVertexCounts.size());
			assert(local_vertex_index < mSrcFaceVertexCounts[prim_idx]);
			return mSrcFaceVertexIndices[mSrcFaceVertexOffsets[prim_idx] + local_vertex_index];
		}

		uint32_t getVertexFaceId(uint32_t vtx) const;

		uint32_t getVertexCount() const { return mVertexCount; }
		uint32_t getPrimCount() const { return mFaceCount; }
		uint32_t getMaxFaceVertexCount() const { return mMaxFaceVertexCount; }

		const std::vector<uint32_t>& getPrimData() const { return mPrimData; }

		const std::pair<PxrIndexType, PxrIndexType>& getCornerVertexPair(uint32_t offset) const {
			assert(offset < mCornerVertexData.size());
			return mCornerVertexData[offset];
		}

		std::vector<uint32_t> getNeighborPrims(uint32_t idx) const;

		// Computes the combined score based on proximity and orientation alignment
		// lowest score means better match
		float evaluatePrimMatch(uint32_t prim_id, const pxr::GfVec3f& target, uint32_t common_point_idx, const pxr::VtArray<pxr::GfVec3f>& positions, float weightDistance = 0.5f, float weightOrientation = 0.5f) const;

		/*
		 * find best mesh prim contaning point with "common_point_idx" index to target point "target". returns lowest index prim 
		 */
		uint32_t findBestPrimFast(const pxr::GfVec3f target, uint32_t common_point_idx) const;

		/*
		 * find best mesh prim contaning point with "common_point_idx" index to target point "target". returns best oriented prim 
		 */
		uint32_t findBestPrimOriented(const pxr::GfVec3f target, uint32_t common_point_idx, const pxr::VtArray<pxr::GfVec3f>& positions) const;


		std::string toString() const;

		size_t calcHash() const;
 
	protected:
		uint32_t mFaceCount;
		uint32_t mVertexCount;
		uint32_t mMaxFaceVertexCount;

    	std::vector<uint32_t> mCounts;   	// per vertex neighbor faces counts
    	std::vector<uint32_t> mOffsets;  	// per vertex neighbor offsets in data array
    	std::vector<uint32_t> mPrimData;    // neighbor prim indices
    	std::vector<uint32_t> mVtxToFace;   // simple reverse relations vertex to face
    	std::vector<std::pair<PxrIndexType, PxrIndexType>> mCornerVertexData;	// neighbor corner vertex pair indices

    	std::vector<PxrIndexType> mSrcFaceVertexIndices;
		std::vector<PxrIndexType> mSrcFaceVertexCounts;
		std::vector<uint32_t> 	  mSrcFaceVertexOffsets;
    
    	bool mValid; // Set by Adjacency data builder!
    
    	mutable size_t mHash;

    	friend class SerializableUsdGeomMeshFaceAdjacency;
};

class SerializableUsdGeomMeshFaceAdjacency: public SerializableDeformerDataBase {
	public:
		using UniquePtr = std::unique_ptr<SerializableUsdGeomMeshFaceAdjacency>;

		SerializableUsdGeomMeshFaceAdjacency();

		bool buildInPlace(const UsdPrimHandle& prim_handle);
		virtual bool isValid() const override { const std::lock_guard<std::mutex> lock(mMutex); return mpAdjacency && mpAdjacency->isValid(); }

		const UsdGeomMeshFaceAdjacency* getAdjacency() const;
		const UsdGeomMeshFaceAdjacency* getAdjacencySubd() const;

		const UsdGeomMeshFaceAdjacency* getAdjacencyFinal() const {
			return mHasSubdividedAdjacencyData ? getAdjacencySubd() : getAdjacency();
		};

		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;
		
	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		bool mHasSubdividedAdjacencyData;
		UsdGeomMeshFaceAdjacency::UniquePtr	mpAdjacency;
		UsdGeomMeshFaceAdjacency::UniquePtr	mpAdjacencySubd;

};


inline std::string to_string(const UsdGeomMeshFaceAdjacency& a) {
	return a.toString();
}


} // namespace Piston

#endif // PISTON_LIB_ADJACENCY_H_