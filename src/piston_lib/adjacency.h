#ifndef PISTON_LIB_ADJACENCY_H_
#define PISTON_LIB_ADJACENCY_H_

#include "framework.h"
#include "serializable_data.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>


namespace Piston {

class SerializableUsdGeomMeshFaceAdjacency;

class UsdGeomMeshFaceAdjacency {
	public:
		using SharedPtr = std::shared_ptr<UsdGeomMeshFaceAdjacency>;
		using PxrIndexType = int;

		UsdGeomMeshFaceAdjacency();
		static SharedPtr create();

		bool init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code = pxr::UsdTimeCode::Default());

		bool isValid() const;

		void invalidate();

		size_t getHash() const { return isValid() ? mHash : 0; }

		uint32_t getNeighborsCount(uint32_t idx) const;
		uint32_t getNeighborsOffset(uint32_t idx) const;
		uint32_t getNeighborPrim(uint32_t prim_offset) const;


		uint32_t getFaceVertexOffset(uint32_t face_idx) const;
		uint32_t getFaceVertexCount(uint32_t face_idx) const;
		PxrIndexType getFaceVertex(uint32_t vtx_idx) const;
		PxrIndexType getFaceVertex(uint32_t face_idx, uint32_t local_vertex_index) const;

		uint32_t getVertexFaceId(uint32_t vtx) const;

		size_t getVertexCount() const { return mVertexCount; }
		size_t getFaceCount() const { return mFaceCount; }
		size_t getMaxFaceVertexCount() const { return mMaxFaceVertexCount; }

		const std::vector<uint32_t>& getPrimData() const { return mPrimData; }

		const std::pair<PxrIndexType, PxrIndexType>& getCornerVertexPair(uint32_t offset) const;

		std::string toString() const;

		size_t calcHash() const;
 
	protected:
		size_t mFaceCount;
		size_t mVertexCount;
		size_t mMaxFaceVertexCount;

    	std::vector<uint32_t> mCounts;   	// per vertex neighbor faces counts
    	std::vector<uint32_t> mOffsets;  	// per vertex neighbor offsets in data array
    	std::vector<uint32_t> mPrimData;    // neighbor prim indices
    	std::vector<uint32_t> mVtxToFace;   // simple reverse relations vertex to face
    	std::vector<std::pair<PxrIndexType, PxrIndexType>> mCornerVertexData;	// neighbor corner vertex pair indices

    	std::vector<PxrIndexType> mSrcFaceVertexIndices;
		std::vector<PxrIndexType> mSrcFaceVertexCounts;
		std::vector<uint32_t> 	   mSrcFaceVertexOffsets;
    
    	bool mValid; // Set by Adjacency data builder!
    
    	mutable size_t mHash;

    	friend class SerializableUsdGeomMeshFaceAdjacency;
};

class SerializableUsdGeomMeshFaceAdjacency: public SerializableDeformerDataBase {
	public:
		using UniquePtr = std::unique_ptr<SerializableUsdGeomMeshFaceAdjacency>;

		SerializableUsdGeomMeshFaceAdjacency();

		bool buildInPlace(const UsdPrimHandle& prim_handle);

		const UsdGeomMeshFaceAdjacency* getAdjacency() const;
		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;
		
	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		UsdGeomMeshFaceAdjacency::SharedPtr	mpAdjacency;
};


inline std::string to_string(const UsdGeomMeshFaceAdjacency& a) {
	return a.toString();
}


} // namespace Piston

#endif // PISTON_LIB_ADJACENCY_H_