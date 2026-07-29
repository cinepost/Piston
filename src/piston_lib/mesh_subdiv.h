#ifndef PISTON_LIB_MESH_SUBDIV_H_
#define PISTON_LIB_MESH_SUBDIV_H_

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/base/vt/array.h>

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/topologyRefiner.h>
#include <opensubdiv/far/primvarRefiner.h>

#include <limits>
#include <string>
#include <array>
#include <iostream>
#include <vector>
#include <memory>
#include <mutex>


namespace Piston {

struct OpenSubdVertex {
    pxr::GfVec3f position;

    // Default constructor required by OpenSubdiv
    OpenSubdVertex() : position(0.0f, 0.0f, 0.0f) {}

    // Required minimal interface for OpenSubdiv Far::PrimvarRefiner
    void Clear(void* = 0) {
        position.Set(0.0f, 0.0f, 0.0f);
    }

    void AddWithWeight(OpenSubdVertex const& src, float weight) {
        position += src.position * weight;
    };
};

class PersistentMeshRefiner {
	public:
		using UniquePtr = std::unique_ptr<PersistentMeshRefiner>;

	    PersistentMeshRefiner(): mIsInitialized(false), mMaxLevel(0) {};

	    ~PersistentMeshRefiner() {
	        delete mpRefiner;
	    }

	    // protect the underlying raw OpenSubdiv pointer
	    PersistentMeshRefiner(const PersistentMeshRefiner&) = delete;
	    PersistentMeshRefiner& operator=(const PersistentMeshRefiner&) = delete;

		void init(const pxr::UsdGeomMesh& sourceMesh, uint8_t maxLevel, const std::string& rest_p_name, pxr::UsdTimeCode rest_time_code);

	    void querySubdividedPoints(int sourceFaceId, std::vector<pxr::GfVec3f>& points, uint32_t& count) const;

	    OpenSubdiv::Far::TopologyRefiner* getRefiner() const { return mpRefiner; }
	    uint8_t getMaxLevel() const { return mMaxLevel; }

	    bool isValidSourceMesh() const;
	    bool isValidOutputMesh() const;

	    const pxr::UsdGeomMesh& getSourceMesh() const { return mSourceMesh; }
	    const pxr::UsdGeomMesh& getSubdividedMesh() const;

	    void update(pxr::UsdTimeCode time_code) const;

	    static UniquePtr create();

	protected:
		void clear();

	private:
		/**
     	 * retrieves face and vertex IDs from the refined mesh that belong to a specific source face ID.
     	 */
		void getSubdividedPrimsFromSource(int sourceFaceId, std::vector<int>& outFaceIds, std::vector<int>& outVertexIds) const;

	private:
	    bool mIsInitialized = false;
	    uint8_t mMaxLevel = 0;

	    pxr::UsdStageRefPtr mpStage;
	    pxr::UsdGeomMesh mSourceMesh;
	    pxr::UsdGeomMesh mOutputMesh;
	    OpenSubdiv::Far::TopologyRefiner* mpRefiner = nullptr;

	    std::string mRestPosName;
	    pxr::UsdTimeCode mRestTimeCode;
	    
	    mutable pxr::UsdTimeCode mLastUpdateTimeCode;

	    mutable std::vector<OpenSubdVertex>  mTmpVertexBuffer;
	    mutable pxr::VtVec3fArray 			 mTmpOutPoints;

	   	friend class UsdPrimHandle;
};


void subdivideUsdMesh(const pxr::UsdGeomMesh& usdMesh, int maxLevel);

} // namespace Piston

#endif // PISTON_LIB_MESH_SUBDIV_H_