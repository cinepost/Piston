#ifndef PISTON_LIB_MESH_SUBDIV_H_
#define PISTON_LIB_MESH_SUBDIV_H_

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/base/vt/array.h>

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/topologyRefiner.h>
#include <opensubdiv/far/primvarRefiner.h>
#include <opensubdiv/bfr/surface.h>

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

struct OsdFaceCoord {
	pxr::GfVec2f uv = {0.0f, 0.0f};
	int 	subFaceId = 0; // 0 for tris and quads

	OsdFaceCoord() = default;
	OsdFaceCoord(float _u ,float _v, int _sub_face_id = 0): uv(_u, _v), subFaceId(_sub_face_id) {} 
	OsdFaceCoord(const pxr::GfVec2f& _uv, int _sub_face_id = 0): uv(_uv), subFaceId(_sub_face_id) {} 
};

class UsdPrimHandle;
class UsdGeomMeshFaceAdjacency;

OsdFaceCoord calculateOsdCoordinates(uint32_t prim_id, const pxr::GfVec3f& pt, const UsdGeomMeshFaceAdjacency& adjacency, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions);

class PersistentMeshEvaluator {
	public:
		using UniquePtr = std::unique_ptr<PersistentMeshEvaluator>;

		PersistentMeshEvaluator(): mIsInitialized(false), mFaceCount(0) {};
	    ~PersistentMeshEvaluator();

	    // protect the underlying raw OpenSubdiv pointer
	    PersistentMeshEvaluator(const PersistentMeshEvaluator&) = delete;
	    PersistentMeshEvaluator& operator=(const PersistentMeshEvaluator&) = delete;

	    bool init(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode rest_time_code = pxr::UsdTimeCode::Default()); 
	    [[nodiscard]] bool isInitialized() const { return mIsInitialized; }
	    
	   	const std::vector<OpenSubdiv::Bfr::Surface<float>>& getSurfaceCache() const;

	   	OpenSubdiv::Bfr::Parameterization getSurfaceParameterization(const OpenSubdiv::Bfr::Surface<float>& surface) const {
	   		return surface.GetParameterization();
	   	}

	   	void getPrimControlPoints(uint32_t prim_id, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, std::vector<pxr::GfVec3f>& control_points) const;

	   	std::vector<float> getSurfaceScratchBuffer() const {
	   		return std::move(std::vector<float>(mMaxPatchPoints * 3));
	   	}

	    // Slow not optimized version
	    void evaluateSurface(uint32_t prim_id, const std::vector<OpenSubdiv::Bfr::Surface<float>>& surfaceCache, const OsdFaceCoord& pt, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, 
	    	pxr::GfVec3f& limitPos, pxr::GfVec3f& limitTangentU, pxr::GfVec3f& limitTangentV);

	    // Optimized version for querying all target points on a primitive
	    void evaluateSurface(uint32_t prim_id, const std::vector<OpenSubdiv::Bfr::Surface<float>>& surfaceCache, const std::vector<OsdFaceCoord>& points, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, 
	    	std::vector< pxr::GfVec3f>& limitPositions, std::vector< pxr::GfVec3f>& limitTangentUs, std::vector<pxr::GfVec3f>& limitTangentVs);

		void evaluateSurface(uint32_t prim_id, const std::vector<OpenSubdiv::Bfr::Surface<float>>& surfaceCache, const std::vector<OsdFaceCoord>& points, const pxr::VtArray<pxr::GfVec3f>& meshPointPositions, 
	    	std::vector< pxr::GfVec3f>& limitPositions, std::vector< pxr::GfVec3f>& limitTangentUs, std::vector<pxr::GfVec3f>& limitTangentVs, std::vector<float>& faceScratchBuffer);

	    static UniquePtr create();

	private:
	    bool mIsInitialized = false;
	    uint32_t mFaceCount = 0;

	    OpenSubdiv::Far::TopologyRefiner* mpRefiner = nullptr;

		// Long lived surface cache for on-the-fly subdivision evaluations
	    mutable std::vector<OpenSubdiv::Bfr::Surface<float>> mSurfaceCache;
	    mutable int mMaxPatchPoints = 0; // used for patch points scratch buffer
};

class PersistentMeshRefiner {
	public:
		using UniquePtr = std::unique_ptr<PersistentMeshRefiner>;

	    PersistentMeshRefiner(): mIsInitialized(false), mMaxLevel(0) {};
	    ~PersistentMeshRefiner();

	    // protect the underlying raw OpenSubdiv pointer
	    PersistentMeshRefiner(const PersistentMeshRefiner&) = delete;
	    PersistentMeshRefiner& operator=(const PersistentMeshRefiner&) = delete;

		bool init(const UsdPrimHandle& prim_handle, uint8_t maxLevel, const std::string& rest_p_name, pxr::UsdTimeCode rest_time_code, bool linear_rest_interp = true);
		bool isInitialized() const { return mIsInitialized; }

		/**
     	 * retrieves face and vertex IDs from the refined mesh that belong to a specific source face ID.
     	 */

		void getSubdividedPrimsFromSource(int sourceFaceId, std::vector<int>& outFaceIds) const;
		void getSubdividedPrimsAndVerticesFromSource(int sourceFaceId, std::vector<int>& outFaceIds, std::vector<int>& outVertexIds) const;

	    void querySubdividedPoints(int sourceFaceId, std::vector<pxr::GfVec3f>& points, uint32_t& count) const;

	    OpenSubdiv::Far::TopologyRefiner* getRefiner() const { return mpRefiner; }
	    [[nodiscard]] uint8_t getMaxLevel() const { return mMaxLevel; }

	    [[nodiscard]] bool isValidSourceMesh() const;
	    [[nodiscard]] bool isValidOutputMesh() const;

	    const pxr::UsdGeomMesh& getSourceMesh() const { return mSourceMesh; }
	    const pxr::UsdGeomMesh& getOutputMesh() const;

	    [[nodiscard]] bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) const;

	    static UniquePtr create();

	protected:
		void clear();

	private:
	    bool mIsInitialized = false;
	    uint8_t mMaxLevel = 0;

	    pxr::UsdStageRefPtr mpStage;
	    mutable pxr::UsdGeomMesh mSourceMesh;
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