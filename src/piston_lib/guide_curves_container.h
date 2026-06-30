#ifndef PISTON_LIB_GUIDE_CURVES_CONTAINER_H_
#define PISTON_LIB_GUIDE_CURVES_CONTAINER_H_

#include "framework.h"
#include "common.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/gf/vec3f.h>

#include <memory>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>

namespace Piston {

class GuideCurvesContainer : public std::enable_shared_from_this<GuideCurvesContainer> {
	public:
		using UniquePtr = std::unique_ptr<GuideCurvesContainer>;

	public:
		static UniquePtr create();

		bool init(const UsdPrimHandle& prim_handle, const std::string& rest_attr_name, pxr::UsdTimeCode reference_time_code, const pxr::VtArray<pxr::GfVec3f>* pRestPointsDataExt = nullptr, const pxr::VtArray<pxr::GfVec3f>* pLivePointsDataExt = nullptr);

		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force);

		size_t getCurvesCount() const { return mCurvesCount; }
		const pxr::VtArray<int>& getCurveVertexCounts() const { return mCurveVertexCounts.AsConst(); } 
		size_t getCurveVertexCount(size_t curve_id) const { assert(curve_id < mCurveVertexCounts.size()); return mCurveVertexCounts.AsConst()[curve_id]; } 


		const pxr::VtArray<pxr::GfVec3f>& getRestCurvePoints() const { return mRestCurvePoints.AsConst(); }
		const pxr::VtArray<pxr::GfVec3f>& getLiveCurvePoints() const { return mLiveCurvePoints.AsConst(); }

		const std::vector<uint32_t>& getCurveOffsets() const { return mCurveOffsets; }
		size_t getCurveVertexOffset(size_t curve_id) const { assert(curve_id < mCurveOffsets.size()); return mCurveOffsets[curve_id]; }

		const pxr::GfVec3f& getCurveRestPoint(uint32_t curve_id, uint32_t vertex_id) const;
		const pxr::GfVec3f& getCurveLivePoint(uint32_t curve_id, uint32_t vertex_id) const;

		size_t getTotalVertexCount() const { return mRestCurvePoints.AsConst().size(); }


		void getVeticesAtLength(uint32_t curve_index, float len, uint32_t& v0, uint32_t& v1) const;
		void getVeticesAtU(uint32_t curve_index, float u, uint32_t& v0, uint32_t& v1) const;

		// Two closest vertices and respective weights at distance from root
		void getVeticesWithWeightsAtLength(uint32_t curve_index, float len, uint32_t& v0, uint32_t& v1, float& w0, float& w1) const;
		
		// Two closest vertices and respective weights at U (curve parametrization).
		void getVeticesWithWeightsAtU(uint32_t curve_index, float u, uint32_t& v0, uint32_t& v1, float& w0, float& w1) const;

		pxr::GfVec3f getPointAtU(uint32_t curve_index, float u, bool live) const;

		float getRestCurveLength(uint32_t curve_index) const { assert(curve_index < mRestCurveLengths.size()); return mRestCurveLengths[curve_index]; }

	protected:
		GuideCurvesContainer();

	private:
		size_t                                  mCurvesCount;
		pxr::VtArray<int> 						mCurveVertexCounts;
		std::vector<uint32_t> 					mCurveOffsets;

		pxr::VtArray<pxr::GfVec3f>              mRestCurvePoints;
		pxr::VtArray<pxr::GfVec3f>              mLiveCurvePoints;

		std::vector<float>                      mRestCurveLengths; // Per curve lenghts

		bool                                    mExternalRestPointDataSource = false;
		bool                                    mExternalLivePointDataSource = false;

		pxr::UsdTimeCode 						mLastUpdateTimeCode;
};

// KDTree

class USDCurveKDTree {
	public:
		enum class CurvePositionType {
	    	Internal,
	    	Root,  // First point of the curve
	    	Tip    // Last point of the curve
		};

		// Structure linking structural metadata back to the USD curve source
		struct CurvePoint {
		    pxr::GfVec3f position;
		    size_t curveIndex;
		    size_t segmentIndex;
		    CurvePositionType positionType;
		};

		// Heap element to track current closest neighbors during traversal
		struct KNNCandidate {
		    CurvePoint point;
		    float distSq;

		    // Max-heap requires the element with the LARGEST distance at the top
		    bool operator<(const KNNCandidate& other) const {
		        return distSq < other.distSq;
		    }
		};

		struct KDNode {
		    CurvePoint point;
		    std::shared_ptr<KDNode> left;
		    std::shared_ptr<KDNode> right;

		    KDNode(const CurvePoint& pt) : point(pt), left(nullptr), right(nullptr) {}
		};

	public:
    	USDCurveKDTree(const pxr::UsdGeomCurves& usdCurves, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());
    	USDCurveKDTree(const GuideCurvesContainer* pCurvesContainer);

    	// Find any nearest point on any curve
    	bool findNearestPoint(const pxr::GfVec3f& query, CurvePoint& result, float& outDistance, std::optional<size_t> ignore_curve_id = std::nullopt) const;

    	// Find nearest curve root (start point)
    	bool findNearestCurveRoot(const pxr::GfVec3f& query, CurvePoint& result, float& outDistance, std::optional<size_t> ignore_curve_id = std::nullopt) const ;

    	// Find nearest curve tip (end point)
    	bool findNearestCurveTip(const pxr::GfVec3f& query, CurvePoint& result, float& outDistance, std::optional<size_t> ignore_curve_id = std::nullopt) const;


    	// Find K-Nearest overall points
    	std::vector<std::pair<CurvePoint, float>> findKNearestPoints(const pxr::GfVec3f& query, size_t k, std::optional<size_t> ignore_curve_id = std::nullopt) const;

    	// Find K-Nearest curve roots
    	std::vector<std::pair<CurvePoint, float>> findKNearestCurveRoots(const pxr::GfVec3f& query, size_t k, std::optional<size_t> ignore_curve_id = std::nullopt) const;

    	// Find K-Nearest curve tips
    	std::vector<std::pair<CurvePoint, float>> findKNearestCurveTips(const pxr::GfVec3f& query, size_t k, std::optional<size_t> ignore_curve_id = std::nullopt) const;

	private:
    	std::shared_ptr<KDNode> buildTreeRec(std::vector<CurvePoint>& points, int depth, int start, int end);

    	// Centralized search execution with selective pruning filters
    	void searchNearestRec(const std::shared_ptr<KDNode>& pNode, 
			const pxr::GfVec3f& query, int depth, const std::vector<CurvePositionType>& allowedTypes,
			const std::optional<size_t>& ignore_curve_id, std::shared_ptr<KDNode>& bestNode, float& bestDistSq) const;

		// Recursive K-NN search using a max-priority queue for strict pruning
		void searchKNearestRec(const std::shared_ptr<KDNode>& pNode, const pxr::GfVec3f& query, 
			int depth, size_t k, const std::vector<CurvePositionType>& allowedTypes, const std::optional<size_t>& ignore_curve_id, std::priority_queue<KNNCandidate>& maxHeap) const;

    	// Utility wrapper to convert max-heap results to a sorted proximity vector
	    std::vector<std::pair<CurvePoint, float>> finalizeKNN(std::priority_queue<KNNCandidate>& maxHeap) const;

    private:
    	std::shared_ptr<KDNode> mpRootNode;
};


} // namespace Piston

#endif // PISTON_LIB_GUIDE_CURVES_CONTAINER_H_