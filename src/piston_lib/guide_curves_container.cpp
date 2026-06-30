#include "guide_curves_container.h"
#include "logging.h"

#include <limits>
#include <cmath>


namespace Piston {

GuideCurvesContainer::GuideCurvesContainer(): mCurvesCount(0), mExternalRestPointDataSource(false), mExternalLivePointDataSource(false), mLastUpdateTimeCode(std::numeric_limits<double>::lowest()) {
	mCurveOffsets.reserve(1024);
}

GuideCurvesContainer::UniquePtr GuideCurvesContainer::create() {
	return GuideCurvesContainer::UniquePtr(new GuideCurvesContainer());
}

bool GuideCurvesContainer::init(const UsdPrimHandle& prim_handle, const std::string& rest_attr_name, pxr::UsdTimeCode rest_time_code, const pxr::VtArray<pxr::GfVec3f>* pRestPointsDataExt, const pxr::VtArray<pxr::GfVec3f>* pLivePointsDataExt) {
	if(!prim_handle.isBasisCurvesGeoPrim()) {
		return false;
	}

	auto geom_curves = pxr::UsdGeomCurves(prim_handle.getPrim());
	if(!geom_curves) {
		LOG_ERR << "Error getting curves geometry from " << prim_handle.getName() << " !";
		return false;
	}

	mCurvesCount = geom_curves.GetCurveCount(rest_time_code);
	if(mCurvesCount == 0) {
		LOG_ERR << "No curves exist in primitive " << prim_handle.getName() << " !";
		return false;
	}

	// Curves. Counts/offsets
	if(!geom_curves.GetCurveVertexCountsAttr().Get(&mCurveVertexCounts, rest_time_code)){
		LOG_ERR << "Error getting curves vertices counts from " << prim_handle.getName() << " !";
		return false;
	}
	assert(mCurveVertexCounts.size() == mCurvesCount);

	if(pRestPointsDataExt) {
		mRestCurvePoints= *pRestPointsDataExt;
		mExternalRestPointDataSource = true;
	} else {
		// Curve rest points
		if(rest_attr_name.empty() || !prim_handle.fetchAttributeValues<pxr::GfVec3f>(rest_attr_name, mRestCurvePoints, rest_time_code)) {
			if(!prim_handle.getPoints(mRestCurvePoints, rest_time_code)) {
				LOG_ERR << "Error getting curves \"rest\" points from " << prim_handle.getName() << " !";
				return false;
			}
		}
	}

	if(pLivePointsDataExt) {
		mLiveCurvePoints = *pLivePointsDataExt;
		mExternalLivePointDataSource = true;
	} else {
		mLiveCurvePoints = mRestCurvePoints;
	}

	// Calc offsets
	mCurveOffsets.resize(mCurvesCount);
	uint32_t total_vertex_count = 0u;
	for(size_t i = 0; i < mCurvesCount; ++i) {
		mCurveOffsets[i] = total_vertex_count;
		total_vertex_count += mCurveVertexCounts[i];
	}

	assert(mCurveVertexCounts.size() == mCurveOffsets.size());


	// Calc lenghts
	mRestCurveLengths.resize(mCurvesCount);
	std::fill(mRestCurveLengths.begin(), mRestCurveLengths.end(), 0.f);

	for(uint32_t curve_index = 0; curve_index < mCurvesCount; ++curve_index) {
		uint32_t pt_offset = mCurveOffsets[curve_index];
		for(uint32_t i = 0; i < (mCurveVertexCounts[curve_index] - 1); ++i) {
			mRestCurveLengths[curve_index] += (mRestCurvePoints[pt_offset] - mRestCurvePoints[pt_offset+1]).GetLength();
			pt_offset++;
		}
	}

	mLastUpdateTimeCode = rest_time_code;
	return true;
}

bool GuideCurvesContainer::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force) {
	assert(!mExternalLivePointDataSource);
	assert(prim_handle.isBasisCurvesGeoPrim());

	if(!force) {
		if(mLastUpdateTimeCode == time_code) return true;
		//if(!prim_handle.hasPositionsTimeSamples(mLastUpdateTimeCode, time_code)) return true;
	}

	// Curve live point positions
	if(!prim_handle.getPoints(mLiveCurvePoints, time_code)) {
		LOG_ERR << "Error getting curves point positions from " << prim_handle << " !";
		return false;
	}

	if(mLiveCurvePoints.size() != mRestCurvePoints.size()) {
		LOG_ERR << prim_handle.getPath() << " \"rest\" and \"live\" curves point positions count (" << mRestCurvePoints.size() << " vs " << mLiveCurvePoints.size() << " ) mismatch !";
		return false;
	}

	mLastUpdateTimeCode = time_code;
	return true;
}

const pxr::GfVec3f& GuideCurvesContainer::getCurveRestPoint(uint32_t curve_id, uint32_t vertex_id) const {
	assert(curve_id < mCurveVertexCounts.size() && curve_id < mCurveOffsets.size());
	assert(vertex_id < mCurveVertexCounts[curve_id]);

	const size_t global_vtx_id = mCurveOffsets[curve_id] + vertex_id;
	const auto& rest_points = mRestCurvePoints.AsConst();
	assert(global_vtx_id < rest_points.size());

	return rest_points[global_vtx_id];
}

const pxr::GfVec3f& GuideCurvesContainer::getCurveLivePoint(uint32_t curve_id, uint32_t vertex_id) const {
	assert(curve_id < mCurveVertexCounts.size() && curve_id < mCurveOffsets.size());
	assert(vertex_id < mCurveVertexCounts[curve_id]);

	const size_t global_vtx_id = mCurveOffsets[curve_id] + vertex_id;
	const auto& live_points = mRestCurvePoints.AsConst();
	assert(global_vtx_id < live_points.size());

	return live_points[global_vtx_id];
}

void GuideCurvesContainer::getVeticesAtLength(uint32_t curve_index, float len, uint32_t& v0, uint32_t& v1) const {
    const float u = len / getRestCurveLength(curve_index);
    getVeticesAtU(curve_index, u, v0, v1);
}

void GuideCurvesContainer::getVeticesAtU(uint32_t curve_index, float u, uint32_t& v0, uint32_t& v1) const {
    assert(curve_index < mCurvesCount);
    uint32_t vtx_offset = mCurveOffsets[curve_index];

    u = std::min(std::max(0.0f, u), 1.0f); // clamp to [0.0-1.0];

    if(mCurveVertexCounts[curve_index] < 2) {
        v0 = v1 = vtx_offset;
        return;
    } else if(mCurveVertexCounts[curve_index] == 2) {
        v0 = vtx_offset; v1 = vtx_offset + 1;
        return;
    }
    
    if(u < 0.0001f) {
        v0 = vtx_offset; v1 = v0+1; 
        return;
    } else if (u > 0.9999f) {
        v1 = vtx_offset + mCurveVertexCounts[curve_index] - 1u; 
        v0 = v1 - 1;
        return;
    }

    u = u * (float)(mCurveVertexCounts[curve_index] - 1u); // unnormalize
    float integer_part = std::floor(u);

    v0 = vtx_offset + std::max(0u, (uint32_t)integer_part);
    v1 = v0 + 1;
}

void GuideCurvesContainer::getVeticesWithWeightsAtLength(uint32_t curve_index, float len, uint32_t& v0, uint32_t& v1, float& w0, float& w1) const {
	const float u = len / getRestCurveLength(curve_index);
	getVeticesWithWeightsAtU(curve_index, u, v0, v1, w0, w1);
}


void GuideCurvesContainer::getVeticesWithWeightsAtU(uint32_t curve_index, float u, uint32_t& v0, uint32_t& v1, float& w0, float& w1) const {
    assert(curve_index < mCurvesCount);
    uint32_t vtx_offset = mCurveOffsets[curve_index];

    u = std::min(std::max(0.0f, u), 1.0f); // clamp to [0.0-1.0];

    if(mCurveVertexCounts[curve_index] < 2) {
        v0 = v1 = vtx_offset; w0 = 1.f; w1 = 0.f;
        return;
    } else if(mCurveVertexCounts[curve_index] == 2) {
        v0 = vtx_offset; v1 = vtx_offset + 1;
        w0 = 1.f - u; w1 = u;
        return;
    }
	
	if(u < 0.0001f) {
		v0 = vtx_offset; v1 = v0+1; 
        w0 = 1.f; w1 = 0.f;
		return;
	} else if (u > 0.9999f) {
	   v1 = vtx_offset + mCurveVertexCounts[curve_index] - 1u; 
        v0 = v1 - 1;
        w0 = 0.f; w1 = 1.f;
		return;
	}

	u = u * (float)(mCurveVertexCounts[curve_index] - 1u); // unnormalize
	float integer_part;
    float fractional_part = std::modf(u, &integer_part);

	w1 = fractional_part;
	w0 = 1.f - w1;

	v0 = vtx_offset + std::max(0u, (uint32_t)integer_part);
	v1 = v0 + 1;
}

pxr::GfVec3f GuideCurvesContainer::getPointAtU(uint32_t curve_index, float u, bool live) const {
	uint32_t v0, v1;
	float w0, w1;

	getVeticesWithWeightsAtU(curve_index, u, v0, v1, w0, w1);

    const auto& points = live ? mLiveCurvePoints.AsConst() : mRestCurvePoints.AsConst();

    return points[v0] * w0 + points[v1] * w1;
}

// KDTree

USDCurveKDTree::USDCurveKDTree(const pxr::UsdGeomCurves& usdCurves, pxr::UsdTimeCode time_code) {
    pxr::VtArray<pxr::GfVec3f> points;
    pxr::VtArray<int> curveVertexCounts;
    
    usdCurves.GetPointsAttr().Get(&points, time_code);
    usdCurves.GetCurveVertexCountsAttr().Get(&curveVertexCounts, time_code);

    std::vector<CurvePoint> flatPoints;
    flatPoints.reserve(points.size());

    size_t pointOffset = 0;
    for (size_t c = 0; c < curveVertexCounts.size(); ++c) {
        int numPoints = curveVertexCounts[c];
        if (numPoints <= 0) continue;

        for (int p = 0; p < numPoints; ++p) {
            CurvePoint cp;
            cp.position = points[pointOffset + p];
            cp.curveIndex = c;
            cp.segmentIndex = p;

            // Tag structural curve topology positions
            if (p == 0) {
                cp.positionType = CurvePositionType::Root;
            } else if (p == numPoints - 1) {
                cp.positionType = CurvePositionType::Tip;
            } else {
                cp.positionType = CurvePositionType::Internal;
            }

            flatPoints.push_back(cp);
        }
        pointOffset += numPoints;
    }

    mpRootNode = buildTreeRec(flatPoints, 0, 0, flatPoints.size());
}

USDCurveKDTree::USDCurveKDTree(const GuideCurvesContainer* pCurvesContainer) {

	std::vector<CurvePoint> flatPoints;
	flatPoints.reserve(pCurvesContainer->getTotalVertexCount());

	const auto& points = pCurvesContainer->getRestCurvePoints();
	const auto& curveVertexCounts = pCurvesContainer->getCurveVertexCounts();

	size_t pointOffset = 0;
    for (size_t c = 0; c < curveVertexCounts.size(); ++c) {
        int numPoints = curveVertexCounts[c];
        if (numPoints <= 0) continue;

        for (int p = 0; p < numPoints; ++p) {
            CurvePoint cp;
            cp.position = points[pointOffset + p];
            cp.curveIndex = c;
            cp.segmentIndex = p;

            // Tag structural curve topology positions
            if (p == 0) {
                cp.positionType = CurvePositionType::Root;
            } else if (p == numPoints - 1) {
                cp.positionType = CurvePositionType::Tip;
            } else {
                cp.positionType = CurvePositionType::Internal;
            }

            flatPoints.push_back(cp);
        }
        pointOffset += numPoints;
    }

	mpRootNode = buildTreeRec(flatPoints, 0, 0, flatPoints.size());
}

std::shared_ptr<USDCurveKDTree::KDNode> USDCurveKDTree::buildTreeRec(std::vector<USDCurveKDTree::CurvePoint>& points, int depth, int start, int end) {
    if (start >= end) return nullptr;

    int axis = depth % 3;

    std::sort(points.begin() + start, points.begin() + end, [axis](const CurvePoint& a, const CurvePoint& b) {
        return a.position[axis] < b.position[axis];
    });

    int medianIndex = start + (end - start) / 2;
    auto pNode = std::make_shared<KDNode>(points[medianIndex]);

    pNode->left = buildTreeRec(points, depth + 1, start, medianIndex);
    pNode->right = buildTreeRec(points, depth + 1, medianIndex + 1, end);

    return pNode;
}

void USDCurveKDTree::searchNearestRec(const std::shared_ptr<USDCurveKDTree::KDNode>& pNode, const pxr::GfVec3f& query, int depth,
	const std::vector<USDCurveKDTree::CurvePositionType>& allowedTypes, const std::optional<size_t>& ignore_curve_id,
	std::shared_ptr<KDNode>& bestNode, float& bestDistSq) const {

    if (!pNode) return;

    // Structural Filter Checks
    bool idMatch = ignore_curve_id.has_value() && (pNode->point.curveIndex == *ignore_curve_id);
    bool typeMatch = true;
    
    if (!allowedTypes.empty()) {
        typeMatch = std::find(allowedTypes.begin(), allowedTypes.end(), pNode->point.positionType) != allowedTypes.end();
    }

    if (!idMatch && typeMatch) {
        float distSq = (pNode->point.position - query).GetLengthSq();
        if (distSq < bestDistSq) {
            bestDistSq = distSq;
            bestNode = pNode;
        }
    }

    // Standard spatial tree traversal logic
    int axis = depth % 3;
    float diff = query[axis] - pNode->point.position[axis];

    std::shared_ptr<KDNode> nextBranch = (diff < 0) ? pNode->left : pNode->right;
    std::shared_ptr<KDNode> otherBranch = (diff < 0) ? pNode->right : pNode->left;

    searchNearestRec(nextBranch, query, depth + 1, allowedTypes, ignore_curve_id, bestNode, bestDistSq);

    if ((diff * diff) < bestDistSq) {
        searchNearestRec(otherBranch, query, depth + 1, allowedTypes, ignore_curve_id, bestNode, bestDistSq);
    }
}

void USDCurveKDTree::searchKNearestRec(const std::shared_ptr<USDCurveKDTree::KDNode>& pNode, const pxr::GfVec3f& query, int depth, size_t k,
	const std::vector<USDCurveKDTree::CurvePositionType>& allowedTypes, const std::optional<size_t>& ignore_curve_id, std::priority_queue<USDCurveKDTree::KNNCandidate>& maxHeap) const {
    
    if (!pNode) return;

    // 1. Evaluate spatial and structural constraints
    bool idMatch = ignore_curve_id.has_value() && (pNode->point.curveIndex == *ignore_curve_id);
    bool typeMatch = true;
    if (!allowedTypes.empty()) {
        typeMatch = std::find(allowedTypes.begin(), allowedTypes.end(), pNode->point.positionType) != allowedTypes.end();
    }

    if (!idMatch && typeMatch) {
        float distSq = (pNode->point.position - query).GetLengthSq();
        
        if (maxHeap.size() < k) {
            maxHeap.push({pNode->point, distSq});
        } else if (distSq < maxHeap.top().distSq) {
            maxHeap.pop(); // Evict the furthest element out of the current K matches
            maxHeap.push({pNode->point, distSq});
        }
    }

    // 2. Traversal sorting logic
    int axis = depth % 3;
    float diff = query[axis] - pNode->point.position[axis];

    std::shared_ptr<KDNode> nextBranch = (diff < 0) ? pNode->left : pNode->right;
    std::shared_ptr<KDNode> otherBranch = (diff < 0) ? pNode->right : pNode->left;

    searchKNearestRec(nextBranch, query, depth + 1, k, allowedTypes, ignore_curve_id, maxHeap);

    // 3. Dynamic Pruning: Only check the other side if it can beat our worst element in the heap
    float currentWorstDistSq = (maxHeap.size() < k) ? std::numeric_limits<float>::max() : maxHeap.top().distSq;
    if ((diff * diff) < currentWorstDistSq) {
        searchKNearestRec(otherBranch, query, depth + 1, k, allowedTypes, ignore_curve_id, maxHeap);
    }
}

std::vector<std::pair<USDCurveKDTree::CurvePoint, float>> USDCurveKDTree::finalizeKNN(std::priority_queue<USDCurveKDTree::KNNCandidate>& maxHeap) const {
    std::vector<std::pair<CurvePoint, float>> results;
    results.reserve(maxHeap.size());
    
    while (!maxHeap.empty()) {
        auto top = maxHeap.top();
        results.push_back({top.point, std::sqrt(top.distSq)});
        maxHeap.pop();
    }
    // Max-heap pops items from furthest to closest; reverse to get closest first
    std::reverse(results.begin(), results.end());
    return results;
}

bool USDCurveKDTree::findNearestPoint(const pxr::GfVec3f& query, USDCurveKDTree::CurvePoint& result, float& outDistance, std::optional<size_t> ignore_curve_id) const {

    assert(mpRootNode);

    std::shared_ptr<KDNode> bestNode = nullptr;
    float bestDistSq = std::numeric_limits<float>::max();

    searchNearestRec(mpRootNode, query, 0, {}, ignore_curve_id, bestNode, bestDistSq);

    if (bestNode) {
        result = bestNode->point;
        outDistance = std::sqrt(bestDistSq);
        return true;
    }
    return false;
}

bool USDCurveKDTree::findNearestCurveRoot(const pxr::GfVec3f& query, USDCurveKDTree::CurvePoint& result, float& outDistance, std::optional<size_t> ignore_curve_id) const {

    assert(mpRootNode);

    std::shared_ptr<KDNode> bestNode = nullptr;
    float bestDistSq = std::numeric_limits<float>::max();

    searchNearestRec(mpRootNode, query, 0, {CurvePositionType::Root}, ignore_curve_id, bestNode, bestDistSq);

    if (bestNode) {
        result = bestNode->point;
        outDistance = std::sqrt(bestDistSq);
        return true;
    }
    return false;
}

bool USDCurveKDTree::findNearestCurveTip(const pxr::GfVec3f& query, USDCurveKDTree::CurvePoint& result, float& outDistance, std::optional<size_t> ignore_curve_id) const {

    assert(mpRootNode);

    std::shared_ptr<KDNode> bestNode = nullptr;
    float bestDistSq = std::numeric_limits<float>::max();

    searchNearestRec(mpRootNode, query, 0, {CurvePositionType::Tip}, ignore_curve_id, bestNode, bestDistSq);

    if (bestNode) {
        result = bestNode->point;
        outDistance = std::sqrt(bestDistSq);
        return true;
    }
    return false;
}

std::vector<std::pair<USDCurveKDTree::CurvePoint, float>> USDCurveKDTree::findKNearestPoints(const pxr::GfVec3f& query, size_t k, std::optional<size_t> ignore_curve_id) const {
	
	assert(mpRootNode);
    if (k == 0) return {};
    std::priority_queue<KNNCandidate> maxHeap;
    searchKNearestRec(mpRootNode, query, 0, k, {}, ignore_curve_id, maxHeap);
    return finalizeKNN(maxHeap);
}

std::vector<std::pair<USDCurveKDTree::CurvePoint, float>> USDCurveKDTree::findKNearestCurveRoots(const pxr::GfVec3f& query, size_t k, std::optional<size_t> ignore_curve_id) const {
    
	assert(mpRootNode);
    if (k == 0) return {};
    std::priority_queue<KNNCandidate> maxHeap;
    searchKNearestRec(mpRootNode, query, 0, k, {CurvePositionType::Root}, ignore_curve_id, maxHeap);
    return finalizeKNN(maxHeap);
}

std::vector<std::pair<USDCurveKDTree::CurvePoint, float>> USDCurveKDTree::findKNearestCurveTips(const pxr::GfVec3f& query, size_t k, std::optional<size_t> ignore_curve_id) const {
    
	assert(mpRootNode);
    if (k == 0) return {};
    std::priority_queue<KNNCandidate> maxHeap;
    searchKNearestRec(mpRootNode, query, 0, k, {CurvePositionType::Tip}, ignore_curve_id, maxHeap);
    return finalizeKNN(maxHeap);
}

} // namespace Piston
