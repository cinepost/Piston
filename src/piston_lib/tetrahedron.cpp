#include "tetrahedron.h"
#include "phantom_trimesh.h"

namespace Piston {

template<typename VtxIndexType>
TetrahedronKDTree<VtxIndexType>::TetrahedronKDTree(const std::vector<Tetrahedron<VtxIndexType>>& tets, const pxr::VtArray<pxr::GfVec3f>& points): mTetrahedrons(tets), mPoints(points) {
    std::vector<TetraIndexType> node_tet_inidices;
    node_tet_inidices.resize(mTetrahedrons.size());
    mTetrahedronAABBs.resize(mTetrahedrons.size());
    mTetrahedronCentroids.resize(mTetrahedrons.size());

    for (auto tet_index = 0; tet_index < mTetrahedrons.size(); ++tet_index) {
        const auto& tet = mTetrahedrons[tet_index];
        mTetrahedronAABBs[tet_index].fit(mPoints[tet.indices[0]]);
        mTetrahedronAABBs[tet_index].fit(mPoints[tet.indices[1]]);
        mTetrahedronAABBs[tet_index].fit(mPoints[tet.indices[2]]);
        mTetrahedronAABBs[tet_index].fit(mPoints[tet.indices[3]]);

        mTetrahedronCentroids[tet_index] =  (mPoints[tet.indices[0]] + mPoints[tet.indices[1]] + mPoints[tet.indices[2]] + mPoints[tet.indices[3]]) * 0.25;
        node_tet_inidices[tet_index] = static_cast<TetraIndexType>(tet_index);
    }
    mpRoot = build(std::move(node_tet_inidices), 0);
}

template<typename VtxIndexType>
typename TetrahedronKDTree<VtxIndexType>::KDNode* TetrahedronKDTree<VtxIndexType>::build(std::vector<TetraIndexType>&& tet_indices, int depth) {
    if (tet_indices.empty()) return nullptr;

    KDNode* node = new KDNode();
    for (auto tet_index : tet_indices) {
        node->nodeBounds.fit(mTetrahedronAABBs[tet_index].min);
        node->nodeBounds.fit(mTetrahedronAABBs[tet_index].max);
    }

    if (tet_indices.size() <= MAX_LEAF_SIZE) {
        node->tet_indices = std::move(tet_indices);
        return node;
    }

    int axis = depth % 3;
    std::sort(tet_indices.begin(), tet_indices.end(), [&, axis](const TetraIndexType i_a, const TetraIndexType i_b) {
        const pxr::GfVec3f& ca = mTetrahedronCentroids[i_a];
        const pxr::GfVec3f& cb = mTetrahedronCentroids[i_b];
        return (axis == 0) ? (ca[0] < cb[0]) : ((axis == 1) ? (ca[1] < cb[1]) : (ca[2] < cb[2]));
    });

    auto medianIdx = tet_indices[tet_indices.size() / 2];
    const auto& centroid = mTetrahedronCentroids[medianIdx];
    double splitVal = (axis == 0) ? centroid[0] : ((axis == 1) ? centroid[1] : centroid[2]);

    std::vector<TetraIndexType> leftTets, rightTets;
    for (auto tet_index : tet_indices) {
        const auto& aabb = mTetrahedronAABBs[tet_index];
        double tMin = (axis == 0) ? aabb.min[0] : ((axis == 1) ? aabb.min[1] : aabb.min[2]);
        double tMax = (axis == 0) ? aabb.max[0] : ((axis == 1) ? aabb.max[1] : aabb.max[2]);
        
        if (tMin <= splitVal) leftTets.push_back(tet_index);
        if (tMax >= splitVal) rightTets.push_back(tet_index);
    }

    // Avoid infinite recursion if items cannot be split cleanly
    if (leftTets.size() == tet_indices.size() || rightTets.size() == tet_indices.size()) {
        node->tet_indices = tet_indices;
        return node;
    }

    node->left = build(std::move(leftTets), depth + 1);
    node->right = build(std::move(rightTets), depth + 1);
    return node;
}

template<typename VtxIndexType>
void TetrahedronKDTree<VtxIndexType>::searchClosest(const KDNode* node, const pxr::GfVec3f& p, TetraIndexType& bestTetIndex, float& bestSqDist) const {
    if (!node) return;

    // Prune path if it cannot beat current best distance
    if (node->nodeBounds.squareDist(p) >= bestSqDist) return;

    if (node->isLeaf()) {
        for (auto tet_index : node->tet_indices) {
            float d = minSqDistToTetrahedron<VtxIndexType>(p, mTetrahedrons[tet_index], mPoints);
            if (d < bestSqDist) {
                bestSqDist = d;
                bestTetIndex = tet_index;
            }
        }
        return;
    }

    float dLeft = node->left ? node->left->nodeBounds.squareDist(p) : std::numeric_limits<float>::max();
    float dRight = node->right ? node->right->nodeBounds.squareDist(p) : std::numeric_limits<float>::max();

    if (dLeft < dRight) {
        searchClosest(node->left, p, bestTetIndex, bestSqDist);
        searchClosest(node->right, p, bestTetIndex, bestSqDist);
    } else {
        searchClosest(node->right, p, bestTetIndex, bestSqDist);
        searchClosest(node->left, p, bestTetIndex, bestSqDist);
    }
}

template<typename VtxIndexType>
typename TetrahedronKDTree<VtxIndexType>::TetraIndexType TetrahedronKDTree<VtxIndexType>::searchInside(const KDNode* node, const pxr::GfVec3f& p) const {
    if (!node || node->nodeBounds.squareDist(p) > 1e-9) return kInvalidTetraID;

    if (node->isLeaf()) {
        for (auto tet_index : node->tet_indices) {
            
            if (isPointInTetrahedron<VtxIndexType>(p, mTetrahedrons[tet_index], mPoints)) {
                return tet_index;
            }
        }
        return kInvalidTetraID;
    }

    // Test closest bounding space child node first
    float dLeft = node->left ? node->left->nodeBounds.squareDist(p) : std::numeric_limits<float>::max();
    float dRight = node->right ? node->right->nodeBounds.squareDist(p) : std::numeric_limits<float>::max();

    if (dLeft < dRight) {
        TetraIndexType found = searchInside(node->left, p);
        if (found != kInvalidTetraID) return found;
        return searchInside(node->right, p);
    } else {
        TetraIndexType found = searchInside(node->right, p);
        if (found != kInvalidTetraID) return found;
        return searchInside(node->left, p);
    }
}

template<typename VtxIndexType>
typename TetrahedronKDTree<VtxIndexType>::TetraIndexType TetrahedronKDTree<VtxIndexType>::queryPoint(const pxr::GfVec3f& p, bool& is_inside) const {
    is_inside = false;
    // Pass 1: Try to look for strict volumetric containment
    TetraIndexType containingTetIndex = searchInside(mpRoot, p);
    if (containingTetIndex != kInvalidTetraID) {
        is_inside = true;
        return containingTetIndex;
    }
    // Pass 2: Fallback to finding the element with closest geometric feature
    TetraIndexType closestTetIndex = kInvalidTetraID;
    float bestSqDist = std::numeric_limits<float>::max();
    searchClosest(mpRoot, p, closestTetIndex, bestSqDist);
    return closestTetIndex;
}

template<typename VtxIndexType>
typename TetrahedronKDTree<VtxIndexType>::TetraIndexType TetrahedronKDTree<VtxIndexType>::queryPointWithCache(const pxr::GfVec3f& p, TetraIndexType& cachedTetraIndex, bool& is_inside) const {
    is_inside = false;
    // Step 1: O(1) Check - Is the point still inside the previous tetrahedron?
    if ((cachedTetraIndex != kInvalidTetraID) && isPointInTetrahedron(p, mTetrahedrons[cachedTetraIndex], mPoints)) {
        is_inside = true;
        return cachedTetraIndex; // Massively faster for coherent points
    }

    // Step 2: O(1) Check - Is it inside the faces/neighbors of the cache?
    // (Optional: If you store neighbor pointers in your Tetrahedron struct, 
    // you can check the 4 neighboring tets here before traversing the tree).

    // Step 3: Fallback - Search the tree for strict containment
    TetraIndexType containingTetIndex = searchInside(mpRoot, p);
    if (containingTetIndex != kInvalidTetraID) {
        is_inside = true;
        cachedTetraIndex = containingTetIndex; // Update the cache for the next frame/step
        return containingTetIndex;
    }

    // Step 4: Fallback - Find the closest face if completely outside the mesh
    TetraIndexType closestTetIndex = kInvalidTetraID;
    float bestSqDist = std::numeric_limits<float>::max();
    searchClosest(mpRoot, p, closestTetIndex, bestSqDist);
    
    cachedTetraIndex = closestTetIndex; // Update the cache with the closest element
    return closestTetIndex;
}

} // namespace Piston

template class Piston::Tetrahedron<Piston::PhantomTrimesh::PxrIndexType>;
template class Piston::TetrahedronKDTree<Piston::PhantomTrimesh::PxrIndexType>;