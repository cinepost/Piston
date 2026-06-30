#ifndef PISTON_LIB_CURVES_CONTAINER_UTILS_H_
#define PISTON_LIB_CURVES_CONTAINER_UTILS_H_

#include <iostream>
#include <vector>
#include <optional>
#include <cmath>
#include <numeric>

// Core OpenUSD Includes
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/matrix4f.h>
#include <pxr/base/vt/array.h>


namespace Piston {

// Represents an infinite 3D line calculated from curve tangents
struct InfiniteCurveLine {
    size_t curveIndex;
    pxr::GfVec3f origin;
    pxr::GfVec3f direction;
};

namespace {

inline InfiniteCurveLine evaluateInfiniteLine(const pxr::VtArray<pxr::GfVec3f>& points, size_t curveStartOffset, size_t numPoints, size_t curveIndex, const pxr::GfVec3f& P) {

    InfiniteCurveLine line;
    line.curveIndex = curveIndex;

    if (numPoints < 2) {
        line.origin = points[curveStartOffset];
        line.direction = pxr::GfVec3f(0, 1, 0);
        return line;
    }

    pxr::GfVec3f root = points[curveStartOffset];
    pxr::GfVec3f tip = points[curveStartOffset + numPoints - 1];

    float distToRootSq = (root - P).GetLengthSq();
    float distToTipSq = (tip - P).GetLengthSq();

    if (distToRootSq <= distToTipSq) {
        line.origin = root;
        line.direction = (root - points[curveStartOffset + 1]).GetNormalized();
    } else {
        line.origin = tip;
        line.direction = (tip - points[curveStartOffset + numPoints - 2]).GetNormalized();
    }

    return line;
}

// Helper to calculate the flat memory offset of a given curve index
inline size_t calculateCurveOffset(const pxr::VtArray<int>& curveVertexCounts, size_t targetIndex) {
    size_t offset = 0;
    for (size_t i = 0; i < targetIndex; ++i) {
        offset += curveVertexCounts[i];
    }
    return offset;
}

}

class USDCurveKDTree;

// Simple helper structure for 2D calculations
struct Vec2D {
    float x, y;
};

class CurveEnclosureFinder {
    public:
        // Main solver: Finds a triplet of curves that enclose point P inside their volume
        static bool findEnclosingCurveTriplet(
            const USDCurveKDTree& kdTree,
            const pxr::VtArray<pxr::GfVec3f>& allPoints,
            const pxr::VtArray<int>& curveVertexCounts,
            const pxr::GfVec3f& P,
            std::vector<size_t>& outCurveIndices,
            std::optional<size_t> ignore_curve_id = std::nullopt);

        // Main Solver: Enforces targetCurveIndex to be one of the three enclosing items
        static bool findEnclosingTripletWithTarget(
            const USDCurveKDTree& kdTree,
            const pxr::VtArray<pxr::GfVec3f>& allPoints,
            const pxr::VtArray<int>& curveVertexCounts,
            const pxr::GfVec3f& P,
            size_t targetCurveIndex,
            std::vector<size_t>& outCurveIndices,
            std::optional<size_t> ignore_curve_id = std::nullopt);

    private:
        // Computes a 2D cross-product behavior to determine which side of an infinite line a point lies on
        static inline float getSign(const Vec2D& p, const Vec2D& lineOrigin, const Vec2D& lineDir) {
            // Normal vector to the line: (-dir.y, dir.x)
            float nx = -lineDir.y;
            float ny = lineDir.x;
            // Dot product of normal with vector from line origin to point P
            return (p.x - lineOrigin.x) * nx + (p.y - lineOrigin.y) * ny;
        }

        // Checks if point P is strictly inside the region bounded by 3 intersecting infinite lines
        static inline bool isPointInsideTriangleLines(const Vec2D& p, const std::vector<Vec2D>& origins, const std::vector<Vec2D>& dirs) {
            // Calculate which side of each line the point sits on
            float d1 = getSign(p, origins[0], dirs[0]);
            float d2 = getSign(p, origins[1], dirs[1]);
            float d3 = getSign(p, origins[2], dirs[2]);

            // To form a valid closed enclosing volume around P, the point must sit on the 
            // same semantic side (all positive or all negative) of all three oriented infinite lines.
            bool allPositive = (d1 > 1e-5f) && (d2 > 1e-5f) && (d3 > 1e-5f);
            bool allNegative = (d1 < -1e-5f) && (d2 < -1e-5f) && (d3 < -1e-5f);

            return allPositive || allNegative;
        }
};


struct CurveBarycentricCoords {
    std::array<size_t, 3> curveIndices = {0, 0, 0};
    std::array<float, 3> weights = {0.0f, 0.0f, 0.0f};
    std::array<float, 3> uCoords = {0.0f, 0.0f, 0.0f}; // U < 0 if below root, U > 1 if above tip
    bool isValid = false;
};

class AdvancedCurveBarycentricSolver {
    public:
        // Computes dual-line weights and infinite-extension U parameters purely from curve indices
        static CurveBarycentricCoords computeWeightsAndUFromTriplet( const pxr::GfVec3f& P, const std::array<size_t, 3>& curveTriplet, const pxr::VtArray<pxr::GfVec3f>& allPoints, const pxr::VtArray<int>& curveVertexCounts);

    private:
        static inline float computePerpendicularDistance(const Vec2D& p, const Vec2D& lineOrigin, const Vec2D& lineDir) {
            float length = std::sqrt(lineDir.x * lineDir.x + lineDir.y * lineDir.y);
            if (length < 1e-6f) return 0.0f;
            
            float dx = lineDir.x / length;
            float dy = lineDir.y / length;

            float nx = -dy;
            float ny = dx;

            return std::abs((p.x - lineOrigin.x) * nx + (p.y - lineOrigin.y) * ny);
        }

};


}  // namespace Piston

#endif  // PISTON_LIB_CURVES_CONTAINER_UTILS_H_
