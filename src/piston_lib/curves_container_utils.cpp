
#include "curves_container_utils.h"
#include "guide_curves_container.h"

namespace Piston {

bool CurveEnclosureFinder::findEnclosingCurveTriplet( const USDCurveKDTree& kdTree, const pxr::VtArray<pxr::GfVec3f>& allPoints, const pxr::VtArray<int>& curveVertexCounts,
    const pxr::GfVec3f& P, std::vector<size_t>& outCurveIndices, std::optional<size_t> ignore_curve_id) {

    // 1. Gather a localized neighborhood of nearby candidate curve segments using your K-D tree.
    // We request a pool of candidates (e.g., K=12) to test combinations.
    auto neighbors = kdTree.findKNearestPoints(P, 12, ignore_curve_id);
    
    std::vector<InfiniteCurveLine> candidateLines;
    std::vector<size_t> uniqueCurves;

    // Extract the raw tangent vectors for unique nearby curves
    for (const auto& neighbor : neighbors) {
        size_t cIdx = neighbor.first.curveIndex;
        if (std::find(uniqueCurves.begin(), uniqueCurves.end(), cIdx) != uniqueCurves.end()) {
            continue; // Skip duplicates
        }
        uniqueCurves.push_back(cIdx);

        // Reconstruct the start offset of this specific curve in the flat OpenUSD array
        size_t offset = 0;
        for (size_t i = 0; i < cIdx; ++i) offset += curveVertexCounts[i];

        candidateLines.push_back(evaluateInfiniteLine(allPoints, offset, curveVertexCounts[cIdx], cIdx, P));
    }

    if (candidateLines.size() < 3) return false;

    // 2. Determine a dynamic projection plane based on the primary direction of our first candidate
    pxr::GfVec3f forward = candidateLines[0].direction;
    pxr::GfVec3f up = (std::abs(forward[2]) < 0.9f) ? pxr::GfVec3f(0, 0, 1) : pxr::GfVec3f(1, 0, 0);
    pxr::GfVec3f right = pxr::GfCross(forward, up).GetNormalized();
    up = pxr::GfCross(right, forward).GetNormalized();

    // Project 3D positions into a local 2D coordinate system relative to P
    auto project2D = [&](const pxr::GfVec3f& point3D) -> Vec2D {
        return Vec2D{ pxr::GfDot(point3D - P, right), pxr::GfDot(point3D - P, up) };
    };

    Vec2D p2D = { 0.0f, 0.0f }; // In our local space, point P is the origin

    // 3. Permute triplets of unique curves to see which combination encloses P
    for (size_t i = 0; i < candidateLines.size(); ++i) {
        for (size_t j = i + 1; j < candidateLines.size(); ++j) {
            for (size_t k = j + 1; k < candidateLines.size(); ++k) {
                
                std::vector<Vec2D> lineOrigins = { project2D(candidateLines[i].origin), project2D(candidateLines[j].origin), project2D(candidateLines[k].origin) };
                std::vector<Vec2D> lineDirs = { 
                    Vec2D{ pxr::GfDot(candidateLines[i].direction, right), pxr::GfDot(candidateLines[i].direction, up) },
                    Vec2D{ pxr::GfDot(candidateLines[j].direction, right), pxr::GfDot(candidateLines[j].direction, up) },
                    Vec2D{ pxr::GfDot(candidateLines[k].direction, right), pxr::GfDot(candidateLines[k].direction, up) }
                };

                if (isPointInsideTriangleLines(p2D, lineOrigins, lineDirs)) {
                    outCurveIndices = { candidateLines[i].curveIndex, candidateLines[j].curveIndex, candidateLines[k].curveIndex };
                    return true; // Match found!
                }
            }
        }
    }

    return false; // Point P is outside the convex spatial volumes of these nearby curves
}


bool CurveEnclosureFinder::findEnclosingTripletWithTarget(const USDCurveKDTree& kdTree, const pxr::VtArray<pxr::GfVec3f>& allPoints, const pxr::VtArray<int>& curveVertexCounts,
    const pxr::GfVec3f& P, size_t targetCurveIndex, std::vector<size_t>& outCurveIndices, std::optional<size_t> ignore_curve_id) {
        
    // 1. Safety Check: Ensure target index is valid and not explicitly ignored
    if (targetCurveIndex >= curveVertexCounts.size()) return false;
    if (ignore_curve_id.has_value() && targetCurveIndex == *ignore_curve_id) return false;

    std::vector<InfiniteCurveLine> candidateLines;
    std::vector<size_t> uniqueCurves;

    // 2. Lock the required target curve as our absolute first candidate slot
    uniqueCurves.push_back(targetCurveIndex);
    size_t targetOffset = calculateCurveOffset(curveVertexCounts, targetCurveIndex);
    candidateLines.push_back(evaluateInfiniteLine(allPoints, targetOffset, curveVertexCounts[targetCurveIndex], targetCurveIndex, P));

    // 3. Gather localized neighboring curves using K-D Tree
    auto neighbors = kdTree.findKNearestPoints(P, 15, ignore_curve_id);
    
    for (const auto& neighbor : neighbors) {
        size_t cIdx = neighbor.first.curveIndex;
        
        // Skip the target curve (already added) and duplicates
        if (std::find(uniqueCurves.begin(), uniqueCurves.end(), cIdx) != uniqueCurves.end()) {
            continue; 
        }
        uniqueCurves.push_back(cIdx);

        size_t offset = calculateCurveOffset(curveVertexCounts, cIdx);
        candidateLines.push_back(evaluateInfiniteLine(allPoints, offset, curveVertexCounts[cIdx], cIdx, P));
    }

    if (candidateLines.size() < 3) return false;

    // 4. Calculate an orthonormal projection plane driven by our locked target curve's tangent alignment
    pxr::GfVec3f forward = candidateLines[0].direction; 
    pxr::GfVec3f up = (std::abs(forward[2]) < 0.9f) ? pxr::GfVec3f(0, 0, 1) : pxr::GfVec3f(1, 0, 0);
    pxr::GfVec3f right = pxr::GfCross(forward, up).GetNormalized();
    up = pxr::GfCross(right, forward).GetNormalized();

    auto project2D = [&](const pxr::GfVec3f& point3D) -> Vec2D {
        return Vec2D{ pxr::GfDot(point3D - P, right), pxr::GfDot(point3D - P, up) };
    };

    Vec2D p2D = { 0.0f, 0.0f }; // Point P acts as origin on the projected cutting plane

    // 5. Permute pairs of secondary neighbor lines to close the triangle with the target line at index 0
    for (size_t j = 1; j < candidateLines.size(); ++j) {
        for (size_t k = j + 1; k < candidateLines.size(); ++k) {
            
            std::vector<Vec2D> lineOrigins = { 
                project2D(candidateLines[0].origin), 
                project2D(candidateLines[j].origin), 
                project2D(candidateLines[k].origin) 
            };
            
            std::vector<Vec2D> lineDirs = { 
                Vec2D{ pxr::GfDot(candidateLines[0].direction, right), pxr::GfDot(candidateLines[0].direction, up) },
                Vec2D{ pxr::GfDot(candidateLines[j].direction, right), pxr::GfDot(candidateLines[j].direction, up) },
                Vec2D{ pxr::GfDot(candidateLines[k].direction, right), pxr::GfDot(candidateLines[k].direction, up) }
            };

            if (isPointInsideTriangleLines(p2D, lineOrigins, lineDirs)) {
                outCurveIndices = { candidateLines[0].curveIndex, candidateLines[j].curveIndex, candidateLines[k].curveIndex };
                return true; // Enclosure containing target verified successfully
            }
        }
    }

    return false; // No valid pairing enclosing P could be completed using this specific target curve
}

CurveBarycentricCoords AdvancedCurveBarycentricSolver::computeWeightsAndUFromTriplet(const pxr::GfVec3f& P, const std::array<size_t, 3>& curveTriplet, const pxr::VtArray<pxr::GfVec3f>& allPoints, const pxr::VtArray<int>& curveVertexCounts)  {
    CurveBarycentricCoords output;
    output.curveIndices = curveTriplet;

    // 1. Safety Check: Verify indices exist within bounds
    for (size_t cIdx : curveTriplet) {
        if (cIdx >= curveVertexCounts.size()) return output;
    }

    // 2. Dynamically extract geometry data and construct the 3D infinite lines
    std::array<InfiniteCurveLine, 3> chosenCurves;
    for (int i = 0; i < 3; ++i) {
        size_t cIdx = curveTriplet[i];
        size_t offset = calculateCurveOffset(curveVertexCounts, cIdx);
        chosenCurves[i] = evaluateInfiniteLine(allPoints, offset, curveVertexCounts[cIdx], cIdx, P);
    }

    // 3. Dynamically generate the projection plane based on the first curve's trajectory vector
    pxr::GfVec3f forward = chosenCurves[0].direction;
    pxr::GfVec3f upBasis = (std::abs(forward[2]) < 0.9f) ? pxr::GfVec3f(0, 0, 1) : pxr::GfVec3f(1, 0, 0);
    pxr::GfVec3f rightBasis = pxr::GfCross(forward, upBasis).GetNormalized();
    upBasis = pxr::GfCross(rightBasis, forward).GetNormalized();

    // 4. Project 3D positions onto the shared 2D cutting plane relative to P
    auto project2D = [&](const pxr::GfVec3f& point3D) -> Vec2D {
        return Vec2D{ pxr::GfDot(point3D - P, rightBasis), pxr::GfDot(point3D - P, upBasis) };
    };

    Vec2D p2D = { 0.0f, 0.0f }; 
    std::array<Vec2D, 3> origins;
    std::array<Vec2D, 3> dirs;

    for (int i = 0; i < 3; ++i) {
        origins[i] = project2D(chosenCurves[i].origin);
        dirs[i] = Vec2D{ pxr::GfDot(chosenCurves[i].direction, rightBasis), pxr::GfDot(chosenCurves[i].direction, upBasis) };
    }

    // 5. Calculate Cross-sectional Radial Blending Weights
    float h0 = computePerpendicularDistance(p2D, origins[0], dirs[0]);
    float h1 = computePerpendicularDistance(p2D, origins[1], dirs[1]);
    float h2 = computePerpendicularDistance(p2D, origins[2], dirs[2]);

    float rawW0 = h1 * h2;
    float rawW1 = h0 * h2;
    float rawW2 = h0 * h1;
    float totalSum = rawW0 + rawW1 + rawW2;

    if (totalSum > 1e-6f) {
        output.weights[0] = rawW0 / totalSum;
        output.weights[1] = rawW1 / totalSum;
        output.weights[2] = rawW2 / totalSum;
    } else {
        output.weights = {0.3333f, 0.3333f, 0.3333f};
    }

    // 6. Compute the Longitudinal U Coordinate Mapping
    for (int i = 0; i < 3; ++i) {
        size_t cIdx = curveTriplet[i];
        size_t offset = calculateCurveOffset(curveVertexCounts, cIdx);
        int numPoints = curveVertexCounts[cIdx];

        pxr::GfVec3f usdRoot = allPoints[offset];
        pxr::GfVec3f usdTip = allPoints[offset + numPoints - 1];
        
        float totalCurveLength = (usdTip - usdRoot).GetLength();
        if (totalCurveLength < 1e-6f) totalCurveLength = 1.0f;

        // Project P onto the infinite line trajectory
        pxr::GfVec3f originToP = P - chosenCurves[i].origin;
        float t = pxr::GfDot(originToP, chosenCurves[i].direction);
        pxr::GfVec3f projectionPoint3D = chosenCurves[i].origin + (t * chosenCurves[i].direction);

        // Calculate exact positions relative to the native curve root/tip anchors
        float distanceFromRoot = (projectionPoint3D - usdRoot).GetLength();
        float distanceFromTip = (projectionPoint3D - usdTip).GetLength();

        // Handle extrapolations outside boundaries smoothly
        if (pxr::GfDot(projectionPoint3D - usdRoot, usdTip - usdRoot) < 0.0f) {
            output.uCoords[i] = -distanceFromRoot / totalCurveLength;
        } else if (pxr::GfDot(projectionPoint3D - usdTip, usdTip - usdRoot) > 0.0f) {
            output.uCoords[i] = 1.0f + (distanceFromTip / totalCurveLength);
        } else {
            output.uCoords[i] = distanceFromRoot / totalCurveLength;
        }
    }

    output.isValid = true;
    return output;
}

}  // namespace Piston
