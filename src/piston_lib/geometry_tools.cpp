#include "geometry_tools.h"
#include "adjacency.h"
#include "phantom_trimesh.h"

#include <pxr/base/gf/math.h>

#include <limits>
#include <cmath>
#include <algorithm>

#define CHECK_PARALLEL

namespace Piston {

static constexpr float kEpsilon = std::numeric_limits<float>::epsilon();

static const pxr::GfMatrix3f kRotMat180 = {
	1.f, 0.f, 0.f,
	0.f,-1.f, 0.f,
	0.f, 0.f,-1.f
};

pxr::GfMatrix3f rotateAlign(const pxr::GfVec3f& n1, const pxr::GfVec3f& n2) {
    const float cosA = pxr::GfDot(n1, n2);
    const pxr::GfVec3f axis = pxr::GfCross(n1, n2);
    const float k = (cosA == -1.f) ? (1.0f / (1.0f + cosA)) : 0.0f;

    pxr::GfMatrix3f result( 
        (axis[0] * axis[0] * k) + cosA,    (axis[1] * axis[0] * k) - axis[2], (axis[2] * axis[0] * k) + axis[1],
		(axis[0] * axis[1] * k) + axis[2], (axis[1] * axis[1] * k) + cosA,    (axis[2] * axis[1] * k) - axis[0],
		(axis[0] * axis[2] * k) - axis[1], (axis[1] * axis[2] * k) + axis[0], (axis[2] * axis[2] * k) + cosA 
	);

    return result;
}

pxr::GfMatrix3f rotateAlign2(const pxr::GfVec3f& n1, const pxr::GfVec3f& n2) {
    const pxr::GfVec3f axis = pxr::GfCross(n1, n2);
    const float c = pxr::GfDot(n1, n2);
    const float s = sqrt(1.f - c * c); 
    const float t = 1.f - c;
    
    return { 
        (axis[0] * axis[0] * t) + c,            (axis[0] * axis[1] * t) - axis[2] * s,  (axis[0] * axis[2] * t) + axis[1] * s,
        (axis[0] * axis[1] * t) + axis[2] * s,  (axis[1] * axis[1] * t) + c,            (axis[1] * axis[2] * t) - axis[0] * s,
        (axis[0] * axis[2] * t) - axis[1] * s,  (axis[1] * axis[2] * t) + axis[0] * s,  (axis[2] * axis[2] * t) + c 
    };
}

bool pointTriangleProject(const pxr::GfVec3f &pt, const pxr::GfVec3f &n, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &u, float &v) {
    pxr::GfVec3f v0v1 = v1 - v0;
    pxr::GfVec3f v0v2 = v2 - v0;
    pxr::GfVec3f pvec = pxr::GfCross(-n, v0v2);
    float det = pxr::GfDot(v0v1, pvec);

#ifdef CHECK_CULLING
    // If the determinant is negative, the triangle is back-facing.
    // If the determinant is close to 0, the ray misses the triangle.
    if (det < kEpsilon) return false;
#else
#ifdef CHECK_PARALLEL
    // If det is close to 0, the ray and triangle are parallel.
    if (std::fabs(det) < kEpsilon) return false;
#endif // CHECK_PARALLEL
#endif // CHECK_CULLING
    float invDet = 1.f / det;

    pxr::GfVec3f tvec = pt - v0;
    u = pxr::GfDot(tvec, pvec) * invDet;
    if ((u < 0.f) || (u > 1.f)) {
        return false;
    }

    pxr::GfVec3f qvec = pxr::GfCross(tvec, v0v1);
    v = pxr::GfDot(-n, qvec) * invDet;
    if ((v < 0.f) || ((u + v) > 1.f)) {
        return false;
    }

    return true;
}

bool pointTriangleProject(const pxr::GfVec3f &pt, const pxr::GfVec3f &n, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &dist, float &u, float &v) {
    pxr::GfVec3f v0v1 = v1 - v0;
    pxr::GfVec3f v0v2 = v2 - v0;
    pxr::GfVec3f pvec = pxr::GfCross(-n, v0v2);
    float det = pxr::GfDot(v0v1, pvec);

#ifdef CHECK_CULLING
    // If the determinant is negative, the triangle is back-facing.
    // If the determinant is close to 0, the ray misses the triangle.
    if (det < kEpsilon) return false;
#else
#ifdef CHECK_PARALLEL
    // If det is close to 0, the ray and triangle are parallel.
    if (std::fabs(det) < kEpsilon) return false;
#endif // CHECK_PARALLEL
#endif // CHECK_CULLING
    float invDet = 1.f / det;

    pxr::GfVec3f tvec = pt - v0;
    u = pxr::GfDot(tvec, pvec) * invDet;
    if ((u < 0.f) || (u > 1.f)) {
    	return false;
    }

    pxr::GfVec3f qvec = pxr::GfCross(tvec, v0v1);
    v = pxr::GfDot(-n, qvec) * invDet;
    if ((v < 0.f) || ((u + v) > 1.f)) {
    	return false;
    }

    dist = pxr::GfDot(v0v2, qvec) * invDet;
    return true;
}

bool rayTriangleIntersect(const pxr::GfVec3f &orig, const pxr::GfVec3f &dir, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &u, float &v) {
    pxr::GfVec3f v0v1 = v1 - v0;
    pxr::GfVec3f v0v2 = v2 - v0;
    pxr::GfVec3f pvec = pxr::GfCross(dir, v0v2);
    float det = pxr::GfDot(v0v1, pvec);

#ifdef CHECK_CULLING
    // If the determinant is negative, the triangle is back-facing.
    // If the determinant is close to 0, the ray misses the triangle.
    if (det < kEpsilon) return false;
#else
#ifdef CHECK_PARALLEL
    // If det is close to 0, the ray and triangle are parallel.
    if (std::fabs(det) < kEpsilon) return false;
#endif // CHECK_PARALLEL
#endif // CHECK_CULLING
    float invDet = 1.f / det;

    bool result = true;
    pxr::GfVec3f tvec = orig - v0;
    u = pxr::GfDot(tvec, pvec) * invDet;
    if ((u < 0.f) || (u > 1.f)) {
        result = false;
    }

    pxr::GfVec3f qvec = pxr::GfCross(tvec, v0v1);
    v = pxr::GfDot(dir, qvec) * invDet;
    if ((v < 0.f) || ((u + v) > 1.f)) {
        result = false;
    }

    return result;
}

bool rayTriangleIntersect(const pxr::GfVec3f &orig, const pxr::GfVec3f &dir, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &dist, float &u, float &v) {
    pxr::GfVec3f v0v1 = v1 - v0;
    pxr::GfVec3f v0v2 = v2 - v0;
    pxr::GfVec3f pvec = pxr::GfCross(dir, v0v2);
    float det = pxr::GfDot(v0v1, pvec);

#ifdef CHECK_CULLING
    // If the determinant is negative, the triangle is back-facing.
    // If the determinant is close to 0, the ray misses the triangle.
    if (det < kEpsilon) return false;
#else
#ifdef CHECK_PARALLEL
    // If det is close to 0, the ray and triangle are parallel.
    if (std::fabs(det) < kEpsilon) return false;
#endif // CHECK_PARALLEL
#endif // CHECK_CULLING
    float invDet = 1.f / det;

    bool result = true;
    pxr::GfVec3f tvec = orig - v0;
    u = pxr::GfDot(tvec, pvec) * invDet;
    if ((u < 0.f) || (u > 1.f)) {
        result = false;
    }

    pxr::GfVec3f qvec = pxr::GfCross(tvec, v0v1);
    v = pxr::GfDot(dir, qvec) * invDet;
    if ((v < 0.f) || ((u + v) > 1.f)) {
        result = false;
    }

    dist = pxr::GfDot(v0v2, qvec) * invDet;
    return result;
}

template <typename T>
void buildVertexNormals(const UsdGeomMeshFaceAdjacency* pAdjacency, const PhantomTrimesh* pTrimesh, std::vector<pxr::GfVec3f>& vertex_normals, const T& pt_positions, BS::thread_pool<BS::tp::none>* pThreadPool) {
    static_assert(std::is_same_v<T, std::vector<pxr::GfVec3f>> || std::is_same_v<T, pxr::VtArray<pxr::GfVec3f>>, "Only std::vector<pxr::GfVec3f> and pxr::VtArray<pxr::GfVec3f> types are permitted!");    
    assert(pAdjacency);

    vertex_normals.resize(pAdjacency->getVertexCount());
    std::vector<PhantomTrimesh::PxrIndexType> vertices;

    if(pTrimesh && pTrimesh->getVertices().size() > 0) {
        vertices = pTrimesh->getVertices();
    } else {
        const auto mesh_vertex_count = pAdjacency->getVertexCount();
        vertices.resize(mesh_vertex_count);
        for(auto i = 0; i < mesh_vertex_count; ++i) {
            vertices[i] = i;
        }
    }
    
    auto func = [&](const std::size_t vertex_index) {
        pxr::GfVec3f vn = {0.f, 0.f, 0.f};

        const auto& vtx = vertices[vertex_index];

        const uint32_t edges_count = pAdjacency->getNeighborsCount(vtx);
        const uint32_t vtx_offset = pAdjacency->getNeighborsOffset(vtx);
        
        for(uint32_t i = 0; i < edges_count; ++i) {
            const auto& vtx_pair = pAdjacency->getCornerVertexPair(vtx_offset + i);
            vn += pxr::GfGetNormalized(pxr::GfCross(pt_positions[vtx_pair.first] - pt_positions[vtx], pt_positions[vtx_pair.second] - pt_positions[vtx])
                , MIN_VECTOR_LENGTH_F
            );
        }

        vertex_normals[vtx] = pxr::GfGetNormalized(vn, MIN_VECTOR_LENGTH_F);
    };

    if(pThreadPool) {
        BS::multi_future<void> loop = pThreadPool->submit_loop(0u, vertices.size(), func);
        loop.wait();
    } else {
        for(size_t i = 0; i < vertices.size(); ++i) {
            func(i);
        }
    }
}

void buildRotationMinimizingFrames(const pxr::GfVec3f* pCurveRootPt, size_t curve_points_count, const pxr::GfVec3f& root_tangent, const pxr::GfVec3f& root_up_vector, std::vector<NTBFrame> v) {
    v.resize(curve_points_count);
    buildRotationMinimizingFrames(pCurveRootPt, curve_points_count, root_tangent, root_up_vector, v.begin(), v.end());
}

void buildRotationMinimizingFrames(const pxr::GfVec3f* pCurveRootPt, size_t curve_points_count, const pxr::GfVec3f& root_tangent, const pxr::GfVec3f& root_up_vector, std::vector<NTBFrame>::iterator it_begin, std::vector<NTBFrame>::iterator it_end) {
    assert(pCurveRootPt);
    assert(curve_points_count > 1);
    assert(std::distance(it_begin, it_end) == curve_points_count);

    static const float kZeroLength = MIN_VECTOR_LENGTH_F; // pxr epsilon is 1e-10, but for double

    // root frame
    const pxr::GfVec3f root_Tn = pxr::GfGetNormalized(root_tangent, MIN_VECTOR_LENGTH_F);
    const pxr::GfVec3f root_Bn = pxr::GfCross(root_Tn, root_up_vector);
    const pxr::GfVec3f root_Nn = pxr::GfCross(root_Bn, root_Tn);

    it_begin->set(root_Nn, root_tangent, root_Bn);

    // Generate tangents
    auto it = it_begin;
    const pxr::GfVec3f* pCurrPt = pCurveRootPt;
    for(size_t i = 1; i < curve_points_count; ++i){
        pxr::GfVec3f t = *(pCurrPt+1) - *pCurrPt;
        if(t.GetLength() < kZeroLength) {
            // for extremely short tangents we just keep using previous one
            it->t = (it - 1)->t;
        } else {
            it->t = t;
        }
        pCurrPt++;
        it++;
    }
    it->t = (it - 1)->t; // last point tangent is equal to the last segment tangent

    // Double reflection method: compute rotation minimizing frames
    for(auto it = it_begin; it != (it_end - 1); it++){
        auto it_next = it + 1;
        assert(it_next != it_end);
        const auto& ni = it->n;
        const auto& ti = it->t; 

        const auto& tj = it_next->t;

        pxr::GfVec3f v1 = ti; //point[i+1] - point[i]; 
        float c1 = pxr::GfDot(v1,v1);
        pxr::GfVec3f nLi = ni - (2.0 / c1) * pxr::GfDot(v1, ni) * v1;
        pxr::GfVec3f tLi = ti - (2.0 / c1) * pxr::GfDot(v1, ti) * v1;
        pxr::GfVec3f v2 = tj - tLi;
        float c2 = pxr::GfDot(v2,v2);

        it_next->n = pxr::GfGetNormalized(nLi - (2.0 / c2) * pxr::GfDot(v2, nLi) * v2, MIN_VECTOR_LENGTH_F);
        it_next->b = pxr::GfGetNormalized(pxr::GfCross(tj,it_next->n), MIN_VECTOR_LENGTH_F);
    }
}
/*
bool getQuadUV(const pxr::GfVec3f& pt, const pxr::GfVec3f& p0, const pxr::GfVec3f& p1, const pxr::GfVec3f& p2, const pxr::GfVec3f& p3, float& u, float& v, float epsilon) {
    // 1. Project 3D points onto a 2D plane to simplify math.
    // We choose the plane of the first triangle (p0, p1, p2).
    pxr::GfVec3f normal = pxr::GfCross(p1 - p0, p2 - p0).GetNormalized();
    
    // Create an orthonormal basis aligned with the quad surface
    pxr::GfVec3f axisX = (p1 - p0).GetNormalized();
    pxr::GfVec3f axisY = pxr::GfCross(normal, axisX).GetNormalized();

    // Lambda to project 3D vector to 2D space
    auto project2D = [&](const pxr::GfVec3f& p) {
        pxr::GfVec3f localP = p - p0;
        return pxr::GfVec2f(pxr::GfDot(localP, axisX), pxr::GfDot(localP, axisY));
    };

    pxr::GfVec2f q0 = project2D(p0); // Will be (0,0)
    pxr::GfVec2f q1 = project2D(p1);
    pxr::GfVec2f q2 = project2D(p2);
    pxr::GfVec2f q3 = project2D(p3);
    pxr::GfVec2f qPt = project2D(pt);

    // 2. Solve bilinear coordinates on the 2D plane: qPt = (1-u)(1-v)q0 + u(1-v)q1 + uv*q2 + (1-u)v*q3
    // This reduces to a quadratic equation: A*v^2 + B*v + C = 0
    pxr::GfVec2f a = q0 - q1 + q2 - q3;
    pxr::GfVec2f b = q1 - q0;
    pxr::GfVec2f c = q3 - q0;
    pxr::GfVec2f d = qPt - q0;

    float A = cross2D(a, c);
    float B = cross2D(a, d) + cross2D(b, c);
    float C = cross2D(b, d);

    u = 0.0f; v = 0.0f;

    // Handle linear edge cases (if A is zero, the quad is a perfect parallelogram)
    if (std::abs(A) < 1e-6f) {
        if (std::abs(B) > 1e-6f) {
            v = -C / B;
        }
    } else {
        // Solve using quadratic formula
        float discriminant = B * B - 4.0f * A * C;
        if (discriminant >= 0.0f) {
            float sqrtD = std::sqrt(discriminant);
            float v1 = (-B + sqrtD) / (2.0f * A);
            float v2 = (-B - sqrtD) / (2.0f * A);
            
            v = (v1 >= -0.001f && v1 <= 1.001f) ? v1 : v2;
        }
    }

    // Secure clamp to ensure precision limits don't break subsequent steps
    //v = std::max(0.0f, std::min(1.0f, v));

    // 3. Now solve for U using our calculated V
    pxr::GfVec2f denomU = b + a * v;
    if (std::abs(denomU[0]) > std::abs(denomU[1])) {
        if (std::abs(denomU[0]) > 1e-6f) u = (d[0] - c[0] * v) / denomU[0];
    } else {
        if (std::abs(denomU[1]) > 1e-6f) u = (d[1] - c[1] * v) / denomU[1];
    }

   return (u >= -epsilon && u <= 1.0f + epsilon) && (v >= -epsilon && v <= 1.0f + epsilon);
}
*/

#include <pxr/base/gf/vec3f.h>
#include <cmath>
#include <algorithm>

// Helper to calculate 2D cross product of 2D vectors
float Cross2D(const pxr::GfVec2f& a, const pxr::GfVec2f& b) {
    return a[0] * b[1] - a[1] * b[0];
}

// Projects a 3D point onto a 3D quad plane and returns parametric (u, v) coordinates.
// Returns true if the projection was successful.
bool getQuadUV(const pxr::GfVec3f& p, const pxr::GfVec3f& p0, const pxr::GfVec3f& p1, const pxr::GfVec3f& p2, const pxr::GfVec3f& p3, float& u, float& v, float epsilon) {

    // 1. Determine the best 2D projection plane to avoid degeneracy (handling 3D orientation)
    pxr::GfVec3f normal = pxr::GfCross(p1 - p0, p3 - p0).GetNormalized();
    float absX = std::abs(normal[0]);
    float absY = std::abs(normal[1]);
    float absZ = std::abs(normal[2]);

    int idx0 = 0, idx1 = 1;
    if (absX > absY && absX > absZ) {
        idx0 = 1; idx1 = 2; // Project to YZ plane
    } else if (absY > absX && absY > absZ) {
        idx0 = 0; idx1 = 2; // Project to XZ plane
    } else {
        idx0 = 0; idx1 = 1; // Project to XY plane
    }

    // 2. Convert 3D points to 2D project coordinates
    pxr::GfVec2f q(p[idx0], p[idx1]);
    pxr::GfVec2f q0(p0[idx0], p0[idx1]);
    pxr::GfVec2f q1(p1[idx0], p1[idx1]);
    pxr::GfVec2f q2(p2[idx0], p2[idx1]);
    pxr::GfVec2f q3(p3[idx0], p3[idx1]);

    // 3. Set up the quadratic equation terms: A*v^2 + B*v + C = 0
    pxr::GfVec2f e10 = q1 - q0;
    pxr::GfVec2f e30 = q3 - q0;
    pxr::GfVec2f e23 = q2 - q3;
    pxr::GfVec2f eq0 = q - q0;

    float A = cross2D(e10 - e23, e30); 
    float B = cross2D(eq0, e10 - e23) + cross2D(e10, e30);
    float C = cross2D(eq0, e10);

    u = 0.0f;
    v = 0.0f;

    // 4. Solve for v
    if (std::abs(A) < 1e-6f) {
        // Linear case (the quad is a trapezoid or parallelogram in this projection)
        if (std::abs(B) < 1e-6f) return false;
        v = -C / B;
    } else {
        // Quadratic case
        float det = B * B - 4.0f * A * C;
        if (det < 0.0f) return false; // Point mapping mathematically fails
        
        float sqrtDet = std::sqrt(det);
        float v1 = (-B + sqrtDet) / (2.0f * A);
        float v2 = (-B - sqrtDet) / (2.0f * A);

        // Pick the root closest to the [0, 1] range
        float dist1 = std::min(std::abs(v1 - 0.5f), std::abs(v1));
        float dist2 = std::min(std::abs(v2 - 0.5f), std::abs(v2));
        v = (dist1 < dist2) ? v1 : v2;
    }

    // 5. Solve for u using the determined v
    pxr::GfVec2f denomu = e10 + v * (e23 - e10);
    if (std::abs(denomu[0]) > std::abs(denomu[1])) {
        u = (eq0[0] - v * e30[0]) / denomu[0];
    } else {
        if (std::abs(denomu[1]) < 1e-6f) return false;
        u = (eq0[1] - v * e30[1]) / denomu[1];
    }

    return (u >= -epsilon && u <= 1.0f + epsilon) && (v >= -epsilon && v <= 1.0f + epsilon);
}


bool getTriUV(const pxr::GfVec3f& pt, const pxr::GfVec3f& p0, const pxr::GfVec3f& p1, const pxr::GfVec3f& p2, float& u, float& v, float epsilon) {
    v = 0.0f; u = 0.0f;

    pxr::GfVec3f e0 = p1 - p0;
    pxr::GfVec3f e1 = p2 - p0;
    pxr::GfVec3f vv = pt - p0;

    float dot00 = pxr::GfDot(e0, e0);
    float dot01 = pxr::GfDot(e0, e1);
    float dot02 = pxr::GfDot(e0, vv);
    float dot11 = pxr::GfDot(e1, e1);
    float dot12 = pxr::GfDot(e1, vv);

    float denom = (dot00 * dot11 - dot01 * dot01);

    if (std::abs(denom) < 1e-6f) {
        return false;
    }

    float invDenom = 1.0f / denom;
    u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    if (u < epsilon || v < epsilon || (u + v) > (1.0f + epsilon)) {
        return false;
    }

    return true;
};

template <typename T>
bool validatePrimIndices(const T& indices, size_t expected_attrib_count, LoggerStream* pLogger) {
    if(indices.size() == 0) return false;
    if(indices.size() != expected_attrib_count) {
        if(pLogger) {
            *pLogger << "Primitive indices count " << indices.size() << " is invalid. Expected " << expected_attrib_count << " indices !";
        }
        return false;
    }

    for(size_t i = 0; i < indices.size(); ++i) {
        if(indices[i] < 0) {
            if(pLogger) {
                *pLogger << "Primitive index " << i << " has invalid value: " << indices[i];
            }
            return false;
        }
    }

    return true;
}


template <typename T>
bool validatePrimIndices(const T& indices, size_t expected_attrib_count, int max_prim_id, LoggerStream* pLogger) {
    if(indices.size() == 0) return false;
    if(indices.size() != expected_attrib_count) {
        if(pLogger) {
            *pLogger << "Primitive indices count " << indices.size() << " is invalid. Expected " << expected_attrib_count << " indices !";
        }
        return false;
    }

    for(size_t i = 0; i < indices.size(); ++i) {
        if(indices[i] < 0 || indices[i] > max_prim_id) {
            if(pLogger) {
                *pLogger << "Primitive index " << i << " has invalid value: " << indices[i];
            }
            return false;
        }
    }

    return true;
}

template bool validatePrimIndices(const std::vector<int>& indices, size_t expected_attrib_count, LoggerStream* pLogger);
template bool validatePrimIndices(const pxr::VtArray<int>& indices, size_t expected_attrib_count, LoggerStream* pLogger);

template bool validatePrimIndices(const std::vector<int>& indices, size_t expected_attrib_count, int max_prim_id, LoggerStream* pLogger);
template bool validatePrimIndices(const pxr::VtArray<int>& indices, size_t expected_attrib_count, int max_prim_id, LoggerStream* pLogger);

template void buildVertexNormals(const UsdGeomMeshFaceAdjacency* pAdjacency, const PhantomTrimesh* pTrimesh, std::vector<pxr::GfVec3f>& vertex_normals, const std::vector<pxr::GfVec3f>& pt_positions, BS::thread_pool<BS::tp::none>* pThreadPool);
template void buildVertexNormals(const UsdGeomMeshFaceAdjacency* pAdjacency, const PhantomTrimesh* pTrimesh, std::vector<pxr::GfVec3f>& vertex_normals, const pxr::VtArray<pxr::GfVec3f>& pt_positions, BS::thread_pool<BS::tp::none>* pThreadPool);

} // namespace Piston