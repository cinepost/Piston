#include "geometry_tools.h"
#include "adjacency.h"
#include "phantom_trimesh.h"

#include <pxr/base/gf/math.h>

#include <limits>

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


glm::mat3 rotateAlign(const glm::vec3& n1, const glm::vec3& n2) {
    const float cosA = dot(n1, n2);
    const glm::vec3 axis = cross(n1, n2);
    const float k = 1.0f / (1.0f + cosA);

    glm::mat3 result( (axis.x * axis.x * k) + cosA,
		(axis.y * axis.x * k) - axis.z, 
		(axis.z * axis.x * k) + axis.y,
		(axis.x * axis.y * k) + axis.z,  
		(axis.y * axis.y * k) + cosA,      
		(axis.z * axis.y * k) - axis.x,
		(axis.x * axis.z * k) - axis.y,  
		(axis.y * axis.z * k) + axis.x,  
		(axis.z * axis.z * k) + cosA 
	);

    return result;
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
    if (fabs(det) < kEpsilon) return false;
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
    if (fabs(det) < kEpsilon) return false;
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
    if (fabs(det) < kEpsilon) return false;
#endif // CHECK_PARALLEL
#endif // CHECK_CULLING
    float invDet = 1.f / det;

    pxr::GfVec3f tvec = orig - v0;
    u = pxr::GfDot(tvec, pvec) * invDet;
    if ((u < 0.f) || (u > 1.f)) {
        return false;
    }

    pxr::GfVec3f qvec = pxr::GfCross(tvec, v0v1);
    v = pxr::GfDot(dir, qvec) * invDet;
    if ((v < 0.f) || ((u + v) > 1.f)) {
        return false;
    }

    return true;
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
    if (fabs(det) < kEpsilon) return false;
#endif // CHECK_PARALLEL
#endif // CHECK_CULLING
    float invDet = 1.f / det;

    pxr::GfVec3f tvec = orig - v0;
    u = pxr::GfDot(tvec, pvec) * invDet;
    if ((u < 0.f) || (u > 1.f)) {
        return false;
    }

    pxr::GfVec3f qvec = pxr::GfCross(tvec, v0v1);
    v = pxr::GfDot(dir, qvec) * invDet;
    if ((v < 0.f) || ((u + v) > 1.f)) {
        return false;
    }

    dist = pxr::GfDot(v0v2, qvec) * invDet;
    return true;
}

void buildVertexNormals(const UsdGeomMeshFaceAdjacency* pAdjacency, const PhantomTrimesh* pTrimesh, std::vector<pxr::GfVec3f>& vertex_normals, bool build_live, BS::thread_pool<BS::tp::none>* pThreadPool) {
    assert(pAdjacency);
    assert(pTrimesh);

    vertex_normals.resize(pAdjacency->getVertexCount());

    const pxr::VtArray<pxr::GfVec3f>& pt_positions = build_live ? pTrimesh->getLivePositions() : pTrimesh->getRestPositions();

    const auto& faces = pTrimesh->getFaces();
    const std::vector<PhantomTrimesh::PxrIndexType>& vertices = pTrimesh->getVertices();
    
    auto func = [&](const std::size_t vertex_index) {
        pxr::GfVec3f vn = {0.f, 0.f, 0.f};

        const auto& vtx = vertices[vertex_index];

        const uint32_t edges_count = pAdjacency->getNeighborsCount(vtx);
        const uint32_t vtx_offset = pAdjacency->getNeighborsOffset(vtx);
        
        for(uint32_t i = 0; i < edges_count; ++i) {
            const auto& vtx_pair = pAdjacency->getCornerVertexPair(vtx_offset + i);
            vn += pxr::GfGetNormalized(pxr::GfCross(pt_positions[vtx_pair.first] - pt_positions[vtx], pt_positions[vtx_pair.second] - pt_positions[vtx]));
        }

        vertex_normals[vtx] = pxr::GfGetNormalized(vn);
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

} // namespace Piston