#ifndef PISTON_LIB_GEOMETRY_TOOLS_H_
#define PISTON_LIB_GEOMETRY_TOOLS_H_

#include "framework.h"
#include "logging.h"

#include <memory>
#include <string>

#include "BS_thread_pool.hpp"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>

#include <glm/vec3.hpp>
#include <glm/glm.hpp>  

#include <pxr/base/gf/matrix3f.h>


namespace Piston {

class UsdGeomMeshFaceAdjacency;
class PhantomTrimesh;

struct Plane {
	Plane(const glm::vec3& p, const glm::vec3& n): point{p[0], p[1], p[2]}, normal{n[0], n[1], n[2]} {};
	Plane(const pxr::GfVec3f& p, const pxr::GfVec3f& n): point{p[0], p[1], p[2]}, normal{n[0], n[1], n[2]} {};
	std::array<float, 3> point;
	std::array<float, 3> normal;
};

struct NTBFrame {
	NTBFrame(): n(1.0, 0.0, 0.0), t(0.0, 0.0, 1.0), b(0.0, 1.0, 0.0) {};
	NTBFrame(const pxr::GfVec3f& _n, const pxr::GfVec3f& _t, const pxr::GfVec3f& _b): n(_n), t(_t), b(_b) {};
	NTBFrame(const pxr::GfVec3f& _n, const pxr::GfVec3f& _t): n(_n), t(_t), b(pxr::GfCross(pxr::GfGetNormalized(_t), _n)) {};

	void set(const pxr::GfVec3f& _n, const pxr::GfVec3f& _t, const pxr::GfVec3f& _b) { n = _n; t = _t; b = _b; }

	pxr::GfVec3f operator*(const pxr::GfVec3f& v) const { return pxr::GfMatrix3f(n[0], t[0], b[0], n[1], t[1], b[1], n[2], t[2], b[2]) * v; }
	pxr::GfVec3f operator*(const std::array<float, 3>& v) const { return pxr::GfMatrix3f(n[0], t[0], b[0], n[1], t[1], b[1], n[2], t[2], b[2]) * pxr::GfVec3f(v[0], v[1], v[2]); }

	inline pxr::GfMatrix3f getMatrix3f() const { return pxr::GfMatrix3f(n[0], t[0], b[0], n[1], t[1], b[1], n[2], t[2], b[2]); }

	pxr::GfVec3f n, t, b;
};

inline void barycentrics_basic_clamp(float& u, float& v) {
    u = std::clamp(u, 0.0f, 1.0f);
    v = std::clamp(v, 0.0f, 1.0f);
}

inline void barycentrics_basic_clamp(float& u, float& v, float& w) {
    barycentrics_basic_clamp(u, v);
    w = std::clamp(w, 0.0f, 1.0f);
}

// Correct projection-based clamping for a point 'p' relative to vertices v0, v1, v2
inline void barycentrics_clamp_to_triangle(float& u, float& v, float& w) {
    if (u >= 0.f && v >= 0.f && w >= 0.f) return;
    
    if (u < v && u < w) {
        float sum = v + w;
        u = 0.0f; v = v / sum; w = w / sum;
    	return;
    } else if (v < w) {
        float sum = u + w;
        u = u / sum; v = 0.0f; w = w / sum;
    	return;
    } else {
        float sum = u + v;
        u = u / sum; v = v / sum; w = 0.0f;
    	return;
    }
}

// Distance from point's barycentric coords to centroid (1/3, 1/3, 1/3)
inline float barycentricDistanceSquaredToCenter(float u, float v, float w) {
    static constexpr float k = 1.0f/3.0f;

    float du = u - k;
    float dv = v - k;
    float dw = w - k;
    return du*du + dv*dv + dw*dw;
}

inline float distance(const glm::vec3& point, const Plane& plane) {
	return dot(glm::vec3{plane.normal[0], plane.normal[1], plane.normal[2]}, glm::vec3{point[0] - plane.point[0], point[1] - plane.point[1], point[2] - plane.point[2]});
}

inline float distance(const Plane& plane, const glm::vec3& point) { return distance(point, plane); } 

inline float distance(const pxr::GfVec3f& point, const Plane& plane) {
	return pxr::GfDot(pxr::GfVec3f{plane.normal[0], plane.normal[1], plane.normal[2]}, pxr::GfVec3f{point[0] - plane.point[0], point[1] - plane.point[1], point[2] - plane.point[2]});
}

inline float distance(const Plane& plane, const pxr::GfVec3f& point) { return distance(point, plane); } 

inline float distanceSquared(const pxr::GfVec3f &p1, const pxr::GfVec3f &p2) {
	const auto dx = p1[0] - p2[0];
	const auto dy = p1[1] - p2[1];
	const auto dz = p1[2] - p2[2];
	return (dx * dx) + (dy * dy) + (dz * dz);
}

inline float distance(const pxr::GfVec3f &p1, const pxr::GfVec3f &p2) {
	return sqrt(distanceSquared(p1, p2));
}

inline float lengthSquared(const pxr::GfVec3f &v) {
	return (v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]);
}

inline float distanceSquared(const pxr::GfVec3f& p, const pxr::GfVec3f& a, const pxr::GfVec3f& b) {
	const pxr::GfVec3f ab = b - a;
    const pxr::GfVec3f ap = p - a;

    float len_squared_ab = lengthSquared(ab);
    
    // Handle degenerate segment (A == B)
    if (len_squared_ab == 0.0f) {
        return lengthSquared(ap); // Distance to point A
    }

    float t = pxr::GfDot(ap, ab) / len_squared_ab;

    // Clamp t to [0, 1]
    t = std::max(0.0f, std::min(1.0f, t)); 

    pxr::GfVec3f closestPoint = {a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]};
    
    return lengthSquared(p - closestPoint);
}

inline float pointTriangleDistSquared(const pxr::GfVec3f& p, const pxr::GfVec3f& a, const pxr::GfVec3f& b, const pxr::GfVec3f& c) {
    const pxr::GfVec3f ab = b - a;
    const pxr::GfVec3f ac = c - a;
    const pxr::GfVec3f ap = p - a;

    // Region A
    float d1 = pxr::GfDot(ab, ap);
    float d2 = pxr::GfDot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return lengthSquared(p - a);

    // Region B
    pxr::GfVec3f bp = p - b;
    float d3 = pxr::GfDot(ab, bp);
    float d4 = pxr::GfDot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return lengthSquared(p - b);

    // Edge AB
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return lengthSquared(p - (a + ab * v));
    }

    // Region C
    pxr::GfVec3f cp = p - c;
    float d5 = pxr::GfDot(ab, cp);
    float d6 = pxr::GfDot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return lengthSquared(p - c);

    // Edge AC
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return lengthSquared(p - (a + ac * w));
    }

    // Edge BC
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return lengthSquared(p - (b + (c - b) * w));
    }

    // Face Region
    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return lengthSquared(p - (a + ab * v + ac * w));
};

glm::mat3 rotateAlign(const glm::vec3& n1, const glm::vec3& n2);

pxr::GfMatrix3f rotateAlign2(const pxr::GfVec3f& n1, const pxr::GfVec3f& n2);
pxr::GfMatrix3f rotateAlign(const pxr::GfVec3f& n1, const pxr::GfVec3f& n2);

bool pointTriangleProject(const pxr::GfVec3f &pt, const pxr::GfVec3f &n, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &u, float &v);
bool pointTriangleProject(const pxr::GfVec3f &pt, const pxr::GfVec3f &n, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &dist, float &u, float &v);

bool rayTriangleIntersect(const pxr::GfVec3f &orig, const pxr::GfVec3f &dir, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &u, float &v);
bool rayTriangleIntersect(const pxr::GfVec3f &orig, const pxr::GfVec3f &dir, const pxr::GfVec3f &v0, const pxr::GfVec3f &v1, const pxr::GfVec3f &v2, float &dist, float &u, float &v);

void buildVertexNormals(const UsdGeomMeshFaceAdjacency* pAdjacency, const PhantomTrimesh* pTrimesh, std::vector<pxr::GfVec3f>& vertex_normals, const pxr::VtArray<pxr::GfVec3f>& pt_positions, BS::thread_pool<BS::tp::none>* pThreadPool = nullptr);

void buildRotationMinimizingFrames(const pxr::GfVec3f* pCurveRootPt, size_t curve_points_count, const pxr::GfVec3f& root_tangent, const pxr::GfVec3f& root_up_vector, std::vector<NTBFrame> v);
void buildRotationMinimizingFrames(const pxr::GfVec3f* pCurveRootPt, size_t curve_points_count, const pxr::GfVec3f& root_tangent, const pxr::GfVec3f& root_up_vector, std::vector<NTBFrame>::iterator it_begin, std::vector<NTBFrame>::iterator it_end);

template <typename T>
bool validatePrimIndices(const T& indices, size_t expected_attrib_count, LoggerStream* pLogger = nullptr);

template <typename T>
bool validatePrimIndices(const T& indices, size_t expected_attrib_count, int max_prim_id, LoggerStream* pLogger = nullptr);

} // namespace Piston

#endif // PISTON_LIB_GEOMETRY_TOOLS_H_