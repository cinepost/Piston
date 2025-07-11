#include "tests.h"


#define RUN_TEST(func, result, verbose) \
{ \
	if(verbose) std::cout << "Test " << #func << " "; \
	bool _result = func(verbose); \
	if(verbose) std::cout << (_result ? "succeeded." : "failed!") << std::endl; \
	if(!_result) result = false; \
}


namespace Piston {

namespace Tests {

static const pxr::GfVec3f sPtOrig{0.f, 0.f, -1.f};
static const pxr::GfVec3f sPtOrigMiss{-1.f, -1.f, -1.f};

static const pxr::GfVec3f sTestTriangleNorm{0.f, 0.f, 1.f};
static const std::array<pxr::GfVec3f, 3> sTestTriangle{pxr::GfVec3f{0.f, 0.f, 0.f}, pxr::GfVec3f{5.f, 0.f, 0.f}, pxr::GfVec3f{0.f, 5.f, 0.f}};

static bool fequal(float a, float b) {
	static constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
	return std::fabs(a - b) <= kEpsilon;
}

static bool testRayTriangleIntersectDist(bool verbose) {
	float u, v, dist;
	bool result = pointTriangleProject(sPtOrig, sTestTriangleNorm, sTestTriangle[0], sTestTriangle[1], sTestTriangle[2], dist, u, v);

	if(fequal(dist, 1.f)) result = false;

	return result;
}

static bool testRayTriangleIntersectDistMiss(bool verbose) {
	float u, v, dist;
	bool result = pointTriangleProject(sPtOrigMiss, sTestTriangleNorm, sTestTriangle[0], sTestTriangle[1], sTestTriangle[2], dist, u, v);

	if(!fequal(dist, 1.f)) result = false;

	return !result;
}

static bool testRayTriangleIntersectNoDist(bool verbose) {
	float u, v;
	bool result = pointTriangleProject(sPtOrig, sTestTriangleNorm, sTestTriangle[0], sTestTriangle[1], sTestTriangle[2], u, v);

	return result;
}

static bool testRayTriangleIntersectNoDistMiss(bool verbose) {
	float u, v;
	bool result = pointTriangleProject(sPtOrigMiss, sTestTriangleNorm, sTestTriangle[0], sTestTriangle[1], sTestTriangle[2], u, v);

	return !result;
}

static bool testPointPlaneDistance(bool verbose) {
	const Plane plane(pxr::GfVec3f{0.f, 0.f, 0.f}, pxr::GfVec3f{0.f, 1.f, 0.f});

	float pos_dist = distance(pxr::GfVec3f{0.f, 1.f, 0.f}, plane);
	float neg_dist = distance(pxr::GfVec3f{0.f,-1.f, 0.f}, plane);
	float zer_dist = distance(pxr::GfVec3f{0.f, 0.f, 0.f}, plane);

	if(!fequal(pos_dist, 1.f)) return false;
	if(!fequal(neg_dist,-1.f)) return false;
	if(!fequal(zer_dist, 0.f)) return false;

	return true;
}

bool runTests(bool verbose) {
	bool result = true;
	
	RUN_TEST(testRayTriangleIntersectDist, result, verbose);
	RUN_TEST(testRayTriangleIntersectDistMiss, result, verbose);
	RUN_TEST(testRayTriangleIntersectNoDist, result, verbose);
	RUN_TEST(testRayTriangleIntersectNoDistMiss, result, verbose);
	RUN_TEST(testPointPlaneDistance, result, verbose);

	if(verbose && result) {
		std::cout << "All test passed !" << std::endl;
	}

	return result;
}

} // namespace Tests

} // namespace Piston