#include "mesh_container.h"
#include "geometry_tools.h"
#include "simple_profiler.h"
#include "piston_math.h"
#include "logging.h"

#include <limits>

namespace Piston {

MeshContainer::UniquePtr MeshContainer::create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	auto pMeshContainer = std::make_unique<MeshContainer>();
	if(!pMeshContainer->init(prim_handle, rest_p_name, time_code)) {
		return nullptr;
	}
	return pMeshContainer;
}

MeshContainer::MeshContainer(): mLastUpdateTimeCode(std::numeric_limits<double>::lowest())  {

}

bool MeshContainer::init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(rest_p_name));
		
	if(!restPositionPrimVar) {
		LOG_WRN << "No valid primvar \"" << rest_p_name << "\" exists in prim " << prim_handle.getPath() << "! Using positions at time code 0.0 !";

		pxr::UsdGeomPointBased mesh(prim_handle.getPrim());

		static const pxr::UsdTimeCode s_zero_time_code(0.0);

		if(!mesh.GetPointsAttr().Get(&mUsdMeshRestPositions, s_zero_time_code)) {
			LOG_ERR << "Error getting " << prim_handle.getPath() << " point positions at time code " << s_zero_time_code.GetValue();
			return false;
		}
	} else {
		const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
		if(!restPosAttr.Get(&mUsdMeshRestPositions, time_code)) {
			LOG_ERR << "Error getting prim " << prim_handle.getPath() << " \"rest\" positions !";
			return false;
		}
		LOG_DBG << "Prim " << prim_handle.getPath() << " has " << mUsdMeshRestPositions.size() << " rest positions.";
	}

	return true;
}

pxr::GfVec3f MeshContainer::getTetrahedronRestCentroid(const PhantomTrimesh::Tetrahedron& t) const {
	assert(t.isValid());

	const pxr::VtArray<pxr::GfVec3f>& rest_positions = mUsdMeshRestPositions.AsConst();
	return (rest_positions[t.indices[0]] + rest_positions[t.indices[1]] + rest_positions[t.indices[2]] + rest_positions[t.indices[3]]) / 4.0f;
}

void MeshContainer::barycentricTetrahedronRestCoords(const PhantomTrimesh::Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w, float& x) const {
	assert(t.isValid());

	const pxr::VtArray<pxr::GfVec3f>& rest_positions = mUsdMeshRestPositions.AsConst();

	const pxr::GfVec3f& a = rest_positions[t.indices[0]];
	const pxr::GfVec3f& b = rest_positions[t.indices[1]];
	const pxr::GfVec3f& c = rest_positions[t.indices[2]];
	const pxr::GfVec3f& d = rest_positions[t.indices[3]];

    pxr::GfVec3f vap = p - a;
    pxr::GfVec3f vbp = p - b;

    pxr::GfVec3f vab = b - a;
    pxr::GfVec3f vac = c - a;
    pxr::GfVec3f vad = d - a;

    pxr::GfVec3f vbc = c - b;
    pxr::GfVec3f vbd = d - b;

    // ScTP computes the scalar triple product
    auto ScTP = [](const pxr::GfVec3f &a, const pxr::GfVec3f &b, const pxr::GfVec3f &c) {
    	// computes scalar triple product
    	return pxr::GfDot(a, pxr::GfCross(b, c));
	};

    float va6 = ScTP(vbp, vbd, vbc);
    float vb6 = ScTP(vap, vac, vad);
    float vc6 = ScTP(vap, vad, vab);
    float vd6 = ScTP(vap, vab, vac);
    float v6 = 1.f / ScTP(vab, vac, vad);

    u = va6*v6;
    v = vb6*v6;
    w = vc6*v6;
    x = vd6*v6;
}

void MeshContainer::barycentricTetrahedronRestCoords(const PhantomTrimesh::Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w) const {
	float _temp_x;
	barycentricTetrahedronRestCoords(t, p, u, v, w, _temp_x);
}

pxr::GfVec3f MeshContainer::getPointPositionFromBarycentricTetrahedronLiveCoords(const PhantomTrimesh::Tetrahedron& t, float u, float v, float w, float x) const {
	assert(t.isValid());

	const pxr::VtArray<pxr::GfVec3f>& live_positions = mUsdMeshLivePositions.AsConst();

	assert(t.indices[0] < live_positions.size());
	assert(t.indices[1] < live_positions.size());
	assert(t.indices[2] < live_positions.size());
	assert(t.indices[3] < live_positions.size());

	const pxr::GfVec3f& a = live_positions[t.indices[0]];
	const pxr::GfVec3f& b = live_positions[t.indices[1]];
	const pxr::GfVec3f& c = live_positions[t.indices[2]];
	const pxr::GfVec3f& d = live_positions[t.indices[3]];

    return {u * a[0] + v * b[0] + w * c[0] + x * d[0], 
    		u * a[1] + v * b[1] + w * c[1] + x * d[1], 
    		u * a[2] + v * b[2] + w * c[2] + x * d[2]};
}

bool MeshContainer::projectPoint(const pxr::GfVec3f& pt, const PhantomTrimesh::TriFace& face, float& u, float& v) const {
	auto const& rest_positions = mUsdMeshRestPositions.AsConst();
	return rayTriangleIntersect(pt, -getFaceRestNormal(face), rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]], u, v);
}

bool MeshContainer::projectPoint(const pxr::GfVec3f& pt, const PhantomTrimesh::TriFace& face, float& u, float& v, float& dist) const {
	auto const& rest_positions = mUsdMeshRestPositions.AsConst();
	return rayTriangleIntersect(pt, -getFaceRestNormal(face), rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]], dist, u, v);
}

bool MeshContainer::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const PhantomTrimesh::TriFace& face, float& u, float& v) const {
	auto const& rest_positions = mUsdMeshRestPositions.AsConst();
	return rayTriangleIntersect(orig, dir, rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]], u, v);
}

bool MeshContainer::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const PhantomTrimesh::TriFace& face, float& u, float& v, float& dist) const {
	auto const& rest_positions = mUsdMeshRestPositions.AsConst();
	return rayTriangleIntersect(orig, dir, rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]], dist, u, v);
}

pxr::GfVec3f MeshContainer::getInterpolatedRestPosition(const PhantomTrimesh::TriFace& face, const float u, const float v) const {
	return getInterpolatedRestPosition(face, u, v, (1.f - u - v));
};


pxr::GfVec3f MeshContainer::getInterpolatedLivePosition(const PhantomTrimesh::TriFace& face, const float u, const float v) const {
	return getInterpolatedLivePosition(face, u, v, (1.f - u - v));
};

pxr::GfVec3f MeshContainer::getInterpolatedRestPosition(const PhantomTrimesh::TriFace& face, const float u, const float v, const float w) const {
	auto const& rest_positions = mUsdMeshRestPositions.AsConst();

	return u * rest_positions[face.indices[1]] + v * rest_positions[face.indices[2]] + w * rest_positions[face.indices[0]];
}

pxr::GfVec3f MeshContainer::getInterpolatedLivePosition(const PhantomTrimesh::TriFace& face, const float u, const float v, const float w) const {
	auto const& live_positions = mUsdMeshLivePositions.AsConst();

	return u * live_positions[face.indices[1]] + v * live_positions[face.indices[2]] + w * live_positions[face.indices[0]];
}

pxr::GfVec3f MeshContainer::getFaceRestCentroid(const PhantomTrimesh::TriFace& face) const {
	static constexpr float kInvThree = 1.f / 3.f;
	auto const& rest_positions = mUsdMeshRestPositions.AsConst();

	return (rest_positions[face.indices[0]] + rest_positions[face.indices[1]] + rest_positions[face.indices[2]]) * kInvThree;
}

bool MeshContainer::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) const {
	if(mLastUpdateTimeCode == time_code && (mUsdMeshLivePositions.size() == mUsdMeshRestPositions.size())) return true;

	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());

	pxr::UsdGeomPointBased mesh(prim_handle.getPrim());

	if(!mesh.GetPointsAttr().Get(&mUsdMeshLivePositions, time_code)) {
		LOG_ERR << "Error getting point positions from " << prim_handle.getPath() << " !";
		return false;
	}

	if(mUsdMeshLivePositions.size() != mUsdMeshRestPositions.size()) {
		LOG_ERR << prim_handle.getPath() << " \"rest\" and \"live\" mesh point positions count (" << mUsdMeshRestPositions.size() << " vs " << mUsdMeshLivePositions.size() << " ) mismatch !";
		return false;
	}

	mLastUpdateTimeCode = time_code;

	return true;
}

const pxr::GfVec3f MeshContainer::getFaceRestNormal(const PhantomTrimesh::TriFace& face) const {
	auto const& rest_positions = mUsdMeshRestPositions.AsConst();

	return pxr::GfGetNormalized(
		pxr::GfCross(rest_positions[face.indices[1]] - rest_positions[face.indices[0]], rest_positions[face.indices[2]] - rest_positions[face.indices[0]])
	);
}

pxr::GfVec3f MeshContainer::getFaceLiveNormal(const PhantomTrimesh::TriFace& face) const {
	auto const& live_positions = mUsdMeshLivePositions.AsConst();

	return pxr::GfGetNormalized(
		pxr::GfCross(live_positions[face.indices[1]] - live_positions[face.indices[0]], live_positions[face.indices[2]] - live_positions[face.indices[0]])
	);
}

} // namespace Piston
