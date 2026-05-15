#include "mesh_container.h"
#include "geometry_tools.h"
#include "simple_profiler.h"
#include "piston_math.h"
#include "logging.h"

#include <limits>

namespace Piston {

template <typename T>
typename TemplatedMeshContainer<T>::UniquePtr TemplatedMeshContainer<T>::create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	auto pMeshContainer = std::make_unique<TemplatedMeshContainer<T>>();
	if(!pMeshContainer->init(prim_handle, rest_p_name, time_code)) {
		return nullptr;
	}
	return pMeshContainer;
}

template <typename T>
TemplatedMeshContainer<T>::TemplatedMeshContainer(): mLastUpdateTimeCode(std::numeric_limits<double>::lowest())  {
	static_assert(std::is_same_v<T, std::vector<PointType>> || std::is_same_v<T, pxr::VtArray<PointType>>, "Only std::vector<PointType> and pxr::VtArray<PointType> types are permitted!");
}

template <typename T>
TemplatedMeshContainer<T>::TemplatedMeshContainer(const TemplatedMeshContainer<T>& other) {
	mUsdMeshRestPositions = other.mUsdMeshRestPositions;
	mUsdMeshLivePositions = other.mUsdMeshLivePositions;
	mLastUpdateTimeCode = other.mLastUpdateTimeCode;
}

template <typename T>
TemplatedMeshContainer<T>::TemplatedMeshContainer(TemplatedMeshContainer<T>&& other) {
	mUsdMeshRestPositions = std::move(other.mUsdMeshRestPositions);
	mUsdMeshLivePositions = std::move(other.mUsdMeshLivePositions);
	mLastUpdateTimeCode = other.mLastUpdateTimeCode;
}

template <typename T>
bool TemplatedMeshContainer<T>::init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code) {
	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());

	pxr::UsdGeomPrimvarsAPI meshPrimvarsApi = prim_handle.getPrimvarsAPI();
	pxr::UsdGeomPrimvar restPositionPrimVar = meshPrimvarsApi.GetPrimvar(pxr::TfToken(rest_p_name));
		
	if(!restPositionPrimVar) {
		LOG_WRN << "No valid primvar \"" << rest_p_name << "\" exists in prim " << prim_handle.getPath() << "! Using positions at time code 0.0 !";

		pxr::UsdGeomPointBased mesh(prim_handle.getPrim());

		static const pxr::UsdTimeCode s_zero_time_code(0.0);

		if constexpr (std::is_same_v<T, pxr::VtArray<PointType>>) {
			if(!mesh.GetPointsAttr().Get(&mUsdMeshRestPositions, s_zero_time_code)) {
				LOG_ERR << "Error getting " << prim_handle.getPath() << " point positions at time code " << s_zero_time_code.GetValue();
				return false;
			}
		} else {
			static_assert(std::is_same_v<T, std::vector<PointType>>);
			pxr::VtArray<PointType> tmp;
			if(!mesh.GetPointsAttr().Get(&tmp, s_zero_time_code)) {
				LOG_ERR << "Error getting " << prim_handle.getPath() << " point positions at time code " << s_zero_time_code.GetValue();
				return false;
			}
			mUsdMeshRestPositions.resize(tmp.size());
			for(size_t i = 0; i < tmp.size(); ++i) mUsdMeshRestPositions[i] = tmp[i];
		}
	} else {
		const pxr::UsdAttribute& restPosAttr = restPositionPrimVar.GetAttr();
	
		if constexpr (std::is_same_v<T, pxr::VtArray<PointType>>) {
			if(!restPosAttr.Get(&mUsdMeshRestPositions, time_code)) {
				LOG_ERR << "Error getting prim " << prim_handle.getPath() << " \"rest\" positions !";
				return false;
			}
		} else {
			static_assert(std::is_same_v<T, std::vector<PointType>>);
			pxr::VtArray<PointType> tmp;
			if(!restPosAttr.Get(&tmp, time_code)) {
				LOG_ERR << "Error getting prim " << prim_handle.getPath() << " \"rest\" positions !";
				return false;
			}
			mUsdMeshRestPositions.resize(tmp.size());
			for(size_t i = 0; i < tmp.size(); ++i) mUsdMeshRestPositions[i] = tmp.AsConst()[i];
		}
		LOG_DBG << "Prim " << prim_handle.getPath() << " has " << mUsdMeshRestPositions.size() << " rest positions.";
	}

	mUsdMeshLivePositions = mUsdMeshRestPositions;
	mLastUpdateTimeCode = time_code;
	return true;
}

template <typename T>
TemplatedMeshContainerBase::PointType TemplatedMeshContainer<T>::getTetrahedronRestCentroid(const PhantomTrimesh::Tetrahedron& t) const {
	assert(t.isValid());

	const auto& rest_positions = getRestPositions();
	return (rest_positions[t.indices[0]] + rest_positions[t.indices[1]] + rest_positions[t.indices[2]] + rest_positions[t.indices[3]]) / 4.0f;
}

template <typename T>
void TemplatedMeshContainer<T>::barycentricTetrahedronRestCoords(const PhantomTrimesh::Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w, float& x) const {
	assert(t.isValid());

	const auto& rest_positions = getRestPositions();

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

template <typename T>
void TemplatedMeshContainer<T>::barycentricTetrahedronRestCoords(const PhantomTrimesh::Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w) const {
	float _temp_x;
	barycentricTetrahedronRestCoords(t, p, u, v, w, _temp_x);
}

template <typename T>
TemplatedMeshContainerBase::PointType TemplatedMeshContainer<T>::getPointPositionFromBarycentricTetrahedronLiveCoords(const PhantomTrimesh::Tetrahedron& t, float u, float v, float w, float x) const {
	assert(t.isValid());

	const auto& live_positions = getLivePositions();

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

template <typename T>
bool TemplatedMeshContainer<T>::projectPoint(const pxr::GfVec3f& pt, const PhantomTrimesh::TriFace& face, float& u, float& v) const {
	const auto& rest_positions = getRestPositions();
	return rayTriangleIntersect(pt, -getFaceRestNormal(face), rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]], u, v);
}

template <typename T>
bool TemplatedMeshContainer<T>::projectPoint(const pxr::GfVec3f& pt, const PhantomTrimesh::TriFace& face, float& u, float& v, float& dist) const {
	const auto& rest_positions = getRestPositions();
	return rayTriangleIntersect(pt, -getFaceRestNormal(face), rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]], dist, u, v);
}

template <typename T>
bool TemplatedMeshContainer<T>::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const PhantomTrimesh::TriFace& face, float& u, float& v) const {
	const auto& rest_positions = getRestPositions();
	return rayTriangleIntersect(orig, dir, rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]], u, v);
}

template <typename T>
bool TemplatedMeshContainer<T>::intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const PhantomTrimesh::TriFace& face, float& u, float& v, float& dist) const {
	const auto& rest_positions = getRestPositions();
	return rayTriangleIntersect(orig, dir, rest_positions[face.indices[0]], rest_positions[face.indices[1]], rest_positions[face.indices[2]], dist, u, v);
}

template <typename T>
pxr::GfVec3f TemplatedMeshContainer<T>::getInterpolatedRestPosition(const PhantomTrimesh::TriFace& face, const float u, const float v) const {
	return getInterpolatedRestPosition(face, u, v, (1.f - u - v));
};

template <typename T>
pxr::GfVec3f TemplatedMeshContainer<T>::getInterpolatedLivePosition(const PhantomTrimesh::TriFace& face, const float u, const float v) const {
	return getInterpolatedLivePosition(face, u, v, (1.f - u - v));
};

template <typename T>
TemplatedMeshContainerBase::PointType TemplatedMeshContainer<T>::getInterpolatedRestPosition(const PhantomTrimesh::TriFace& face, const float u, const float v, const float w) const {
	const auto& rest_positions = getRestPositions();

	return u * rest_positions[face.indices[1]] + v * rest_positions[face.indices[2]] + w * rest_positions[face.indices[0]];
}

template <typename T>
TemplatedMeshContainerBase::PointType TemplatedMeshContainer<T>::getInterpolatedLivePosition(const PhantomTrimesh::TriFace& face, const float u, const float v, const float w) const {
	const auto& live_positions = getLivePositions();

	return u * live_positions[face.indices[1]] + v * live_positions[face.indices[2]] + w * live_positions[face.indices[0]];
}

template <typename T>
TemplatedMeshContainerBase::PointType TemplatedMeshContainer<T>::getFaceRestCentroid(const PhantomTrimesh::TriFace& face) const {
	static constexpr float kInvThree = 1.f / 3.f;
	const auto& rest_positions = getRestPositions();

	return (rest_positions[face.indices[0]] + rest_positions[face.indices[1]] + rest_positions[face.indices[2]]) * kInvThree;
}

template <typename T>
bool TemplatedMeshContainer<T>::update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force) const {
	assert(prim_handle.isMeshGeoPrim() || prim_handle.isBasisCurvesGeoPrim());

	pxr::UsdAttribute attr;

	if(!force) {
		if(mLastUpdateTimeCode == time_code) return true;
		
		pxr::UsdGeomPointBased mesh(prim_handle.getPrim());
		attr = mesh.GetPointsAttr();
		if (!attr.ValueMightBeTimeVarying()) return true;
	}

	if(!attr) attr = pxr::UsdGeomPointBased(prim_handle.getPrim()).GetPointsAttr();

	if constexpr (std::is_same_v<T, pxr::VtArray<PointType>>) {
		if(!attr.Get(&mUsdMeshLivePositions, time_code)) {
			LOG_ERR << "Error getting point positions from " << prim_handle.getPath() << " !";
			return false;
		}
	} else {
		static_assert(std::is_same_v<T, std::vector<PointType>>);
		pxr::VtArray<PointType> tmp;
		if(!attr.Get(&tmp, time_code)) {
			LOG_ERR << "Error getting point positions from " << prim_handle.getPath() << " !";
			return false;
		}
		mUsdMeshLivePositions.resize(tmp.size());
		for(size_t i = 0; i < mUsdMeshLivePositions.size(); ++i) mUsdMeshLivePositions[i] = tmp.AsConst()[i];
	}

	if(mUsdMeshLivePositions.size() != mUsdMeshRestPositions.size()) {
		LOG_ERR << prim_handle.getPath() << " \"rest\" and \"live\" mesh point positions count (" << mUsdMeshRestPositions.size() << " vs " << mUsdMeshLivePositions.size() << " ) mismatch !";
		return false;
	}

	mLastUpdateTimeCode = time_code;

	return true;
}

template <typename T>
void TemplatedMeshContainer<T>::makeUnique() {
	if constexpr (std::is_same_v<T, pxr::VtArray<PointType>>) {
		const auto& tmp = mUsdMeshLivePositions;
		mUsdMeshLivePositions = pxr::VtArray<PointType> (tmp.cbegin(), tmp.cend());
	}
}

template <typename T>
const TemplatedMeshContainerBase::PointType TemplatedMeshContainer<T>::getFaceRestNormal(const PhantomTrimesh::TriFace& face) const {
	const auto& rest_positions = getRestPositions();

	return pxr::GfGetNormalized(
		pxr::GfCross(rest_positions[face.indices[1]] - rest_positions[face.indices[0]], rest_positions[face.indices[2]] - rest_positions[face.indices[0]])
	);
}

template <typename T>
TemplatedMeshContainerBase::PointType TemplatedMeshContainer<T>::getFaceLiveNormal(const PhantomTrimesh::TriFace& face) const {
	const auto& live_positions = getLivePositions();

	return pxr::GfGetNormalized(
		pxr::GfCross(live_positions[face.indices[1]] - live_positions[face.indices[0]], live_positions[face.indices[2]] - live_positions[face.indices[0]])
	);
}

template class Piston::TemplatedMeshContainer<pxr::VtArray<Piston::TemplatedMeshContainerBase::PointType>>;
template class Piston::TemplatedMeshContainer<std::vector<Piston::TemplatedMeshContainerBase::PointType>>;

} // namespace Piston
