#ifndef PISTON_LIB_MESH_CONTAINER_H_
#define PISTON_LIB_MESH_CONTAINER_H_

#include "framework.h"
#include "common.h"
#include "kdtree.hpp"
#include "phantom_trimesh.h"

#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/base/gf/matrix3f.h>

#include <limits>
#include <string>
#include <array>
#include <unordered_map>
#include <memory>
#include <mutex>


namespace Piston {

class MeshContainer {
	public:
		using UniquePtr = std::unique_ptr<MeshContainer>;

		MeshContainer();

	public:
		static MeshContainer::UniquePtr create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		bool init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		const pxr::VtArray<pxr::GfVec3f>& getRestPositions() const { return mUsdMeshRestPositions.AsConst(); }
		const pxr::VtArray<pxr::GfVec3f>& getLivePositions() const { return mUsdMeshLivePositions.AsConst(); }

		const pxr::GfVec3f& getRestPointPosition(size_t i) const { assert(i < mUsdMeshRestPositions.size()); return mUsdMeshRestPositions.AsConst()[i]; }
		const pxr::GfVec3f& getLivePointPosition(size_t i) const { assert(i < mUsdMeshLivePositions.size()); return mUsdMeshLivePositions.AsConst()[i]; }

		size_t getPointsCount() const { return mUsdMeshRestPositions.size(); }

		bool projectPoint(const pxr::GfVec3f& pt, const PhantomTrimesh::TriFace& face, float& u, float& v) const;
		bool projectPoint(const pxr::GfVec3f& pt, const PhantomTrimesh::TriFace& face, float& u, float& v, float& dist) const;
		bool intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const PhantomTrimesh::TriFace& face, float& u, float& v) const;
		bool intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const PhantomTrimesh::TriFace& face, float& u, float& v, float& dist) const;

		pxr::GfVec3f getInterpolatedRestPosition(const PhantomTrimesh::TriFace& face, const float u, const float v) const;
		pxr::GfVec3f getInterpolatedLivePosition(const PhantomTrimesh::TriFace& face, const float u, const float v) const;

		pxr::GfVec3f getInterpolatedRestPosition(const PhantomTrimesh::TriFace& face, const float u, const float v, const float w) const;
		pxr::GfVec3f getInterpolatedLivePosition(const PhantomTrimesh::TriFace& face, const float u, const float v, const float w) const;

		pxr::GfVec3f getFaceRestCentroid(const PhantomTrimesh::TriFace& face) const;

		const pxr::GfVec3f getFaceRestNormal(const PhantomTrimesh::TriFace& face) const;
		pxr::GfVec3f getFaceLiveNormal(const PhantomTrimesh::TriFace& face) const;

		pxr::GfVec3f getTetrahedronRestCentroid(const PhantomTrimesh::Tetrahedron& t) const;
		void barycentricTetrahedronRestCoords(const PhantomTrimesh::Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w, float& x) const;

		void barycentricTetrahedronRestCoords(const PhantomTrimesh::Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w) const;

		pxr::GfVec3f getPointPositionFromBarycentricTetrahedronLiveCoords(const PhantomTrimesh::Tetrahedron& t, float u, float v, float w, float x) const;

		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code) const;
		
	private:
		mutable pxr::UsdTimeCode mLastUpdateTimeCode;

		pxr::VtArray<pxr::GfVec3f> 								mUsdMeshRestPositions;
		mutable pxr::VtArray<pxr::GfVec3f> 						mUsdMeshLivePositions;
};

} // namespace Piston

#endif // PISTON_LIB_MESH_CONTAINER_H_