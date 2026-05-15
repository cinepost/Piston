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

struct TemplatedMeshContainerBase {
    using PointType = pxr::GfVec3f;
};

template <typename T>
class TemplatedMeshContainer: public TemplatedMeshContainerBase {
	public:
		using UniquePtr = std::unique_ptr<TemplatedMeshContainer<T>>;
		using ContainerType = T;

		TemplatedMeshContainer();
		TemplatedMeshContainer(const TemplatedMeshContainer<T>& other);
		TemplatedMeshContainer(TemplatedMeshContainer<T>&& other);

		bool operator==(const TemplatedMeshContainer<T>& other) const {
			return mUsdMeshRestPositions == other.mUsdMeshRestPositions && mUsdMeshLivePositions == other.mUsdMeshLivePositions;
		}

		void makeUnique();

	public:
		static UniquePtr create(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		bool init(const UsdPrimHandle& prim_handle, const std::string& rest_p_name, pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		const T& getRestPositions() const {
			if constexpr (std::is_same_v<T, pxr::VtArray<PointType>>) {
				return mUsdMeshRestPositions.AsConst();
			} else {
				return mUsdMeshRestPositions;
			}
		}
		
		const T& getLivePositions() const { 
			if constexpr (std::is_same_v<T, pxr::VtArray<PointType>>) {
				return mUsdMeshLivePositions.AsConst(); 
			} else {
				return mUsdMeshLivePositions;
			}

		}

		const PointType& getRestPointPosition(size_t i) const { assert(i < getRestPositions().size()); return getRestPositions()[i]; }
		const PointType& getLivePointPosition(size_t i) const { assert(i < getLivePositions().size()); return getLivePositions()[i]; }

		size_t getPointsCount() const { return getRestPositions().size(); }

		bool projectPoint(const pxr::GfVec3f& pt, const PhantomTrimesh::TriFace& face, float& u, float& v) const;
		bool projectPoint(const pxr::GfVec3f& pt, const PhantomTrimesh::TriFace& face, float& u, float& v, float& dist) const;
		bool intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const PhantomTrimesh::TriFace& face, float& u, float& v) const;
		bool intersectRay(const pxr::GfVec3f& orig, const pxr::GfVec3f& dir, const PhantomTrimesh::TriFace& face, float& u, float& v, float& dist) const;

		PointType getInterpolatedRestPosition(const PhantomTrimesh::TriFace& face, const float u, const float v) const;
		PointType getInterpolatedLivePosition(const PhantomTrimesh::TriFace& face, const float u, const float v) const;

		PointType getInterpolatedRestPosition(const PhantomTrimesh::TriFace& face, const float u, const float v, const float w) const;
		PointType getInterpolatedLivePosition(const PhantomTrimesh::TriFace& face, const float u, const float v, const float w) const;

		PointType getFaceRestCentroid(const PhantomTrimesh::TriFace& face) const;

		const PointType getFaceRestNormal(const PhantomTrimesh::TriFace& face) const;
		PointType getFaceLiveNormal(const PhantomTrimesh::TriFace& face) const;

		PointType getTetrahedronRestCentroid(const PhantomTrimesh::Tetrahedron& t) const;
		void barycentricTetrahedronRestCoords(const PhantomTrimesh::Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w, float& x) const;

		void barycentricTetrahedronRestCoords(const PhantomTrimesh::Tetrahedron& t, const pxr::GfVec3f& p, float& u, float& v, float& w) const;

		PointType getPointPositionFromBarycentricTetrahedronLiveCoords(const PhantomTrimesh::Tetrahedron& t, float u, float v, float w, float x) const;

		bool update(const UsdPrimHandle& prim_handle, pxr::UsdTimeCode time_code, bool force) const;

		pxr::UsdTimeCode getLastUpdateTimeCode() const { return mLastUpdateTimeCode; }
		
	private:
		mutable pxr::UsdTimeCode mLastUpdateTimeCode;

		T 			mUsdMeshRestPositions;
		mutable T 	mUsdMeshLivePositions;
};

using MeshContainer = TemplatedMeshContainer<pxr::VtArray<TemplatedMeshContainerBase::PointType>>;

} // namespace Piston

#endif // PISTON_LIB_MESH_CONTAINER_H_