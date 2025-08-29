#ifndef PISTON_LIB_WRAP_CURVES_DEFORMER_H_
#define PISTON_LIB_WRAP_CURVES_DEFORMER_H_

#include "framework.h"
#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "curves_container.h"
#include "geometry_tools.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <glm/vec3.hpp> // glm::vec3


namespace Piston {

class WrapCurvesDeformer : public BaseCurvesDeformer, public inherit_shared_from_this<BaseCurvesDeformer, WrapCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<WrapCurvesDeformer>;
		using PxrIndexType = int;

		enum class BindMode: uint8_t { 
			SPACE, 
			DIST 
		};

	private:
		struct PointBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			static constexpr float kFltMax = std::numeric_limits<float>::max(); 
			uint32_t face_id;
			float u, v, dist;
			PointBindData(): face_id(kInvalidFaceID), dist(kFltMax) {};

			bool isValid() const { return face_id != kInvalidFaceID; }
		};

	public:
		~WrapCurvesDeformer();

		static SharedPtr create();
		virtual const std::string& toString() const override;

		void setBindMode(BindMode mode);
		const BindMode& getBindMode() const { return mBindMode; } 

	protected:
		WrapCurvesDeformer();
		virtual bool deformImpl(pxr::UsdTimeCode time_code);

		bool deformImpl_SpaceMode(std::vector<pxr::GfVec3f>& points, pxr::UsdTimeCode time_code);
		bool deformImpl_DistMode(std::vector<pxr::GfVec3f>& points, pxr::UsdTimeCode time_code);

	private:
		virtual bool buildDeformerData(pxr::UsdTimeCode rest_time_code) override;
		
		bool buildDeformerData_SpaceMode(const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code);
		bool buildDeformerData_DistMode(const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code);

		BindMode                                            mBindMode = BindMode::SPACE;

		UsdGeomMeshFaceAdjacency::SharedPtr					mpAdjacency;
		PhantomTrimesh<PxrIndexType>::UniquePtr 			mpPhantomTrimesh;
		
		std::vector<PointBindData>               			mPointBinds;

		std::vector<pxr::GfVec3f> 							mLiveVertexNormals;
};

} // namespace Piston

#endif // PISTON_LIB_WRAP_CURVES_DEFORMER_H_