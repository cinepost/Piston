#ifndef PISTON_LIB_WRAP_CURVES_DEFORMER_H_
#define PISTON_LIB_WRAP_CURVES_DEFORMER_H_

#include "framework.h"
#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "curves_container.h"
#include "geometry_tools.h"
#include "wrap_curves_deformer_data.h"

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

		using BindMode = WrapCurvesDeformerData::BindMode;
		using PointBindData = WrapCurvesDeformerData::PointBindData;

	public:
		~WrapCurvesDeformer();

		static SharedPtr create(const std::string& name);
		virtual const std::string& toString() const override;

		void setBindMode(BindMode mode);
		const BindMode& getBindMode() const;

	protected:
		WrapCurvesDeformer(const std::string& name);
		virtual bool deformImpl(PointsList& points, pxr::UsdTimeCode time_code) override;
		virtual bool deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) override;

		bool deformImpl_SpaceMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code);
		bool deformImpl_DistMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code);

	private:
		bool __deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code);

		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false) override;
		virtual bool writeJsonDataToPrimImpl()const override;
		
		bool buildDeformerData_SpaceMode(const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code);
		bool buildDeformerData_DistMode(const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code);

		std::unique_ptr<WrapCurvesDeformerData> mpWrapCurvesDeformerData;

		std::vector<pxr::GfVec3f> 				mLiveVertexNormals;
		std::vector<pxr::GfVec3f> 				mLiveTriFaceNormals;
};

} // namespace Piston

#endif // PISTON_LIB_WRAP_CURVES_DEFORMER_H_