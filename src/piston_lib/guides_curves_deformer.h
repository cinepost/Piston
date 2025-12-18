#ifndef PISTON_LIB_GUIDES_CURVES_DEFORMER_H_
#define PISTON_LIB_GUIDES_CURVES_DEFORMER_H_

#include "framework.h"
#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "curves_container.h"
#include "guides_curves_deformer_data.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <glm/vec3.hpp> // glm::vec3


namespace Piston {

class GuidesCurvesDeformer : public BaseCurvesDeformer, public inherit_shared_from_this<BaseCurvesDeformer, GuidesCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<GuidesCurvesDeformer>;
		using PointBindData = GuidesCurvesDeformerData::PointBindData;

	public:
		~GuidesCurvesDeformer();

		static SharedPtr create(const std::string& name);
		virtual const std::string& toString() const override;

	protected:
		GuidesCurvesDeformer(const std::string& name);

		virtual bool validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) override;

		virtual bool deformImpl(PointsList& points, pxr::UsdTimeCode time_code) override;
		virtual bool deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) override;

	private:
		bool __deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code);

		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false) override;
		virtual bool writeJsonDataToPrimImpl() const override;

		std::unique_ptr<GuidesCurvesDeformerData>             mpGuidesCurvesDeformerData;
};

} // namespace Piston

#endif // PISTON_LIB_GUIDES_CURVES_DEFORMER_H_