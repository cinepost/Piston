#ifndef PISTON_LIB_FAST_CURVES_DEFORMER_H_
#define PISTON_LIB_FAST_CURVES_DEFORMER_H_

#include "framework.h"
#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "curves_container.h"
#include "fast_curves_deformer_data.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <glm/vec3.hpp> // glm::vec3


namespace Piston {

class FastCurvesDeformer : public BaseCurvesDeformer, public inherit_shared_from_this<BaseCurvesDeformer, FastCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<FastCurvesDeformer>;
		using CurveBindData = FastCurvesDeformerData::CurveBindData;

	public:
		~FastCurvesDeformer();

		static SharedPtr create();
		virtual const std::string& toString() const override;

	protected:
		FastCurvesDeformer();
		virtual bool deformImpl(pxr::UsdTimeCode time_code) override;
		virtual bool deformMtImpl(pxr::UsdTimeCode time_code) override;

	private:
		bool __deform__(bool multi_threaded, pxr::UsdTimeCode time_code);

		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code) override;
		virtual bool writeJsonDataToPrimImpl() const override;

		bool buildCurvesBindingData(pxr::UsdTimeCode rest_time_code);
		bool calcPerBindNormals(bool build_live);
		bool calcPerBindTangentsAndBiNormals(bool build_live);

		void transformCurvesToNTB();

		bool bindCurveToTriface(uint32_t curve_index, uint32_t face_id, CurveBindData& bind);

		std::unique_ptr<FastCurvesDeformerData>             mpFastCurvesDeformerData;

		std::vector<pxr::GfVec3f> 							mLiveVertexNormals;
		std::vector<pxr::GfVec3f>               			mPerBindLiveNormals; // we keep memeory to save on per-frame reallocations
		std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>   mPerBindLiveTBs; // we keep memeory to save on per-frame reallocations
};

} // namespace Piston

#endif // PISTON_LIB_FAST_CURVES_DEFORMER_H_