#ifndef PISTON_LIB_GUIDES_CURVES_DEFORMER_H_
#define PISTON_LIB_GUIDES_CURVES_DEFORMER_H_

#include "framework.h"
#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "guide_curves_container.h"
#include "guide_curves_deformer_data.h"
#include "geometry_tools.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <glm/vec3.hpp> // glm::vec3


namespace Piston {

namespace {
	const std::string kGuideIDPrimAttrName = "clumpid";
	const std::string kGuidesSkinPrimAttrName = "skinprim";
	const std::string kGuidesSkinPrimRestAttrName = "rest";
}


class GuideCurvesDeformer : public BaseCurvesDeformer, public inherit_shared_from_this<BaseCurvesDeformer, GuideCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<GuideCurvesDeformer>;

		using BindMode = GuideCurvesDeformerData::BindMode;
		using PointBindData = GuideCurvesDeformerData::PointBindData;

	public:
		~GuideCurvesDeformer();

		static SharedPtr create(const std::string& name);
		virtual const std::string& toString() const override;

		void setBindMode(BindMode mode);
		BindMode getBindMode() const;

		void setGuideIDPrimAttrName(const std::string& name);
		const std::string& getGuideIDPrimAttrName() const { return mGuideIDPrimAttrName; }

		void setGuidesSkinPrimAttrName(const std::string& name);
		const std::string& getGuidesSkinPrimAttrName() const { return mGuidesSkinPrimAttrName; }

		void setGuidesSkinPrim(const pxr::UsdPrim& geoPrim);

		void setGuidesSkinPrimRestAttrName(const std::string& name);
		const std::string& getGuidesSkinPrimRestAttrName() const { return mGuidesSkinPrimRestAttrName; }

	protected:
		GuideCurvesDeformer(const std::string& name);

		virtual bool validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) override;

		virtual bool deformImpl(PointsList& points, pxr::UsdTimeCode time_code) override;
		virtual bool deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) override;

	private:
		bool __deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code);

		bool buildSkinPrimData(bool multi_threaded);
		bool hasSkinPrimitiveData() const;

		bool buildNTBFrames(std::vector<NTBFrame>& guide_frames, bool multi_threaded, bool build_live);

		bool buildDeformerDataNTBMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded);
		bool buildDeformerDataAngleMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded);
		bool buildDeformerDataSpaceMode(pxr::UsdTimeCode rest_time_code, bool multi_threaded);

		bool deformImpl_SpaceMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code);
		bool deformImpl_AngleMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code);
		bool deformImpl_NTBMode(bool multi_threaded, PointsList& points, pxr::UsdTimeCode time_code);

		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false);
		virtual bool writeJsonDataToPrimImpl() const;

		std::unique_ptr<GuideCurvesDeformerData>   		mpGuideCurvesDeformerData;
		GuideCurvesContainer::UniquePtr 				mpGuideCurvesContainer;
		
		UsdPrimHandle 									mGuidesSkinGeoPrimHandle;
		SerializableUsdGeomMeshFaceAdjacency::UniquePtr mpSkinAdjacencyData;
		SerializablePhantomTrimesh::UniquePtr			mpSkinPhantomTrimeshData;

		SerializablePhantomTrimesh::UniquePtr			mpGuidesPhantomTrimeshData;


		std::string 									mGuideIDPrimAttrName = kGuideIDPrimAttrName;
		std::string 									mGuidesSkinPrimAttrName = kGuidesSkinPrimAttrName;
		std::string                                     mGuidesSkinPrimRestAttrName = kGuidesSkinPrimRestAttrName;
		pxr::VtArray<int> 								mGuideIndices;

		float                                       	mFalloff = .0f;
};

} // namespace Piston

#endif // PISTON_LIB_GUIDES_CURVES_DEFORMER_H_