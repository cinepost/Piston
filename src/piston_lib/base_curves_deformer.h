#ifndef PISTON_LIB_BASE_CURVES_DEFORMER_H_
#define PISTON_LIB_BASE_CURVES_DEFORMER_H_

#include "framework.h"
#include "common.h"
#include "curves_container.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>

namespace Piston {

namespace {
	const std::string kMeshRestPositionAttrName = "rest_p";
	const std::string kСurvesSkinPrimAttrName = "skinprim";
}

class BaseCurvesDeformer : public std::enable_shared_from_this<BaseCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<BaseCurvesDeformer>;

		enum class Type { 
			FAST, 
			INTERPOLATED, 
			VOLUMETRIC,
			UNKNOWN 
		};
		
	public:
		virtual ~BaseCurvesDeformer() {}

		void setMeshGeoPrim(pxr::UsdPrim* pGeoPrim);
		void setCurvesGeoPrim(pxr::UsdPrim* pGeoPrim);

		void setMeshRestPositionAttrName(const std::string& name);
		const std::string& getMeshRestPositionAttrName() const { return mMeshRestPositionAttrName; }

		void setСurvesSkinPrimAttrName(const std::string& name);
		const std::string& getСurvesSkinPrimAttrName() const { return mСurvesSkinPrimAttrName; }

		bool deform(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());
		bool deform_mp(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		virtual const std::string& toString() const;

	protected:
		BaseCurvesDeformer();

		virtual bool deformImpl(pxr::UsdTimeCode time_code) = 0;

	protected:
		bool mDirty = true;
		
		UsdPrimHandle mMeshGeoPrimHandle;
		UsdPrimHandle mCurvesGeoPrimHandle;

		std::string   mMeshRestPositionAttrName = kMeshRestPositionAttrName;
		std::string   mСurvesSkinPrimAttrName = kСurvesSkinPrimAttrName;
		
		PxrCurvesContainer::UniquePtr mpCurvesContainer;
	private:
		virtual bool buildDeformerData(pxr::UsdTimeCode reference_time_code) = 0;

		pxr::UsdStageRefPtr mpTempStage;
};

} // namespace Piston

inline std::string to_string(Piston::BaseCurvesDeformer::Type mt) {
#define t2s(t_) case Piston::BaseCurvesDeformer::Type::t_: return #t_;
    switch (mt) {
        t2s(FAST);
        t2s(INTERPOLATED);
        t2s(VOLUMETRIC);
        default:
            should_not_get_here();
            return "UNKNOWN";
    }
#undef t2s
}

#endif // PISTON_LIB_BASE_CURVES_DEFORMER_H_