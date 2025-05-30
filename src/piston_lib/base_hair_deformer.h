#ifndef PISTON_LIB_BASE_HAIR_DEFORMER_H_
#define PISTON_LIB_BASE_HAIR_DEFORMER_H_

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
	const std::string kHairToMeshBindingAttrName = "bind_to_tri";
}

class BaseHairDeformer : public std::enable_shared_from_this<BaseHairDeformer> {
	public:
		using SharedPtr = std::shared_ptr<BaseHairDeformer>;

		enum class Type { 
			FAST, 
			INTERPOLATED, 
			VOLUMETRIC,
			UNKNOWN 
		};
		
	public:
		void setMeshGeoPrim(pxr::UsdPrim* pGeoPrim);
		void setHairGeoPrim(pxr::UsdPrim* pGeoPrim);

		void setMeshRestPositionAttrName(const std::string& name);
		const std::string& getMeshRestPositionAttrName() const { return mRestPositionAttrName; }

		bool deform(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		virtual const std::string& toString() const;

	protected:
		BaseHairDeformer();

		virtual bool deformImpl(pxr::UsdTimeCode time_code) = 0;

	protected:
		UsdPrimHandle mMeshGeoPrimHandle;
		UsdPrimHandle mHairGeoPrimHandle;

		std::string   mRestPositionAttrName = "rest_p";
		std::string   mHairToMeshBindingAttrName = kHairToMeshBindingAttrName;
		
		PxrCurvesContainer::UniquePtr mpCurvesContainer;

	private:
		virtual bool buildDeformerData(pxr::UsdTimeCode reference_time_code) = 0;

		pxr::UsdStageRefPtr mpTempStage;
		bool mDirty = true;

};

} // namespace Piston

inline std::string to_string(Piston::BaseHairDeformer::Type mt) {
#define t2s(t_) case Piston::BaseHairDeformer::Type::t_: return #t_;
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

#endif // PISTON_LIB_BASE_HAIR_DEFORMER_H_