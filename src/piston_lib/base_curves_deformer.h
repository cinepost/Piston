#ifndef PISTON_LIB_BASE_CURVES_DEFORMER_H_
#define PISTON_LIB_BASE_CURVES_DEFORMER_H_

#include "framework.h"
#include "common.h"
#include "points_list.h"
#include "base_deformer.h"
#include "curves_container.h"
#include "mesh_container.h"
#include "debug_drawing.h"
#include "deformer_stats.h"
#include "deformer_data_cache.h"
#include "serializable_data.h"
#include "simple_profiler.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>

#include <atomic>
#include <memory>
#include <string>
#include <mutex>


namespace Piston {

class BaseCurvesDeformer :public BaseDeformer, public inherit_shared_from_this<BaseDeformer, BaseCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<BaseCurvesDeformer>;

	public:
		virtual ~BaseCurvesDeformer() {}

		virtual const std::string& toString() const override;
		
		// DocString: setCurvesGeoPrim
		/**
		 * @brief Sets the Pixar USD curves primitive that will undergo deformation.
		 * @param prim The USD curves primitive to be deformed.
		 */		
		void setCurvesGeoPrim(const pxr::UsdPrim& prim);
		const pxr::UsdPrim& getCurvesGeoPrim() const;

		void setCurvesRestAttrName(const std::string& name);
		const std::string& getCurvesRestAttrName() const { return mCurvesGeoPrimHandle.getRestAttrName(); }

		// DocString: deform
		/**
		 * @brief Sets the Pixar USD curves primitive that will undergo deformation.
		 * @param prim The USD curves primitive to be deformed.
		 * @return something
		 *
		 */	
		virtual bool deform(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default(), bool multi_threaded = true, bool ignoreVelocities = false) override;
		virtual bool deform_dbg(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default(), bool ignoreVelocities = false) override;

	private:
		virtual bool buildDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false);

	protected:
		BaseCurvesDeformer(const Type type, const std::string& name);

		virtual const UsdPrimHandle& getOutputPrimHandle() const override { return mCurvesGeoPrimHandle; }

	protected:
		UsdPrimHandle 					mCurvesGeoPrimHandle;
		PxrCurvesContainer::UniquePtr 	mpCurvesContainer;

		// we use these containers to store deformed points data when LRU cache is disabled
		std::unique_ptr<PointsList> 	mpDeformedPointsList;
		std::unique_ptr<PointsList> 	mpDeformedPointsListStep;
		std::unique_ptr<PointsList> 	mpTempVelocitiesList;
};

} // namespace Piston

#endif // PISTON_LIB_BASE_CURVES_DEFORMER_H_