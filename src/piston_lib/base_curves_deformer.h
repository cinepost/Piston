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

	protected:
		BaseCurvesDeformer(const Type type, const std::string& name);

		virtual bool deformImpl(PointsList& points, pxr::UsdTimeCode time_code) override {
			assert(mpCurvesContainer);
			return mpCurvesContainer->update(mCurvesGeoPrimHandle, time_code, isDirty());
		}

		virtual bool deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) override {
			assert(mpCurvesContainer);
			return mpCurvesContainer->update(mCurvesGeoPrimHandle, time_code, isDirty());
		}

		virtual const UsdPrimHandle& getOutputPrimHandle() const override { return mCurvesGeoPrimHandle; }

		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false) override;

		virtual size_t getDeformedPointsCount() const override final; 
		virtual bool outputDeformedPoints(const PointsList* pPointsList, pxr::UsdTimeCode time_code) override final;
		virtual bool outputVelocites(const PointsList* pVelocitiesList, pxr::UsdTimeCode time_code) override final;

	protected:
		UsdPrimHandle 					mCurvesGeoPrimHandle;
		PxrCurvesContainer::UniquePtr 	mpCurvesContainer;
};

} // namespace Piston

#endif // PISTON_LIB_BASE_CURVES_DEFORMER_H_