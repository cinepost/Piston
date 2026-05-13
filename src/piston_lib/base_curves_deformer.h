#ifndef PISTON_LIB_BASE_CURVES_DEFORMER_H_
#define PISTON_LIB_BASE_CURVES_DEFORMER_H_

#include "framework.h"
#include "common.h"
#include "points_list.h"
#include "curves_container.h"
#include "mesh_container.h"
#include "deformer_stats.h"
#include "deformer_data_cache.h"
#include "serializable_data.h"
#include "simple_profiler.h"

#include "BS_thread_pool.hpp" // BS::multi_future, BS::thread_pool

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>

#include <atomic>
#include <memory>
#include <string>
#include <mutex>


namespace Piston {

namespace {
	const std::string kVelocitiAttrName = "velocities";
	const std::string kDeformerRestPositionAttrName = "";
	const std::string kCurvesRestPositionAttrName = "";

	const std::string kСurvesSkinPrimAttrName = ""; //"skinprim"
}

class BaseCurvesDeformer : public std::enable_shared_from_this<BaseCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<BaseCurvesDeformer>;

		enum class Type { 
			FAST, 
			WRAP,
			GUIDES, 
			UNKNOWN 
		};

		enum class MotionBlurDirection {
			TRAILING,
			CENTERED,
			LEADING
		};
		
	public:
		virtual ~BaseCurvesDeformer() {}

		// DocString: setDeformerGeoPrim
		/**
		 * @brief Sets the Pixar USD primitive used as the deformation geometry.
		 * @param prim The USD primitive to be used for deformation.
		 */
		void setDeformerGeoPrim(const pxr::UsdPrim& prim);
		void setDeformerGeoPrim(const BaseCurvesDeformer::SharedPtr& pDeformer);
		const pxr::UsdPrim& getDeformerGeoPrim() const;

		// DocString: setCurvesGeoPrim
		/**
		 * @brief Sets the Pixar USD curves primitive that will undergo deformation.
		 * @param prim The USD curves primitive to be deformed.
		 */		
		void setCurvesGeoPrim(const pxr::UsdPrim& prim);
		const pxr::UsdPrim& getCurvesGeoPrim() const;

		void setPointsCacheUsageState(bool state);
		bool getPointsCacheUsageState() const;

		void setInstancingState(bool state);
		bool getInstancingState() const;
		
		void setDataPrimPath(const std::string& path);
		const pxr::SdfPath& getDataPrimPath() const;

		void setDeformerRestAttrName(const std::string& name);
		const std::string& getDeformerRestAttrName() const { return mDeformerRestAttrName; }
		void setCurvesRestAttrName(const std::string& name);
		const std::string& getCurvesRestAttrName() const { return mCurvesRestAttrName; }
		void setSkinPrimAttrName(const std::string& name);
		const std::string& getSkinPrimAttrName() const { return mSkinPrimAttrName; }

		void setReadJsonDataFromPrim(bool state);
		bool getReadJsonDataState() const { return mReadJsonDeformerData; }
		
		void setRestTimeCode(pxr::UsdTimeCode time_code);
		pxr::UsdTimeCode getRestTimeCode() const;

		bool writeJsonDataToPrim(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		void setVelocityAttrName(const std::string& name);
		const std::string& getVelocityAttrName() const { return mVelocityAttrName; }


		// DocString: deform
		/**
		 * @brief Sets the Pixar USD curves primitive that will undergo deformation.
		 * @param prim The USD curves primitive to be deformed.
		 * @return something
		 *
		 */	
		bool deform(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default(), bool multi_threaded = true);
		bool deform_dbg(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		const std::string& getName() const { return mName; }

		std::string repr() const;
		virtual const std::string& toString() const;

		const DeformerStats& getStats() const { return mStats; }

		void setMotionBlurState(bool state);

		bool getMotionBlurState() const { return mCalcMotionVectors; }

		void showDebugGeometry(bool state);

	protected:
		BaseCurvesDeformer(const Type type, const std::string& name);

		virtual bool validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) = 0;

		virtual bool deformImpl(PointsList& points, pxr::UsdTimeCode time_code) = 0;
		virtual bool deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) = 0;

		virtual void invalidateData(DeformerDataCache& cache) = 0;

		const UsdPrimHandle& getCurvesGeoPrimHandle() const { return mCurvesGeoPrimHandle; }
		const UsdPrimHandle& getOutputPrimHandle() const { return mCurvesGeoPrimHandle; }

		void makeDirty();
		void clearLRUCaches();

	protected:
		bool mUsePointsCache = true;
		bool mShowDebugGeometry = false;
		bool mDirty = true;
		bool mDeformerDataWritten = false;
		bool mInstancingEnabled = true;
		
		UsdPrimHandle 	mDeformerGeoPrimHandle;
		UsdPrimHandle 	mCurvesGeoPrimHandle;

		std::string 	mDeformerRestAttrName = kDeformerRestPositionAttrName;
		std::string 	mCurvesRestAttrName = kCurvesRestPositionAttrName;
		std::string 	mSkinPrimAttrName = kСurvesSkinPrimAttrName;
		
		std::string   	mVelocityAttrName = kVelocitiAttrName;
		
		PxrCurvesContainer::UniquePtr 	mpCurvesContainer;
		MeshContainer::UniquePtr   		mpDeformerMeshContainer;

		BS::thread_pool<BS::tp::none> mPool;
		DeformerStats mStats;

		std::mutex      mPrmMutex;

		// we use these containers to store deformed points data when LRU cache is disabled
		std::unique_ptr<PointsList> mpDeformedPointsList;
		std::unique_ptr<PointsList> mpDeformedPointsListStep;
		std::unique_ptr<PointsList> mpTempVelocitiesList;

	protected:
		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false) = 0;
		virtual bool writeJsonDataToPrimImpl() const = 0;

		virtual void drawDebugGeometry(pxr::UsdTimeCode time_code) {};

	private:
		bool buildDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false);
		const std::string& uniqueName() const { return mUniqueName; }
		std::string velocityKeyName() const { return uniqueName() + "_vel"; }

		static std::atomic_uint32_t current_id;

		Type mType;
		std::string mName;
		uint32_t mID;
		std::string mUniqueName;

		bool mCalcMotionVectors = false;
		MotionBlurDirection mMotionBlurDirection = MotionBlurDirection::TRAILING;

		pxr::UsdTimeCode mRestTimeCode;
		pxr::SdfPath mDataPrimPath;

		bool mReadJsonDeformerData = false;
		bool mWriteJsonDeformerData = false;

		friend class UsdPrimHandle;
};

} // namespace Piston

std::string to_string(const Piston::BaseCurvesDeformer::Type& mt);

#endif // PISTON_LIB_BASE_CURVES_DEFORMER_H_