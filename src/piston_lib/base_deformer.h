#ifndef PISTON_LIB_BASE_DEFORMER_H_
#define PISTON_LIB_BASE_DEFORMER_H_

#include "framework.h"
#include "common.h"
#include "points_list.h"
#include "curves_container.h"
#include "mesh_container.h"
#include "debug_drawing.h"
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
	const std::string kСurvesSkinPrimAttrName = ""; //"skinprim"
}

class BaseDeformer : public std::enable_shared_from_this<BaseDeformer> {
	public:
		using SharedPtr = std::shared_ptr<BaseDeformer>;

		enum class Type { 
			FAST, 
			WRAP,
			GUIDES,
			POINT_INSTANCER,
			UNKNOWN 
		};

		enum class MotionBlurDirection {
			TRAILING,
			CENTERED,
			LEADING
		};
		
	public:
		virtual ~BaseDeformer() {}

		// DocString: setDeformerGeoPrim
		/**
		 * @brief Sets the Pixar USD primitive used as the deformation geometry.
		 * @param prim The USD primitive to be used for deformation.
		 */
		void setDeformerGeoPrim(const pxr::UsdPrim& prim);
		void setDeformerGeoPrim(const BaseDeformer::SharedPtr& pDeformer);
		const pxr::UsdPrim& getDeformerGeoPrim() const;

		void setDeformerSubdivLevel(uint8_t level = 0);
		uint8_t getDeformerSubdivLevel() const; 

		void setPointsCacheUsageState(bool state);
		bool getPointsCacheUsageState() const;

		void setInstancingState(bool state);
		bool getInstancingState() const;
		
		void setDataPrimPath(const std::string& path);
		const pxr::SdfPath& getDataPrimPath() const;

		void setDeformerRestAttrName(const std::string& name);
		const std::string& getDeformerRestAttrName() const { return mDeformerGeoPrimHandle.getRestAttrName(); }

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
		virtual bool deform(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default(), bool multi_threaded = true, bool ignoreVelocities = false);
		bool deform_dbg(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default(), bool ignoreVelocities = false);

		const std::string& getName() const { return mName; }

		std::string repr() const;
		virtual const std::string& toString() const;

		const DeformerStats& getStats() const { return mStats; }

		void setMotionBlurState(bool state);

		bool getMotionBlurState() const { return mCalcMotionVectors; }

		void showDebugGeometry(bool state);

		void setDebugGeometryMultiplier(float m) { mDebugGeometryMult = m; }

		uint32_t getUniqueID() const { return mID; }

	protected:
		BaseDeformer(const Type type, const std::string& name);

		virtual bool deformImpl(PointsList& points, pxr::UsdTimeCode time_code) = 0;
		virtual bool deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) = 0;

		virtual size_t getDeformedPointsCount() const = 0; 
		virtual bool outputDeformedPoints(const PointsList* pPointsList, pxr::UsdTimeCode time_code) = 0;
		virtual bool outputVelocites(const PointsList* pVelocitiesList, pxr::UsdTimeCode time_code) = 0;

		virtual bool validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) = 0;
		virtual void invalidateData(DeformerDataCache& cache) = 0;

		virtual const UsdPrimHandle& getOutputPrimHandle() const = 0;

		void makeDirty();
		void clearLRUCaches();

		bool isDirty() const { return mDirty; }

	protected:
		bool  		mDirty = true;
		bool  		mDeformerDataWritten = false;

		BS::thread_pool<BS::tp::none> mPool;
		
		Type 				mType;
		std::string 		mName;
		uint32_t 			mID;
		pxr::UsdTimeCode 	mRestTimeCode;

		bool  mUsePointsCache = true;
		bool  mShowDebugGeometry = false;
		float mDebugGeometryMult = 1.0f;
		bool  mInstancingEnabled = true;

		std::string mUniqueName;
		uint8_t mDeformerSubdivLevel = 0;
		
		UsdPrimHandle 	mDeformerGeoPrimHandle;

		std::string 	mSkinPrimAttrName = kСurvesSkinPrimAttrName;		
		std::string   	mVelocityAttrName = kVelocitiAttrName;
		
		MeshContainer::UniquePtr   		mpDeformerMeshContainer;

		DeformerStats   mStats;

		std::mutex      mPrmMutex;

	protected:
		virtual bool writeJsonDataToPrimImpl() const = 0;
		virtual void drawDebugSubdivDeformerGeometry(pxr::UsdTimeCode time_code);

		bool canProduceOutputTimeSamples(pxr::UsdTimeCode time_from, pxr::UsdTimeCode time_to) const {
			return mDeformerGeoPrimHandle.hasPositionsTimeSamples(time_from, time_to);
		}

		const std::string& uniqueName() const { return mUniqueName; }
		std::string velocityKeyName() const { return uniqueName() + "_vel"; }

		MotionBlurDirection motionBlurDirection() const { return mMotionBlurDirection; }
		bool calcMotionVectors() const { return mCalcMotionVectors; }

		virtual void drawDebugGeometry(pxr::UsdTimeCode time_code, const PointsList* pDeformedPoints) = 0;

	private:
		bool buildDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false);
		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false) = 0;
		
		static std::atomic_uint32_t current_id;

		bool mCalcMotionVectors = false;
		MotionBlurDirection mMotionBlurDirection = MotionBlurDirection::TRAILING;

		pxr::SdfPath mDataPrimPath;

		bool mReadJsonDeformerData = false;
		bool mWriteJsonDeformerData = false;

		DebugGeo::UniquePtr mpSubdivDebugGeo;

		// we use these containers to store deformed points data when LRU cache is disabled
		std::unique_ptr<PointsList> 	mpDeformedPointsList;
		std::unique_ptr<PointsList> 	mpDeformedPointsListStep;
		std::unique_ptr<PointsList> 	mpTempVelocitiesList;

		friend class UsdPrimHandle;
};

} // namespace Piston

std::string to_string(const Piston::BaseDeformer::Type& mt);

#endif // PISTON_LIB_BASE_DEFORMER_H_