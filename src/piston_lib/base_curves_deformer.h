#ifndef PISTON_LIB_BASE_CURVES_DEFORMER_H_
#define PISTON_LIB_BASE_CURVES_DEFORMER_H_

#include "framework.h"
#include "common.h"
#include "curves_container.h"
#include "deformer_stats.h"
#include "serializable_data.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "simple_profiler.h"

#include "BS_thread_pool.hpp" // BS::multi_future, BS::thread_pool

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>

#include <atomic>
#include <memory>
#include <string>


namespace Piston {

namespace {
	const std::string kMeshRestPositionAttrName = "rest_p";
	const std::string kСurvesSkinPrimAttrName = "skinprim";
	const std::string kVelocitiAttrName = "velocities";
}

class BaseCurvesDeformer : public std::enable_shared_from_this<BaseCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<BaseCurvesDeformer>;

		enum class Type { 
			FAST, 
			WRAP, 
			UNKNOWN 
		};

		enum class MotionBlurDirection {
			TRAILING,
			CENTERED,
			LEADING
		};
		
	public:
		virtual ~BaseCurvesDeformer() {}

		void setMeshGeoPrim(const pxr::UsdPrim& pGeoPrim);
		void setCurvesGeoPrim(const pxr::UsdPrim& pGeoPrim);

		void setReadJsonDataFromPrim(bool state);
		bool getReadJsonDataState() const { return mReadJsonDeformerData; }
		bool writeJsonDataToPrim(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		void setMeshRestPositionAttrName(const std::string& name);
		const std::string& getMeshRestPositionAttrName() const { return mMeshRestPositionAttrName; }

		void setСurvesSkinPrimAttrName(const std::string& name);
		const std::string& getСurvesSkinPrimAttrName() const { return mСurvesSkinPrimAttrName; }

		void setVelocityAttrName(const std::string& name);
		const std::string& getVelocityAttrName() const { return mVelocityAttrName; }

		bool deform(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default(), bool multi_threaded = true);
		bool deform_dbg(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		virtual const std::string& toString() const;

		const DeformerStats& getStats() const { return mStats; }

		void setMotionBlurState(bool state);

		bool getMotionBlurState() const { return mCalcMotionVectors; }

	protected:
		BaseCurvesDeformer(const Type type, const std::string& name);

		virtual bool deformImpl(PointsList& points, pxr::UsdTimeCode time_code) = 0;
		virtual bool deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) = 0;
		void makeDirty();
		
	protected:
		bool mDirty = true;
		
		UsdPrimHandle mMeshGeoPrimHandle;
		UsdPrimHandle mCurvesGeoPrimHandle;

		SerializableUsdGeomMeshFaceAdjacency::UniquePtr 	mpAdjacencyData;
		SerializablePhantomTrimesh::UniquePtr				mpPhantomTrimeshData;

		std::string   mMeshRestPositionAttrName = kMeshRestPositionAttrName;
		std::string   mСurvesSkinPrimAttrName = kСurvesSkinPrimAttrName;
		std::string   mVelocityAttrName = kVelocitiAttrName;
		
		PxrCurvesContainer::UniquePtr mpCurvesContainer;

		BS::thread_pool<BS::tp::none> mPool;
		DeformerStats mStats;

	private:
		bool buildDeformerData(pxr::UsdTimeCode reference_time_code);
		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode reference_time_code) = 0;
		virtual bool writeJsonDataToPrimImpl() const = 0;

		const std::string& uniqueName() const { return mUniqueName; }
		std::string velocityKeyName() const { return uniqueName() + "_vel"; }

		static std::atomic_uint32_t current_id;

		Type mType;
		std::string mName;
		uint32_t mID;
		std::string mUniqueName;

		pxr::UsdStageRefPtr mpTempStage;

		bool mCalcMotionVectors = true;
		MotionBlurDirection mMotionBlurDirection = MotionBlurDirection::TRAILING;

		bool mReadJsonDeformerData = false;
		bool mWriteJsonDeformerData = false;
};

} // namespace Piston

inline std::string to_string(Piston::BaseCurvesDeformer::Type mt) {
#define t2s(t_) case Piston::BaseCurvesDeformer::Type::t_: return #t_;
    switch (mt) {
        t2s(FAST);
        t2s(WRAP);
        default:
            assert(false);
            return "UNKNOWN";
    }
#undef t2s
}

#endif // PISTON_LIB_BASE_CURVES_DEFORMER_H_