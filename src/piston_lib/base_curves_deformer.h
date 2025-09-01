#ifndef PISTON_LIB_BASE_CURVES_DEFORMER_H_
#define PISTON_LIB_BASE_CURVES_DEFORMER_H_

#include "framework.h"
#include "common.h"
#include "curves_container.h"
#include "deformer_stats.h"
#include "serializable_data.h"
#include "adjacency.h"
#include "phantom_trimesh.h"

#include "BS_thread_pool.hpp" // BS::multi_future, BS::thread_pool

#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>

#include <memory>
#include <string>


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
			WRAP, 
			UNKNOWN 
		};
		
	public:
		virtual ~BaseCurvesDeformer() {}

		void setMeshGeoPrim(pxr::UsdPrim* pGeoPrim);
		void setCurvesGeoPrim(pxr::UsdPrim* pGeoPrim);

		void readJsonDeformDataFromPrim(bool state);
		void writeJsonDeformDataToPrim(bool state);

		void setMeshRestPositionAttrName(const std::string& name);
		const std::string& getMeshRestPositionAttrName() const { return mMeshRestPositionAttrName; }

		void setСurvesSkinPrimAttrName(const std::string& name);
		const std::string& getСurvesSkinPrimAttrName() const { return mСurvesSkinPrimAttrName; }

		bool deform(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());
		bool deform_mp(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default());

		virtual const std::string& toString() const;

		const DeformerStats& getStats() const { return mStats; }

	protected:
		BaseCurvesDeformer();

		virtual bool deformImpl(pxr::UsdTimeCode time_code) = 0;
		void makeDirty();
		
	protected:
		bool mDirty = true;
		
		UsdPrimHandle mMeshGeoPrimHandle;
		UsdPrimHandle mCurvesGeoPrimHandle;

		SerializableUsdGeomMeshFaceAdjacency::UniquePtr 	mpAdjacencyData;
		SerializablePhantomTrimesh::UniquePtr				mpPhantomTrimeshData;

		std::string   mMeshRestPositionAttrName = kMeshRestPositionAttrName;
		std::string   mСurvesSkinPrimAttrName = kСurvesSkinPrimAttrName;
		
		PxrCurvesContainer::UniquePtr mpCurvesContainer;

		BS::thread_pool<BS::tp::none> mPool;
		DeformerStats mStats;
	private:
		virtual bool buildDeformerData(pxr::UsdTimeCode reference_time_code) = 0;

		pxr::UsdStageRefPtr mpTempStage;

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
            should_not_get_here();
            return "UNKNOWN";
    }
#undef t2s
}

#endif // PISTON_LIB_BASE_CURVES_DEFORMER_H_