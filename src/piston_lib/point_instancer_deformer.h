#ifndef PISTON_LIB_POINT_INSTANCER_DEFORMER_H_
#define PISTON_LIB_POINT_INSTANCER_DEFORMER_H_

#include "framework.h"
#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "geometry_tools.h"
#include "instancer_container.h"
#include "point_instancer_deformer_data.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/pointInstancer.h>


namespace Piston {

class PointInstancerDeformer :public BaseDeformer, public inherit_shared_from_this<BaseDeformer, PointInstancerDeformer> {
	public:
		using SharedPtr = std::shared_ptr<PointInstancerDeformer>;

		using BindMode = PointInstancerDeformerData::BindMode;
		using PointBindData = PointInstancerDeformerData::PointBindData;

	public:
		~PointInstancerDeformer();

		static SharedPtr create(const std::string& name);
		virtual const std::string& toString() const override;

		virtual bool deform(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default(), bool multi_threaded = true, bool ignoreVelocities = false) override;
		virtual bool deform_dbg(pxr::UsdTimeCode time_code = pxr::UsdTimeCode::Default(), bool ignoreVelocities = false) override;

	protected:
		PointInstancerDeformer(const std::string& name);

		virtual void invalidateData(DeformerDataCache& cache) override;

		virtual bool validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim);
		virtual const UsdPrimHandle& getOutputPrimHandle() const override { return mInstancerGeoPrimHandle; }

	private:
		virtual bool buildDeformerData(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false);

		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false);
		bool buildDeformerData_SimpleMode(bool multi_threaded, const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code);
		virtual bool writeJsonDataToPrimImpl() const;

		BindMode                                    mBindMode;

		UsdPrimHandle 								mInstancerGeoPrimHandle;

		std::shared_ptr<SerializableUsdGeomMeshFaceAdjacency> 	mpAdjacencyData;
		std::shared_ptr<SerializablePhantomTrimesh>				mpPhantomTrimeshData;

		InstancerContainer::UniquePtr                           mpInstancerContainer;

		std::shared_ptr<PointInstancerDeformerData> mpPointInstancerDeformerData;

		std::vector<pxr::GfVec3f> 					mLiveVertexNormals;
		std::vector<pxr::GfVec3f> 					mLiveTriFaceNormals;

		std::vector<pxr::GfMatrix3f>            	mTmpFaceNTBMatrices;
		std::vector<pxr::GfVec3f>               	mTmpCurvesLocalAnimVectors;
};

} // namespace Piston

#endif // PISTON_LIB_POINT_INSTANCER_DEFORMER_H_