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

		// DocString: setInstancerGeoPrim
		/**
		 * @brief Sets the Pixar USD curves primitive that will undergo deformation.
		 * @param prim The USD curves primitive to be deformed.
		 */		
		void setInstancerGeoPrim(const pxr::UsdPrim& prim);
		const pxr::UsdPrim& getInstancerGeoPrim() const;

		void setInstancerRestAttrName(const std::string& name);
		const std::string& getInstancerRestAttrName() const { return mInstancerGeoPrimHandle.getRestAttrName(); }


		static SharedPtr create(const std::string& name);
		virtual const std::string& toString() const override final;

	protected:
		PointInstancerDeformer(const std::string& name);

		virtual bool deformImpl(PointsList& points, pxr::UsdTimeCode time_code) override final {
			PROFILE("PointInstancerDeformer::deformImpl");
			return __deform__(points, false, time_code);
		}

		virtual bool deformMtImpl(PointsList& points, pxr::UsdTimeCode time_code) override final {
			PROFILE("PointInstancerDeformer::deformMtImpl");
			return __deform__(points, true, time_code);
		}

		virtual void invalidateData(DeformerDataCache& cache) override final;

		virtual size_t getDeformedPointsCount() const override final; 
		virtual bool outputDeformedPoints(const PointsList* pPointsList, pxr::UsdTimeCode time_code) override final; 
		virtual bool outputVelocites(const PointsList* pVelocitiesList, pxr::UsdTimeCode time_code) override final; 

		virtual bool validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim) override final;
		virtual const UsdPrimHandle& getOutputPrimHandle() const override final { return mInstancerGeoPrimHandle; }

		virtual void drawDebugGeometry(pxr::UsdTimeCode time_code, const PointsList* pDeformedPoints) override final;

	private:
		bool __deform__(PointsList& points, bool multi_threaded, pxr::UsdTimeCode time_code);

		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode rest_time_code, bool multi_threaded = false) override final;
		bool buildDeformerData_SimpleMode(bool multi_threaded, const std::vector<pxr::GfVec3f>& rest_vertex_normals, pxr::UsdTimeCode rest_time_code);
		virtual bool writeJsonDataToPrimImpl() const override final;

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