#ifndef PISTON_LIB_BASE_MESH_CURVES_DEFORMER_H_
#define PISTON_LIB_BASE_MESH_CURVES_DEFORMER_H_

#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"


namespace Piston {

namespace {
	const std::string kMeshRestPositionAttrName = "rest_p";
	const std::string kСurvesSkinPrimAttrName = "skinprim";
}

class BaseMeshCurvesDeformer : public BaseCurvesDeformer {
	public:
		using SharedPtr = std::shared_ptr<BaseMeshCurvesDeformer>;
	
	public:
		virtual ~BaseMeshCurvesDeformer() {}

		void setMeshRestPositionAttrName(const std::string& name);
		const std::string& getMeshRestPositionAttrName() const { return mMeshRestPositionAttrName; }

		void setСurvesSkinPrimAttrName(const std::string& name);
		const std::string& getСurvesSkinPrimAttrName() const { return mСurvesSkinPrimAttrName; }

		virtual const std::string& toString() const override; 

	protected:
		BaseMeshCurvesDeformer(const BaseCurvesDeformer::Type type, const std::string& name);

		virtual bool validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim);
		
	protected:
		SerializableUsdGeomMeshFaceAdjacency::UniquePtr 	mpAdjacencyData;
		SerializablePhantomTrimesh::UniquePtr				mpPhantomTrimeshData;

		std::string   mMeshRestPositionAttrName = kMeshRestPositionAttrName;
		std::string   mСurvesSkinPrimAttrName = kСurvesSkinPrimAttrName;
		
	protected:
		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode reference_time_code, bool multi_threaded = false);
		virtual bool writeJsonDataToPrimImpl() const;
};

} // namespace Piston

#endif // PISTON_LIB_BASE_MESH_CURVES_DEFORMER_H_