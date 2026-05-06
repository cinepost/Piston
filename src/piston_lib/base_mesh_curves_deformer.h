#ifndef PISTON_LIB_BASE_MESH_CURVES_DEFORMER_H_
#define PISTON_LIB_BASE_MESH_CURVES_DEFORMER_H_

#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"

#include <memory>


namespace Piston {

namespace {
	const std::string kMeshRestPositionAttrName = "rest_p";
}

class BaseMeshCurvesDeformer : public BaseCurvesDeformer {
	public:
		using SharedPtr = std::shared_ptr<BaseMeshCurvesDeformer>;
	
	public:
		virtual ~BaseMeshCurvesDeformer() {}
		virtual const std::string& toString() const override; 

	protected:
		BaseMeshCurvesDeformer(const BaseCurvesDeformer::Type type, const std::string& name);

		virtual bool validateDeformerGeoPrim(const pxr::UsdPrim& geoPrim);
		
	protected:
		std::shared_ptr<SerializableUsdGeomMeshFaceAdjacency> 	mpAdjacencyData;
		std::shared_ptr<SerializablePhantomTrimesh>				mpPhantomTrimeshData;
		
	protected:
		virtual bool buildDeformerDataImpl(pxr::UsdTimeCode reference_time_code, bool multi_threaded = false);
		virtual bool writeJsonDataToPrimImpl() const;

		virtual void invalidateData(DeformerDataCache& cache) override;
};

} // namespace Piston

#endif // PISTON_LIB_BASE_MESH_CURVES_DEFORMER_H_