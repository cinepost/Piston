#ifndef PISTON_LIB_FAST_CURVES_DEFORMER_H_
#define PISTON_LIB_FAST_CURVES_DEFORMER_H_

#include "framework.h"
#include "base_curves_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"
#include "curves_container.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <glm/vec3.hpp> // glm::vec3


namespace Piston {

class FastCurvesDeformer : public BaseCurvesDeformer, public inherit_shared_from_this<BaseCurvesDeformer, FastCurvesDeformer> {
	public:
		using SharedPtr = std::shared_ptr<FastCurvesDeformer>;
		using PxrIndexType = int;

		enum class DeformMode {
			FACET 	 = 0,
			SMOOTH 	 = 1,
			ACCURATE = 2
		};

	private:
		struct CurveBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			uint32_t face_id;
			float u, v, dist;

			CurveBindData(): face_id(kInvalidFaceID) {};
		};

	public:
		~FastCurvesDeformer();

		static SharedPtr create();

		void setDeformMode(DeformMode mode);
		DeformMode getDeformMode() const { return mDeformMode; }

		virtual const std::string& toString() const override;

	protected:
		FastCurvesDeformer();
		virtual bool deformImpl(pxr::UsdTimeCode time_code);

	private:
		virtual bool buildDeformerData(pxr::UsdTimeCode rest_time_code) override;
		bool buildCurvesBindingData(pxr::UsdTimeCode rest_time_code);

		bool calcPerBindNormals(bool build_live);
		bool calcPerBindTangentsAndBiNormals(bool build_live);

		void transformCurvesToNTB();

		DeformMode mDeformMode = DeformMode::FACET;

		UsdGeomMeshFaceAdjacency::SharedPtr					mpAdjacency;
		PhantomTrimesh<PxrIndexType>::SharedPtr 			mpPhantomTrimesh;
		std::vector<CurveBindData>              			mCurveBinds;

		std::vector<pxr::GfVec3f>               			mPerBindRestNormals;
		std::vector<pxr::GfVec3f>               			mPerBindLiveNormals; // we keep memeory to save on per-frame reallocations

		std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>   mPerBindRestTBs; // per curve-bind binormal and bangent vector pairs
		std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>   mPerBindLiveTBs; // we keep memeory to save on per-frame reallocations
};

} // namespace Piston

#endif // PISTON_LIB_FAST_CURVES_DEFORMER_H_