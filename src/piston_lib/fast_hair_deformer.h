#ifndef PISTON_LIB_FAST_HAIR_DEFORMER_H_
#define PISTON_LIB_FAST_HAIR_DEFORMER_H_

#include "framework.h"
#include "base_hair_deformer.h"
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

class FastHairDeformer : public BaseHairDeformer, public inherit_shared_from_this<BaseHairDeformer, FastHairDeformer> {
	public:
		using SharedPtr = std::shared_ptr<FastHairDeformer>;
		using PxrIndexType = int;

	private:
		struct CurveBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			uint32_t face_id;
			float u, v, dist;
			glm::vec3 offset;

			CurveBindData(): face_id(kInvalidFaceID) {};
		};

	public:
		static SharedPtr create();

		virtual const std::string& toString() const override;

	protected:
		FastHairDeformer();
		virtual bool deformImpl(pxr::UsdTimeCode time_code);

	private:
		virtual bool buildDeformerData(pxr::UsdTimeCode reference_time_code) override;
		bool buildCurvesBindingData(pxr::UsdTimeCode reference_time_code);

		size_t                                  mCurvesCount;
		UsdGeomMeshFaceAdjacency				mAdjacency;
		PhantomTrimesh<PxrIndexType>::SharedPtr mpPhantomTrimesh;
		pxr::VtArray<pxr::GfVec3f>              mCurveRefPoints;
		std::vector<CurveBindData>              mCurveBinds;
		pxr::VtArray<int> 						mCurveVertexCounts;
		std::vector<uint32_t> 					mCurveOffsets;
};

} // namespace Piston

#endif // PISTON_LIB_FAST_HAIR_DEFORMER_H_