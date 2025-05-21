#ifndef PISTON_LIB_FAST_HAIR_DEFORMER_H_
#define PISTON_LIB_FAST_HAIR_DEFORMER_H_

#include "framework.h"
#include "base_hair_deformer.h"
#include "adjacency.h"
#include "phantom_trimesh.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>

#include <glm/vec3.hpp> // glm::vec3

namespace Piston {

class FastHairDeformer : public BaseHairDeformer, public inherit_shared_from_this<BaseHairDeformer, FastHairDeformer> {
	public:
		using SharedPtr = std::shared_ptr<FastHairDeformer>;
		
	public:
		static SharedPtr create();

		virtual const std::string& toString() const override;

	protected:
		FastHairDeformer();
		virtual bool deformImpl(pxr::UsdTimeCode time_code);

	protected:
		pxr::VtArray<pxr::GfVec3f> 	mMeshRestPositions;

	private:
		virtual bool buildDeformerData() override;
		bool buildHairToMeshBindingData();

		UsdGeomMeshFaceAdjacency	mAdjacency;
		std::vector<glm::vec3> 		mRestFaceNormals;
};

} // namespace Piston

#endif // PISTON_LIB_FAST_HAIR_DEFORMER_H_