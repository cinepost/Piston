#ifndef PISTON_LIB_FAST_CURVES_DEFORMER_DATA_H_
#define PISTON_LIB_FAST_CURVES_DEFORMER_DATA_H_

#include "serializable_data.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <glm/vec3.hpp> // glm::vec3


namespace Piston {

class FastCurvesDeformerData : public SerializableDeformerDataBase {
	public:
		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

	protected:
		virtual bool dumpToJSON(json& j) const = 0;
		virtual bool readFromJSON(const json& j) = 0;

		virtual void clearData() override;

	private:
		std::vector<CurveBindData>              			mCurveBinds;

		std::vector<pxr::GfVec3f> 							mRestVertexNormals;
		std::vector<pxr::GfVec3f> 							mLiveVertexNormals;

		std::vector<pxr::GfVec3f>               			mPerBindRestNormals;
		std::vector<pxr::GfVec3f>               			mPerBindLiveNormals; // we keep memeory to save on per-frame reallocations

		std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>   mPerBindRestTBs; // per curve-bind binormal and bangent vector pairs
		std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>   mPerBindLiveTBs; // we keep memeory to save on per-frame reallocations
};

} // namespace Piston

#endif // PISTON_LIB_FAST_CURVES_DEFORMER_DATA_H_