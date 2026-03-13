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

class FastCurvesDeformer;

class FastCurvesDeformerData : public SerializableDeformerDataBase {
	public:
		struct CurveBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			uint32_t face_id;
			float u, v;
			CurveBindData(): face_id(kInvalidFaceID), u(.0f), v(.0f) {};
		};

		const std::vector<CurveBindData>& 							getCurveBinds() const { return mCurveBinds; }
		const std::vector<pxr::GfVec3f>&        					getRestVertexNormals() const { return mRestVertexNormals; }
		const std::vector<pxr::GfVec3f>&        					getPerBindRestNormals() const { return mPerBindRestNormals; }
		const std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>& 	getPerBindRestTBs()	const { return mPerBindRestTBs; }

		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		size_t calcHash() const;

		std::vector<CurveBindData>              			mCurveBinds;
		std::vector<pxr::GfVec3f> 							mRestVertexNormals;
		std::vector<pxr::GfVec3f>               			mPerBindRestNormals;
		std::vector<std::pair<pxr::GfVec3f,pxr::GfVec3f>>   mPerBindRestTBs; // per curve-bind binormal and bangent vector pairs

		friend class FastCurvesDeformer;
};

inline void to_json(json& j, const FastCurvesDeformerData::CurveBindData& bind) {
	j = {bind.face_id, bind.u, bind.v};
}

inline void from_json(const json& j, FastCurvesDeformerData::CurveBindData& bind) {
	bind.face_id = j.at(0).template get<uint32_t>();
	bind.u = j.at(1).template get<float>();
	bind.v = j.at(2).template get<float>();
}

} // namespace Piston

#endif // PISTON_LIB_FAST_CURVES_DEFORMER_DATA_H_