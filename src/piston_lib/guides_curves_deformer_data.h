#ifndef PISTON_LIB_GUIDES_CURVES_DEFORMER_DATA_H_
#define PISTON_LIB_GUIDES_CURVES_DEFORMER_DATA_H_

#include "serializable_data.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <glm/vec3.hpp> // glm::vec3


namespace Piston {

class GuidesCurvesDeformer;

class GuidesCurvesDeformerData : public SerializableDeformerDataBase {
	public:
		struct PointBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			uint32_t face_id;
			float u, v;
			PointBindData(): face_id(kInvalidFaceID) {};
		};

		const std::vector<PointBindData>& 							getPointBinds() const { return mPointBinds; }

		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		size_t calcHash() const;

		std::vector<PointBindData>              			mPointBinds;

		friend class GuidesCurvesDeformer;
};

void to_json(json& j, const GuidesCurvesDeformerData::PointBindData& bind);
void from_json(const json& j, GuidesCurvesDeformerData::PointBindData& bind);

} // namespace Piston

#endif // PISTON_LIB_GUIDES_CURVES_DEFORMER_DATA_H_