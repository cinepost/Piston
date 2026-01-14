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

class GuideCurvesDeformer;

class GuideCurvesDeformerData : public SerializableDeformerDataBase {
	public:
		enum class BindMode: uint8_t { 
			SPACE, 
			DIST 
		};

		struct PointBindData {
			static constexpr uint32_t kInvalid = std::numeric_limits<uint16_t>::max();
			uint16_t guide_id, vtx;
			pxr::GfVec3f vec;
			PointBindData(): guide_id(kInvalid), vtx(kInvalid) {};

			inline size_t hash() const { return static_cast<size_t>(guide_id + vtx) + static_cast<size_t>(vec[0] + vec[1] + vec[2]); }
		};

		const std::vector<PointBindData>& 	getPointBinds() const { return mPointBinds; }
		BindMode        					getBindMode() const { return mBindMode; }
		void  								setBindMode(const BindMode& mode);

		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

		std::vector<PointBindData>& 	pointBinds() { return mPointBinds; }

	private:
		size_t calcHash() const;

		std::vector<PointBindData> 	mPointBinds;
		BindMode                    mBindMode = BindMode::DIST;

		friend class GuideCurvesDeformer;
};

void to_json(json& j, const GuideCurvesDeformerData::PointBindData& bind);
void from_json(const json& j, GuideCurvesDeformerData::PointBindData& bind);

inline std::string to_string(const GuideCurvesDeformerData::BindMode& mode) {
	std::string str;
	switch(mode) {
		case GuideCurvesDeformerData::BindMode::SPACE:
			return "SPACE";
		default:
			return "DIST";
	}
}

} // namespace Piston

#endif // PISTON_LIB_GUIDES_CURVES_DEFORMER_DATA_H_