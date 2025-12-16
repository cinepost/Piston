#ifndef PISTON_LIB_WRAP_CURVES_DEFORMER_DATA_H_
#define PISTON_LIB_WRAP_CURVES_DEFORMER_DATA_H_

#include "serializable_data.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

#include <glm/vec3.hpp> // glm::vec3


namespace Piston {

class WrapCurvesDeformer;

class WrapCurvesDeformerData : public SerializableDeformerDataBase {
	public:
		enum class BindMode: uint8_t { 
			SPACE, 
			DIST 
		};

		struct PointBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			static constexpr float kFltMax = std::numeric_limits<float>::max(); 
			uint32_t face_id;
			float u, v, dist;
			PointBindData(): face_id(kInvalidFaceID), dist(kFltMax) {};

			inline bool isValid() const { return face_id != kInvalidFaceID; }
		};

		const std::vector<PointBindData>& 		getPointBinds() const { return mPointBinds; }
		const BindMode&        					getBindMode() const { return mBindMode; }
		void  setBindMode(const BindMode& mode);

		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		size_t calcHash() const;

		BindMode                                mBindMode = BindMode::SPACE;
		
		std::vector<PointBindData>              mPointBinds;

		friend class WrapCurvesDeformer;
};

void to_json(json& j, const WrapCurvesDeformerData::PointBindData& bind);
void from_json(const json& j, WrapCurvesDeformerData::PointBindData& bind);

inline std::string to_string(const WrapCurvesDeformerData::BindMode& mode) {
	std::string str;
	switch(mode) {
		case WrapCurvesDeformerData::BindMode::SPACE:
			return "SPACE";
		default:
			return "DIST";
	}
}

inline WrapCurvesDeformerData::BindMode from_string(const std::string& str) {
	if(str == "SPACE") return WrapCurvesDeformerData::BindMode::SPACE;

	return WrapCurvesDeformerData::BindMode::DIST;	
}

} // namespace Piston

#endif // PISTON_LIB_WRAP_CURVES_DEFORMER_DATA_H_