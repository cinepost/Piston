#ifndef PISTON_LIB_WRAP_CURVES_DEFORMER_DATA_H_
#define PISTON_LIB_WRAP_CURVES_DEFORMER_DATA_H_

#include "serializable_data.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/curves.h>

namespace Piston {

class WrapCurvesDeformer;

class WrapCurvesDeformerData : public SerializableDeformerDataBase {
	public:
		enum class BindMode: uint8_t { 
			SPACE, 
			DIST 
		};

		struct OsdPointBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			uint32_t face_id = kInvalidFaceID;
			pxr::GfVec2f uv = {0.0f, 0.0f};
			pxr::GfVec3f localPos;

			bool isValid() const { return face_id != kInvalidFaceID; }

			OsdPointBindData() = default;
		};

		struct PointBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			static constexpr float kFltMax = std::numeric_limits<float>::max(); 
			uint32_t face_id = kInvalidFaceID;
			float u = 0.0f, v = 0.0f;
			float dist = kFltMax;

			PointBindData() = default;
			bool isValid() const { return face_id != kInvalidFaceID; }
		};

		const std::vector<PointBindData>& 		getPointBinds() const { return mPointBinds; }
		const std::vector<OsdPointBindData>& 	getOsdPointBinds() const { return mOsdPointBinds; }

		BindMode        					getBindMode() const { return mBindMode; }
		void  								setBindMode(const BindMode& mode);

		virtual bool isValid() const override { const std::lock_guard<std::mutex> lock(mMutex); return mIsValid; };
		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

		WrapCurvesDeformerData();

	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		size_t 	calcHash() const;
		void 	setValid(bool state) { const std::lock_guard<std::mutex> lock(mMutex); mIsValid = state; }

		BindMode                                mBindMode = BindMode::DIST;
		
		std::vector<PointBindData>              mPointBinds;
		std::vector<OsdPointBindData>           mOsdPointBinds;

		bool 									mIsValid;

		friend class WrapCurvesDeformer;
};

void to_json(json& j, const WrapCurvesDeformerData::PointBindData& bind);
void from_json(const json& j, WrapCurvesDeformerData::PointBindData& bind);

void to_json(json& j, const WrapCurvesDeformerData::OsdPointBindData& bind);
void from_json(const json& j, WrapCurvesDeformerData::OsdPointBindData& bind);

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