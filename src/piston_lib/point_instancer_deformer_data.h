#ifndef PISTON_LIB_POINT_INSTANCER_DEFORMER_DATA_H_
#define PISTON_LIB_POINT_INSTANCER_DEFORMER_DATA_H_

#include "serializable_data.h"

#include <memory>
#include <limits>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/pointInstancer.h>

namespace Piston {

class PointInstancerDeformer;

class PointInstancerDeformerData : public SerializableDeformerDataBase {
	public:
		enum class BindMode: uint8_t { 
			SIMPLE, 
			VOLUME
		};

		struct PointBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			static constexpr float kFltMax = std::numeric_limits<float>::max(); 
			uint32_t face_id;
			float u, v, dist;
			PointBindData(): face_id(kInvalidFaceID), dist(kFltMax) {};

			inline bool isValid() const { return face_id != kInvalidFaceID; }
		};

		void setBindMode(BindMode mode);
		BindMode getBindMode() const { return mBindMode; }

		const std::vector<PointBindData>& 	getPointBinds() const { return mPointBinds; }

		virtual bool isValid() const override { const std::lock_guard<std::mutex> lock(mMutex); return mIsValid; };
		virtual const std::string& typeName() const override;
		virtual const std::string& jsonDataKey() const override;
		virtual const DataVersion& jsonDataVersion() const override;

		PointInstancerDeformerData();

	protected:
		virtual bool dumpToJSON(json& j) const override;
		virtual bool readFromJSON(const json& j) override;

		virtual void clearData() override;

	private:
		size_t 	calcHash() const;
		void 	setValid(bool state) { const std::lock_guard<std::mutex> lock(mMutex); mIsValid = state; }

		BindMode                                mBindMode;

		std::vector<PointBindData>              mPointBinds;
		bool 									mIsValid;

		friend class PointInstancerDeformer;
};

void to_json(json& j, const PointInstancerDeformerData::PointBindData& bind);
void from_json(const json& j, PointInstancerDeformerData::PointBindData& bind);

} // namespace Piston

#endif // PISTON_LIB_POINT_INSTANCER_DEFORMER_DATA_H_