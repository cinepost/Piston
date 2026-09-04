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
		static const uint8_t kNeighborsDefault = 4; // 3 or 4. Used in simple Multi-Point Proxinity mode
		enum class BindMode: uint8_t { 
			SIMPLE, 
			MPPP
		};

		struct PointBindData {
			static constexpr uint32_t kInvalidFaceID = std::numeric_limits<uint32_t>::max();
			static constexpr float kFltMax = std::numeric_limits<float>::max(); 
			pxr::GfVec3f local_pos;
			uint32_t face_id;
			uint8_t  edge_id;
			PointBindData(): face_id(kInvalidFaceID) {};

			inline bool isValid() const { return face_id != kInvalidFaceID; }
		};

		// Multi-Point Proximity
		struct BindSample {
    		uint32_t		meshVertexIndex;
    		float 			weight;
    		pxr::GfVec3f 	localOffset; // local-space offset
		};

		struct MPPPointBindings {
			std::vector<BindSample> 	mSamples; 		// Instancer points count * mStride
			const size_t 				mStride; 		// Constant (e.g., 3 or 4)
			size_t                      mSize;

			MPPPointBindings(size_t stride): mStride(std::min((size_t)1, stride)), mSize(0) {
				assert(mStride > 0 && mStride < 9);
			}

			MPPPointBindings(): MPPPointBindings(kNeighborsDefault) {}

			size_t size() const { return mSize; }

			void clear() { 
				mSamples.clear();
				mSize = 0;
			}

			void resize(size_t new_size) {
				assert(new_size > 0);

				if(mSize == new_size) return;
				mSize = new_size;
				mSamples.resize(mSize * mStride);
			}

			size_t getStride() const { return mStride; }
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
		MPPPointBindings                        mMPPPointBindings;

		bool 									mIsValid;

		friend class PointInstancerDeformer;
};

void to_json(json& j, const PointInstancerDeformerData::PointBindData& bind);
void from_json(const json& j, PointInstancerDeformerData::PointBindData& bind);

inline std::string to_string(const PointInstancerDeformerData::BindMode& mode) {
	std::string str;
	switch(mode) {
		case PointInstancerDeformerData::BindMode::SIMPLE:
			return "SIMPLE";
		default:
			return "MPPP";
	}
}

inline void from_string(const std::string& str, PointInstancerDeformerData::BindMode& mode) {
	if(str == "SIMPLE") {
		mode = PointInstancerDeformerData::BindMode::SIMPLE;
	} else {
		mode = PointInstancerDeformerData::BindMode::MPPP;	
	}
}

} // namespace Piston

#endif // PISTON_LIB_POINT_INSTANCER_DEFORMER_DATA_H_