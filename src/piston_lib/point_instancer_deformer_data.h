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
		static const uint8_t kNeighborsDefault = 4; // 3 or 4. Used in simple Multi-Point Proximity mode
		enum class BindMode: uint8_t { 
			SIMPLE, 
			MPPP
		};

		struct PointBindData {
			static constexpr uint32_t kInvalidPointID = std::numeric_limits<uint32_t>::max();
			static constexpr float kFltMax = std::numeric_limits<float>::max(); 

			enum class Flags : uint8_t {
			    NONE    = 0,
			    OUTSIDE = 1 << 0
			};

			uint32_t 	point_indices[4]{ kInvalidPointID, kInvalidPointID, kInvalidPointID, kInvalidPointID };
			int8_t  	edge_id = -1;
			Flags 		flags = Flags::NONE;

			pxr::GfVec3f localPos;
			pxr::GfVec3f restNormal;
			pxr::GfVec3f restTangent;
			pxr::GfVec3f restBinormal;
			float u = 0.0, v = 0.0; // used to reconstruct quad surface normal

			PointBindData() = default; 

			inline bool isValid() const { return point_indices[0] != kInvalidPointID; }
			inline bool isQuadBound() const { return point_indices[3] != kInvalidPointID; }
			inline bool isOutside() const { return (static_cast<uint8_t>(flags) & static_cast<uint8_t>(Flags::OUTSIDE)) != static_cast<uint8_t>(Flags::NONE); }
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

inline PointInstancerDeformerData::PointBindData::Flags operator|(PointInstancerDeformerData::PointBindData::Flags lhs, PointInstancerDeformerData::PointBindData::Flags rhs) {
    return static_cast<PointInstancerDeformerData::PointBindData::Flags>(static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

inline PointInstancerDeformerData::PointBindData::Flags operator&(PointInstancerDeformerData::PointBindData::Flags lhs, PointInstancerDeformerData::PointBindData::Flags rhs) {
    return static_cast<PointInstancerDeformerData::PointBindData::Flags>(static_cast<uint8_t>(lhs) & static_cast<uint8_t>(rhs));
}

inline PointInstancerDeformerData::PointBindData::Flags& operator|=(PointInstancerDeformerData::PointBindData::Flags& lhs, PointInstancerDeformerData::PointBindData::Flags rhs) {
    lhs = lhs | rhs;
    return lhs;
}

inline PointInstancerDeformerData::PointBindData::Flags& operator&=(PointInstancerDeformerData::PointBindData::Flags& lhs, PointInstancerDeformerData::PointBindData::Flags rhs) {
    lhs = lhs & rhs;
    return lhs;
}

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