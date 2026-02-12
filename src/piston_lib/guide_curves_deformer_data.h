#ifndef PISTON_LIB_GUIDES_CURVES_DEFORMER_DATA_H_
#define PISTON_LIB_GUIDES_CURVES_DEFORMER_DATA_H_

#include "serializable_data.h"
#include "phantom_trimesh.h"
#include "float24.h"

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
			NTB,
			ANGLE,
			SPACE
		};

		struct PointBindData {
			union EncodedID{
				uint32_t raw_data;

				struct {
					uint32_t element_id : 30; // it's either tetra or triface id
					uint32_t is_tetra : 1;
					uint32_t is_24bit : 1;
				} mode_space;

				struct {
					// TODO: maybe we should just store one point bind index ? Do we still need guide_od/segment_id separation ?
					uint32_t guide_id : 24;
					uint32_t segment_id : 8;
				} mode_angle;

				struct {
					uint32_t frame_id : 32;
				} mode_ntb;

				EncodedID() { raw_data = kInvalid; }
				EncodedID(uint32_t i) { raw_data = i; }

				bool operator==(uint32_t i) const { return raw_data == i; }

				operator float() const { return raw_data; }
			};

			static constexpr uint32_t kInvalid = std::numeric_limits<uint32_t>::max();
			EncodedID 				encoded_id;		// encoding depends on a binding mode. IMPORTANT: msb is reserved as a flag !!!
			std::array<float, 3> 	data; 			
			PointBindData(): encoded_id(kInvalid) {};
			inline size_t hash() const { return static_cast<size_t>(encoded_id.raw_data) + static_cast<size_t>(data[0]) + static_cast<size_t>(data[1]) + static_cast<size_t>(data[2]); }

			inline void setData(const pxr::GfVec3f& v) { data[0] = v[0]; data[1] = v[1]; data[2] = v[2]; }
			inline void setData(float x, float y, float z) { data[0] = x; data[1] = y; data[2] = z; }
			inline void setData(float x, float y, float z, float w) { 
				setData(float24_s(x), float24_s(y), float24_s(z), float24_s(w));
			}

			inline void setData(float24_s x, float24_s y, float24_s z, float24_s w) {
				uint8_t* bytePtr = reinterpret_cast<uint8_t*>(data.data());
				memcpy(bytePtr, x.data.buffer, 3);
				memcpy(bytePtr + 3, y.data.buffer, 3);
				memcpy(bytePtr + 6, z.data.buffer, 3);
				memcpy(bytePtr + 9, w.data.buffer, 3);
			}

			inline const std::array<float, 3>& getData() const { return data; }
			inline void getData(float& x, float& y, float& z) const { x = data[0]; y = data[1]; z = data[2]; }

			inline void getData(float& x, float& y, float& z, float& w) const {
				std::array<float24_s, 4> tmp;
				getData(tmp);
				x = tmp[0]; y = tmp[1]; z = tmp[2]; w = tmp[3];
			}
			inline void getData(std::array<float24_s, 4>& a) const {
				const uint8_t* bytePtr = reinterpret_cast<const uint8_t*>(data.data()); 
				memcpy(a[0].data.buffer, bytePtr, 3);
				memcpy(a[1].data.buffer, bytePtr + 3, 3);
				memcpy(a[2].data.buffer, bytePtr + 6, 3);
				memcpy(a[3].data.buffer, bytePtr + 9, 3);
			}

			inline void getData(pxr::GfVec3f& v) const { v[0] = data[0]; v[1] = data[1]; v[2] = data[2]; }

			inline void encodeID_modeSPACE(uint32_t id, bool is_tetra, bool is_24bit) {
				encoded_id.mode_space.element_id = id;
				encoded_id.mode_space.is_tetra = is_tetra;
				encoded_id.mode_space.is_24bit = is_24bit;
			}
			inline void decodeID_modeSPACE(uint32_t& id, bool& is_tetra, bool& is_24bit) const {
				id = encoded_id.mode_space.element_id;
				is_tetra = encoded_id.mode_space.is_tetra;
				is_24bit = encoded_id.mode_space.is_24bit;
			}

			inline void encodeID_modeANGLE(uint32_t guide_id, uint8_t segment_id) {
				encoded_id.mode_angle.guide_id = guide_id;
				encoded_id.mode_angle.segment_id = (uint32_t)segment_id;
			}
			inline void decodeID_modeANGLE(uint32_t& guide_id, uint8_t& segment_id) const {
				guide_id = encoded_id.mode_angle.guide_id;
				segment_id = (uint8_t)encoded_id.mode_angle.segment_id;
			}

			inline void encodeID_modeNTB(uint32_t frame_id) {
				encoded_id.mode_ntb.frame_id = frame_id;
			}
			inline void decodeID_modeNTB(uint32_t& frame_id) const {
				frame_id = encoded_id.mode_ntb.frame_id;
			}
		};

		// Struct used to determine guide root ntb frame calculation
		struct GuideOrigin{
			static const uint32_t kInvalid = 0xFFFFFFFF;
			static const uint32_t kInvalidFaceID = 0x3FFFFFFF;
			static const uint32_t kInvalidAxisID = 3;

			uint32_t raw_data;

			GuideOrigin(): raw_data(kInvalid) {}
			GuideOrigin(uint32_t face_id, uint32_t axis_id){ encode(face_id, axis_id); }

			void encode(uint32_t face_id, uint32_t axis_id) {
				raw_data = (face_id & kInvalidFaceID) | (axis_id << 30);
			}

			void decode(uint32_t& face_id, uint32_t& axis_id) const {
				face_id = raw_data & kInvalidFaceID;
				axis_id = raw_data >> 30;
			}

			operator uint32_t() const { return raw_data; }
		};

		const std::vector<PointBindData>& 	getPointBinds() const { return mPointBinds; }
		const std::vector<GuideOrigin>& 	getGuideOrigins() const { return mGuideOrigins; }
		const std::vector<int>& 			getSkinPrimIndices() const { return mSkinPrimIndices; }
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
		std::vector<GuideOrigin>& 		guideOrigins() { return mGuideOrigins; }
		std::vector<int>&               skinPrimIndices() { return mSkinPrimIndices; }
		void setSkinPrimPath(const std::string& prim_path);

	private:
		size_t calcHash() const;

		std::vector<PointBindData> 	mPointBinds;
		std::vector<GuideOrigin> 	mGuideOrigins;
		BindMode                    mBindMode = BindMode::NTB;
		std::string                 mSkinPrimPath;
		std::vector<int> 			mSkinPrimIndices;

		friend class GuideCurvesDeformer;
};

//void to_json(json& j, const GuideCurvesDeformerData::PointBindData& bind) {
//	j = {bind.encoded_id.raw_data, bind.data[0], bind.data[1], bind.data[2]};
//}

//void from_json(const json& j, GuideCurvesDeformerData::PointBindData& bind) {
//	bind.encoded_id.raw_data = j.at(0).template get<uint32_t>();
//	bind.data = {j.at(2).template get<float>(), j.at(3).template get<float>(), j.at(4).template get<float>()};
//}

//void to_json(json& j, const GuideCurvesDeformerData::GuideOrigin& o) {
//	j = o.raw_data;
//}

//void from_json(const json& j, GuideCurvesDeformerData::GuideOrigin& o) {
//	o.raw_data = j.at(0).template get<uint32_t>();
//}

inline std::string to_string(const GuideCurvesDeformerData::BindMode& mode) {
	std::string str;
	switch(mode) {
		case GuideCurvesDeformerData::BindMode::NTB:
			return "NTB";
		case GuideCurvesDeformerData::BindMode::ANGLE:
			return "ANGLE";
		default:
			return "SPACE";
	}
}

} // namespace Piston

#endif // PISTON_LIB_GUIDES_CURVES_DEFORMER_DATA_H_