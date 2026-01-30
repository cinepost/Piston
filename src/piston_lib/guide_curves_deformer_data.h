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
					uint32_t is_24bit : 1;
					uint32_t is_tetra : 1;
				} mode_space;

				struct {
					uint32_t guide_id : 24;
					uint32_t segment_id : 8;
				} mode_angle;
			};

			static constexpr uint32_t kInvalid = std::numeric_limits<uint32_t>::max();
			EncodedID 				encoded_id;		// encoding depends on a binding mode. IMPORTANT: msb is reserved as a flag !!!
			std::array<float, 3> 	data; 			
			PointBindData(): encoded_id(kInvalid) {};
			inline size_t hash() const { return static_cast<size_t>(encoded_id) + static_cast<size_t>(data[0]) + static_cast<size_t>(data[1]) + static_cast<size_t>(data[2]); }

			inline void setData(const pxr::GfVec3f& v) { data[0] = v[0]; data[1] = v[1]; data[2] = v[2]; }
			inline void setData(float x, float y, float z) { assert(!encoded_id.is_24bit); data = {x, y, z}; }
			inline void setData(float x, float y, float z, float w) { encoded_id.is_24bit = 1; data = {x, y, z}; }

			inline const std::array<float, 3>& getData() const { return data; }
			inline const void getData(std::array<float24_s, 4>& a) const { assert(encoded_id.is_24bit); }
			inline void getData(pxr::GfVec3f& v) const { assert(!encoded_id.is_24bit); v[0] = data[0]; v[1] = data[1]; v[2] = data[2]; }

			inline void encodeID_modeSPACE(uint32_t id, bool is_tetra, bool is_enclosed);
			inline void decodeID_modeSPACE(uint32_t& id, bool& is_tetra, bool is_enclosed);

			inline void encodeID_modeANGLE(uint32_t guide_id, uint8_t guide_vertex_id);
			inline void decodeID_modeANGLE(uint32_t& guide_id, uint8_t& guide_vertex_id);
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
		void setSkinPrimPath(const std::string& prim_path);

	private:
		size_t calcHash() const;

		std::vector<PointBindData> 	mPointBinds;
		BindMode                    mBindMode = BindMode::NTB;
		std::string                 mSkinPrimPath;

		friend class GuideCurvesDeformer;
};

void to_json(json& j, const GuideCurvesDeformerData::PointBindData& bind);
void from_json(const json& j, GuideCurvesDeformerData::PointBindData& bind);

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