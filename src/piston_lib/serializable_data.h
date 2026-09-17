#ifndef PISTON_LIB_SERIALIZABLEDATA_H_
#define PISTON_LIB_SERIALIZABLEDATA_H_

#include "framework.h"
#include "common.h"

#include <pxr/usd/usd/prim.h>
#include <pxr/base/gf/vec3f.h>

#include <nlohmann/json.hpp>

#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <mutex>

using json = nlohmann::json;

namespace Piston {

class SerializableDeformerDataBase: public std::enable_shared_from_this<SerializableDeformerDataBase>{
	public:
		static const size_t kDefaultReserveElements = 1024;

		struct DataVersion {
			size_t major;
			size_t minor;
			size_t build;

			DataVersion(): major(0), minor(0), build(0) {};
			DataVersion(size_t _maj, size_t _min, size_t _bld): major(_maj), minor(_min), build(_bld) {};
		};

		enum class ErrorCode: uint8_t {
			EC_NO_ERROR		= 0,
			EC_JSON_EMPTY 	= 1,
			EC_PARSE_ERROR	= 2
		};

		SerializableDeformerDataBase();

		bool serialize(BSON& v_bson) const;
		bool deserialize(const BSON& v_bson);
		void clear();

		virtual bool isValid() const = 0;
		virtual const std::string& typeName() const = 0;
		virtual const std::string& jsonDataKey() const = 0;
		virtual const DataVersion& jsonDataVersion() const = 0;
		
	protected:
		virtual bool dumpToJSON(json& j) const = 0;
		virtual bool readFromJSON(const json& j) = 0;
		virtual void clearData() = 0;

		mutable std::mutex mMutex;
};

inline std::string to_string(const SerializableDeformerDataBase::ErrorCode& err) {
	switch(err) {
		case SerializableDeformerDataBase::ErrorCode::EC_JSON_EMPTY :
			return "Empty json/bson";
		case SerializableDeformerDataBase::ErrorCode::EC_PARSE_ERROR :
			return "Json parse error";
		default:
			return ""; 
	}
}

}  // namespace Piston

namespace nlohmann {
    template <>
    struct adl_serializer<pxr::GfVec3f> {
        static void from_json(const json& j, pxr::GfVec3f& v) {
            // Check if JSON entry is actually an array with 3 elements
            if (!j.is_array() || j.size() != 3) {
                throw json::type_error::create(302, "Validation failed: GfVec3f requires a JSON array of 3 numbers", &j);
            }
            v.Set(j[0].get<float>(), j[1].get<float>(), j[2].get<float>());
        }

        static void to_json(json& j, const pxr::GfVec3f& v) {
            j = json::array({v[0], v[1], v[2]});
        }
    };
}

#endif  // PISTON_LIB_SERIALIZABLEDATA_H_
