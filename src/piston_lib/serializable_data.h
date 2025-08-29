#ifndef PISTON_LIB_SERIALIZABLEDATA_H_
#define PISTON_LIB_SERIALIZABLEDATA_H_

#include "framework.h"
#include "common.h"

#include <pxr/usd/usd/prim.h>

#include <nlohmann/json.hpp>

#include <fstream>
#include <vector>
#include <string>

using json = nlohmann::json;

namespace Piston {

class SerializableDeformerDataBase {
	public:
		static const size_t kDefaultReserveElements = 1024;

		struct DataVersion {
			size_t major;
			size_t minor;
			size_t build;

			DataVersion(): major(0), minor(0), build(0) {};
			DataVersion(size_t _maj, size_t _min, size_t _bld): major(_maj), minor(_min), build(_bld) {};
		};

		enum class ErrorCode {
			NO_ERROR,
			JSON_EMPTY,
			PARSE_ERROR
		};

		SerializableDeformerDataBase();

		bool serialize(std::vector<std::uint8_t>& v_bson) const;
		bool deserialize(const std::vector<std::uint8_t>& v_bson);
		void clear();

		bool isPopulated() const { return mIsPopulated; }

		virtual const std::string& typeName() const = 0;
		virtual const std::string& jsonDataKey() const = 0;
		virtual const DataVersion& jsonDataVersion() const = 0;
		
	protected:
		void setPopulated(bool state) { mIsPopulated = state; }

		virtual bool dumpToJSON(json& j) const = 0;
		virtual bool readFromJSON(const json& j) = 0;
		virtual void clearData() = 0;

	private:
		bool mIsPopulated;

};

inline std::string to_string(const SerializableDeformerDataBase::ErrorCode& err) {
	switch(err) {
		case SerializableDeformerDataBase::ErrorCode::JSON_EMPTY :
			return "Empty json/bson";
		case SerializableDeformerDataBase::ErrorCode::PARSE_ERROR :
			return "Json parse error";
		default:
			return ""; 
	}
}

}  // namespace Piston

#endif  // PISTON_LIB_SERIALIZABLEDATA_H_
