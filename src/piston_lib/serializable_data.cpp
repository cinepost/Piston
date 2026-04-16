#include "serializable_data.h"
#include "logging.h"

#include <type_traits>

namespace Piston {

SerializableDeformerDataBase::SerializableDeformerDataBase(): mIsPopulated(false) {

}

bool SerializableDeformerDataBase::serialize(BSON& v_bson) const {
	if(!isPopulated()) {
		LOG_ERR << "Can't serialize empty deformer data !";
		return false;
	}

	json j = R"({"compact": true, "schema": 0})"_json;
	j["data_name"] = jsonDataKey();
	j["data_version_major"] = jsonDataVersion().major;
	j["data_version_minor"] = jsonDataVersion().minor;
	j["data_version_build"] = jsonDataVersion().build;

	if(!dumpToJSON(j["payload"])) {
		LOG_ERR << "Error serializing deformer data !";
		return false;
	}
  
	#if BSON_USES_PXR_VTARRAY
		std::vector<uint8_t> tmp;
		tmp.reserve(65536);
		json::to_bson(j, tmp);

		v_bson.assign(tmp.begin(), tmp.end());

	#else
		v_bson = json::to_bson(j);
	#endif

	return true;
}

bool SerializableDeformerDataBase::deserialize(const BSON& v_bson) {
	if(v_bson.empty()) return false;

	json j = json::from_bson(v_bson);
	clear();

	if(j["data_name"] != jsonDataKey()) {
		LOG_ERR << "Error de-serializing deformer data ! Data key is different !";
		return false;
	}

	if(j["data_version_major"] != jsonDataVersion().major || j["data_version_minor"] != jsonDataVersion().minor || j["data_version_build"] != jsonDataVersion().build) {
		LOG_ERR << "Error de-serializing deformer data ! Data version is different !";
		return false;
	}

	if(!readFromJSON(j["payload"])) {
		LOG_ERR << "Error de-serializing deformer data !";
		return false;
	}

	mIsPopulated = true;
	return true;
}

void SerializableDeformerDataBase::clear() {
	const std::lock_guard<std::mutex> lock(mMutex);
	clearData();
	mIsPopulated = false;
}

}  // namespace Falcor