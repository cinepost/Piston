#include "serializable_data.h"


namespace Piston {

SerializableDeformerDataBase::SerializableDeformerDataBase(): mIsPopulated(false) {

}

bool SerializableDeformerDataBase::serialize(std::vector<std::uint8_t>& v_bson) const {
	if(!isPopulated()) {
		std::cerr << "Can't serialize empty deformer data !" << std::endl;
		return false;
	}

	json j = R"({"compact": true, "schema": 0})"_json;
	j["data_name"] = jsonDataKey();
	j["data_version_major"] = jsonDataVersion().major;
	j["data_version_minor"] = jsonDataVersion().minor;
	j["data_version_build"] = jsonDataVersion().build;

	if(!dumpToJSON(j["payload"])) {
		std::cerr << "Error serializing deformer data !" << std::endl;
		return false;
	}

	v_bson = json::to_bson(j);    
	return true;
}

bool SerializableDeformerDataBase::deserialize(const std::vector<std::uint8_t>& v_bson) {
	if(v_bson.empty()) return false;

	json j = json::from_bson(v_bson);
	clear();

	if(j["data_name"] != jsonDataKey()) {
		std::cerr << "Error de-serializing deformer data ! Data key is different !" << std::endl;
		return false;
	}

	if(j["data_version_major"] != jsonDataVersion().major || j["data_version_minor"] != jsonDataVersion().minor || j["data_version_build"] != jsonDataVersion().build) {
		std::cerr << "Error de-serializing deformer data ! Data version is different !" << std::endl;
		return false;
	}

	if(!readFromJSON(j["payload"])) {
		std::cerr << "Error de-serializing deformer data !" << std::endl;
		return false;
	}

	mIsPopulated = true;
	return true;
}

void SerializableDeformerDataBase::clear() {
	clearData();
	mIsPopulated = false;
}

}  // namespace Falcor