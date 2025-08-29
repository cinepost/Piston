#include "serializable_data.h"


namespace Piston {

SerializableDeformerDataBase::SerializableDeformerDataBase(): mIsPopulated(false) {

}

bool SerializableDeformerDataBase::serialize(std::vector<std::uint8_t>& v_bson) const {
	if(!mIsPopulated) {
		std::cerr << "Can't serialize empty deformer data !" << std::endl;
		return false;
	}

	json j = R"({"compact": true, "schema": 0})"_json;
	j["data_name"] = jsonDataKey();
	j["data_version_major"] = jsonDataVersion().major;
	j["data_version_minor"] = jsonDataVersion().minor;
	j["data_version_build"] = jsonDataVersion().build;

	if(!dumpToJSON(j)) {
		std::cerr << "Error serializing deformer data !" << std::endl;
		return false;
	}

	v_bson = json::to_bson(j);

	return true;
}

bool SerializableDeformerDataBase::deserialize(const std::vector<std::uint8_t>& v_bson) {
	if(v_bson.empty()) return false;

	json j = json::from_bson(v_bson);

	if(!readFromJSON(j)) {
		std::cerr << "Error de-serializing deformer data !" << std::endl;
		mIsPopulated = false;
		return false;
	}

	mIsPopulated = true;
}

void SerializableDeformerDataBase::clear() {
	clearData();
	mIsPopulated = false;
}

}  // namespace Falcor