#include "common.h"
#include "serializable_data.h"

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/value.h>


namespace Piston {

bool isMeshGeoPrim(const pxr::UsdPrim& geoPrim) {
	if(geoPrim.GetTypeName() != "Mesh") return false;
	return true;
}

bool isCurvesGeoPrim(const pxr::UsdPrim& geoPrim) {
	if(geoPrim.GetTypeName() != "BasisCurves") return false;
	return true;
}

UsdPrimHandle::UsdPrimHandle(): mPrim(pxr::UsdPrim()) {}

UsdPrimHandle::UsdPrimHandle(const pxr::UsdPrim& prim) {
	mPrim = prim;
}

const pxr::UsdPrim& UsdPrimHandle::getPrim() const {
	static const pxr::UsdPrim sNullPrim;
	return mPrim.IsValid() ? mPrim : sNullPrim;
}

void UsdPrimHandle::clear() {
	mPrim = pxr::UsdPrim();
}

void UsdPrimHandle::clearPrimBson(const std::string& identifier) const {
	pxr::UsdPrim prim = getPrim();
	const pxr::TfToken token(identifier);

	if(!prim.HasCustomDataKey(token)) {
		std::cerr << "No bson payload \"" << identifier << "\" exist in prim " << getPath() << std::endl;
		return;
	}

	prim.ClearCustomDataByKey(token);
	
}

bool UsdPrimHandle::getDataFromBson(SerializableDeformerDataBase* pDeformerData) const {
	assert(pDeformerData);
	std::vector<std::uint8_t> v_bson;
	if(!getBsonFromPrim(pDeformerData->jsonDataKey(), v_bson)) {
		return false;
	}

	return pDeformerData->deserialize(v_bson);
}

bool UsdPrimHandle::writeDataToBson(SerializableDeformerDataBase* pDeformerData) const {
	assert(pDeformerData);
	std::vector<std::uint8_t> v_bson;
	if(!pDeformerData->serialize(v_bson)) {
		return false;
	}

	return setBsonToPrim(pDeformerData->jsonDataKey(), v_bson);
}

bool UsdPrimHandle::getBsonFromPrim(const std::string& identifier, std::vector<std::uint8_t>& v_bson) const {

	if(!isValid()) {
		return false;
	}

	static const pxr::TfToken key("customData");
	const pxr::TfToken key_path(identifier);

	if(!mPrim.HasMetadataDictKey(key, key_path)) {
		std::cerr << "Error getting bson from prim " << getPath() << ". No data \"" << identifier << "\" exist !!! " << std::endl;
		return false;
	}

	return mPrim.GetMetadataByDictKey<std::vector<std::uint8_t>>(key, key_path, &v_bson);
}

bool UsdPrimHandle::setBsonToPrim(const std::string& identifier, const std::vector<std::uint8_t>& v_bson) const {
	if(v_bson.empty()) {
		std::cout << identifier << " bson is empty!" << std::endl;
	}

	if(!isValid()) {
		std::cout << getPath() << " is not valid!" << std::endl;
	}

	if(v_bson.empty() || !isValid()) {
		return false;
	}

	static const pxr::TfToken key("customData");
	const pxr::TfToken key_path(identifier);

	if(mPrim.HasMetadataDictKey(key, key_path)) {
		if(!mPrim.ClearMetadataByDictKey(key, key_path)) {
			std::cerr << "Error clearing old " << identifier << " data on " << getName() << std::endl;
		}
	}

	//const std::string hex_string = bson_to_hex_string(v_bson);
	//return mPrim.SetMetadataByDictKey<std::string>(key, key_path, hex_string);
	const std::string hex_string = bson_to_hex_string(v_bson);
	return mPrim.SetMetadataByDictKey<std::vector<std::uint8_t>>(key, key_path, v_bson);
}

bool UsdPrimHandle::operator==(const pxr::UsdPrim& prim) const {
	if(mPrim.IsValid() != prim.IsValid()) {
		return false;
	}

	if(mPrim.GetPath() != prim.GetPath()) {
		dbg_printf("Prim paths are different !!!\n");
		return false;
	}

	return true;
}

std::string bson_to_hex_string(const std::vector<uint8_t>& vec) {
  std::stringstream ss;

  ss << std::hex << std::setfill('0');

  for (size_t i = 0; i < vec.size(); ++i) {
    ss << std::hex << std::setw(2) << static_cast<int>(vec[i]);
  }

  return ss.str();
}

std::vector<uint8_t> hex_string_to_bson(const std::string& str) {
	std::ostringstream ret;
	std::string result;
	std::vector<uint8_t> bson;
	
	uint8_t byte;
	for (std::string::size_type i = 0; i < str.length(); ++i) {

		ret << std::hex << std::setfill('0') << std::setw(2) << (int)str[i];
		result = ret.str();
		byte = (uint8_t) strtol(result.c_str(), nullptr, 10);
		bson.push_back(byte);
		ret.str("");
		ret.clear();
	}
	return bson;
}

} // namespace Piston
