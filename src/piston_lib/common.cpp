#include "common.h"
#include "serializable_data.h"

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/value.h>


static pxr::VtArray<uint8_t> bsonToPxrArray(const std::vector<uint8_t>& vec) {
	pxr::Vt_ArrayForeignDataSource fd(nullptr, 1);
	static const bool addRef = 1;
	return {&fd, (uint8_t*)vec.data(), vec.size(), addRef};
}
	
	
namespace Piston {

const char *stringifyMemSize(size_t bytes) {
	static const char *suffix[] = {"B", "KB", "MB", "GB", "TB"};
	static constexpr const char length = sizeof(suffix) / sizeof(suffix[0]);

	int i = 0;
	double dblBytes = bytes;

	if (bytes > 1024) {
		for (i = 0; (bytes / 1024) > 0 && i<length-1; i++, bytes /= 1024) {
			dblBytes = bytes / 1024.0;
		}
	}

	static char output[200];
	sprintf(output, "%.02lf %s", dblBytes, suffix[i]);
	return output;
}

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

double UsdPrimHandle::getStageFPS() const {
	if(!isValid()) {
		return 0.0;
	}

	return getStage()->GetFramesPerSecond();
}

double UsdPrimHandle::getStageTimeCodesPerSecond() const {
	if(!isValid()) {
		return 1.0;
	}

	return getStage()->GetTimeCodesPerSecond();
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

	pxr::VtArray<uint8_t> v;
	if(!mPrim.GetMetadataByDictKey(key, key_path, &v)) {
		return false;
	}

	if(v.empty()) return false;

	v_bson.resize(v.size());
	for(size_t i = 0; i < v.size(); ++i) v_bson[i] = v[i];

	return true;
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


	// TODO: serialize directly to VtArray!
	pxr::VtArray<uint8_t> v(v_bson.size());
	for(size_t i = 0; i < v_bson.size(); ++i) v[i] = v_bson[i];

	const bool result = mPrim.SetMetadataByDictKey<pxr::VtArray<uint8_t>>(key, key_path, v);
	return result;
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

PointsList::PointsList(size_t size): mPoints(size), mVtArray(&mForeignDataSource, (pxr::GfVec3f*)mPoints.data(), mPoints.size(), true) {
	assert(size > 0);
	calcSizeInBytes();
}

PointsList::PointsList(Piston::PointsList&& other): mPoints(std::move(other.mPoints)), mVtArray(&mForeignDataSource, (pxr::GfVec3f*)mPoints.data(), mPoints.size(), false), mSizeInBytes(other.mSizeInBytes) {
}

void PointsList::resize(size_t size) {
	mPoints.resize(size);
	mVtArray.resize(size);
	calcSizeInBytes();
}

void PointsList::fillWithZero() {
	static const pxr::GfVec3f zero = {0.0, 0.0, 0.0};
	std::fill(mPoints.begin(), mPoints.end(), zero);
}

void PointsList::calcSizeInBytes() const {
	mSizeInBytes = mPoints.size() * sizeof(pxr::GfVec3f);
}

std::string bson_to_hex_string(const std::vector<uint8_t>& vec, bool truncate) {
  std::stringstream ss;

  ss << std::hex << std::setfill('0');

  static const size_t truncated_string_max_size = 64;
  const size_t _size = truncate ? std::min(truncated_string_max_size, vec.size()) : vec.size(); 

  for (size_t i = 0; i < vec.size(); ++i) {
    ss << std::hex << std::setw(2) << static_cast<int>(vec[i]);
  }

  if(truncate) ss << "...";

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
