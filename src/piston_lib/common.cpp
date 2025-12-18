#include "common.h"
#include "serializable_data.h"
#include "simple_profiler.h"

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/value.h>

#include <stdio.h>
#include <stdint.h>


static pxr::VtArray<uint8_t> bsonToPxrArray(const std::vector<uint8_t>& vec) {
	pxr::Vt_ArrayForeignDataSource fd(nullptr, 1);
	static const bool addRef = 1;
	return {&fd, (uint8_t*)vec.data(), vec.size(), addRef};
}
	
	
namespace Piston {

static const pxr::SdfPath sHiddenPrimPath("/__piston_data__");

static inline void bytes_to_hexstr(const std::vector<uint8_t>& bytes, std::string& hexstr) {
	static const uint8_t lookup[]= "0123456789abcdef";
	
	if (bytes.empty()) {
		return;
	}

	hexstr.resize(bytes.size() * 2);

	for (size_t i = 0; i < bytes.size(); ++i) {
		hexstr[i * 2 + 0] = lookup[(bytes[i] >> 4) & 0x0F];
		hexstr[i * 2 + 1] = lookup[(bytes[i]     ) & 0x0F];
	}
}

static inline void hexstr_to_bytes(const std::string& hexstr, std::vector<uint8_t>& bytes) {
	static const uint8_t lookup[] = {
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //  !"#$%&'
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ()*+,-./
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, // 01234567
		0x08, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 89:;<=>?
		0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, // @ABCDEFG
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // HIJKLMNO
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // PQRSTUVW
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // XYZ[\]^_
		0x00, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, // `abcdefg
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // hijklmno
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // pqrstuvw
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // xyz{|}~.
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // ........
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // ........
	};
  
	size_t bytes_count = hexstr.size() / 2;
	bytes.resize(bytes_count);

	for(size_t i=0; i<bytes_count; ++i) {
		bytes[i] = lookup[hexstr[i * 2]] << 4 | lookup[hexstr[i * 2 + 1]];
	}
}

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
	BSON v_bson;
	if(!getBsonFromPrim(pDeformerData->jsonDataKey(), v_bson)) {
		return false;
	}

	bool result;

	{
		ScopedTimeMeasure _t("UsdPrimHandle::getDataFromBson deserialize " + pDeformerData->jsonDataKey());
		result = pDeformerData->deserialize(v_bson);
	}

	return result;
}

bool UsdPrimHandle::writeDataToBson(SerializableDeformerDataBase* pDeformerData) const {
	assert(pDeformerData);
	BSON v_bson;

	{
		ScopedTimeMeasure _t("UsdPrimHandle::writeDataToBson serialize " + pDeformerData->jsonDataKey());

		if(!pDeformerData->serialize(v_bson)) {
			return false;
		}
	}

	return setBsonToPrim(pDeformerData->jsonDataKey(), v_bson);
}

bool UsdPrimHandle::getBsonFromPrim(const std::string& identifier, BSON& v_bson) const {

	if(!isValid()) {
		return false;
	}

	auto pStage = mPrim.GetStage();
	assert(pStage); 

	pxr::UsdPrim data_prim = pStage->GetPrimAtPath(sHiddenPrimPath);
	if(!data_prim.IsValid()) {
		dbg_printf("Stage has no Piston hidden data prim!\n");
		return false;
	}

	const pxr::TfToken key_path(identifier);

	if(!data_prim.HasCustomDataKey(key_path)) {
		std::cerr << "Error getting bson from prim " << getPath() << ". No custom data \"" << identifier << "\" exist !!! " << std::endl;
		return false;
	}

	auto _v = data_prim.GetCustomDataByKey(key_path);
	hex_string_to_bson(_v.Get<std::string>(), v_bson);
	return true;
}

bool UsdPrimHandle::setBsonToPrim(const std::string& identifier, const BSON& v_bson) const {
	if(v_bson.empty()) {
		std::cout << identifier << " bson is empty!" << std::endl;
	}

	if(!isValid()) {
		std::cout << getPath() << " is not valid!" << std::endl;
	}

	if(v_bson.empty() || !isValid()) {
		return false;
	}

	auto pStage = mPrim.GetStage();
	assert(pStage); 

	pxr::UsdPrim data_prim = pStage->GetPrimAtPath(sHiddenPrimPath);
	if(!data_prim.IsValid()) {
		data_prim = pStage->DefinePrim(sHiddenPrimPath);
		data_prim.SetHidden(true);
	}

	assert(data_prim.IsValid());

	const pxr::TfToken key_path(identifier);

	if(data_prim.HasCustomDataKey(key_path)) {
		data_prim.ClearCustomDataByKey(key_path);
	}

	const pxr::VtValue _v(bson_to_hex_string(v_bson));
	
	data_prim.SetCustomDataByKey(key_path, _v);
	return true;
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

std::string bson_to_hex_string(const BSON& bson) {
	dbg_printf("bson_to_hex_string()\n");

	ScopedTimeMeasure _t("bson_to_hex_string");

	std::string result;
  	bytes_to_hexstr(bson, result);
	return result;
}

void hex_string_to_bson(const std::string& str, BSON& bson) {
	dbg_printf("hex_string_to_bson()\n");

	ScopedTimeMeasure _t("hex_string_to_bson");

	hexstr_to_bytes(str, bson);
}

} // namespace Piston
