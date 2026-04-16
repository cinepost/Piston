#include "common.h"
#include "deformer_factory.h"
#include "serializable_data.h"
#include "simple_profiler.h"
#include "logging.h"

#include <pxr/base/tf/token.h>
#include <pxr/base/tf/pathUtils.h>
#include <pxr/base/vt/value.h>
#include <pxr/base/vt/dictionary.h>
#include <pxr/usd/usd/namespaceEditor.h>

#include <stdio.h>
#include <stdint.h>

/*
static pxr::VtArray<uint8_t> bsonToPxrArray(const std::vector<uint8_t>& vec) {
	pxr::Vt_ArrayForeignDataSource fd(nullptr, 1);
	static const bool addRef = 1;
	return {&fd, (uint8_t*)vec.data(), vec.size(), addRef};
}
*/	
	
namespace Piston {

static inline std::string uniqueDataName(const UsdPrimHandle* pPrim, const SerializableDeformerDataBase* pDeformerData) {
	assert(pDeformerData);
	assert(pPrim);
	std::string s = pDeformerData->jsonDataKey() + pPrim->getFullName();
	std::replace(s.begin(), s.end(), '/', '_');
	return s;
}

static inline void bytes_to_hexstr(const BSON& bytes, std::string& hexstr) {
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

static inline void hexstr_to_bytes(const std::string& hexstr, BSON& bytes) {
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
		bytes[i] = lookup[static_cast<unsigned char>(hexstr[i * 2])] << 4 | lookup[static_cast<unsigned char>(hexstr[i * 2 + 1])];
	}
}

const char *stringifyMemSize(size_t bytes) {
	static const char *suffix[] = {"B", "KB", "MB", "GB", "TB"};
	static constexpr const char length = sizeof(suffix) / sizeof(suffix[0]);

	int i = 0;
	double dblBytes = static_cast<double>(bytes);

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

template<typename T>
bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, pxr::VtArray<T>& array, pxr::UsdTimeCode time_code) const {
	if(!isValid()) {
		LOG_ERR << "\"" << getPath() << "\" primitive is invalid!"; 
		return false;
	}

	if(attribute_name.empty()) {
		LOG_ERR << "No attribute name provided !";
		return false;
	}

	pxr::UsdGeomPrimvar primVar = getPrimvarsAPI().GetPrimvar(pxr::TfToken(attribute_name));

	if(!primVar) {
		LOG_ERR << "Error getting \"" << attribute_name << "\" primvar on " << getPath();
		return false;
	}

	array.clear();
	if(!primVar.GetAttr().Get(&array, time_code)) {
		LOG_ERR << "Error getting " << getPath() << " \"" << attribute_name << "\" values !";
		return false;
	}

	return (array.size() > 0);
}

template<typename T>
bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, std::vector<T>& vec, pxr::UsdTimeCode time_code) const {
	pxr::VtArray<T> values_array;
	if(!fetchAttributeValues<T>(attribute_name, values_array, time_code)) {
		return false;
	}

	// TODO: make sure we can't use memcpy here. If we can then just do it!

	const auto& c_values = values_array.AsConst();
	if(vec.size() < c_values.size()) {
		vec.resize(c_values.size());
	}

	for(size_t i = 0; i < c_values.size(); ++i) {
		vec[i] = c_values[i];
	}

	return true;
}

template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, pxr::VtArray<int>& array, pxr::UsdTimeCode time_code) const;
template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, pxr::VtArray<uint32_t>& array, pxr::UsdTimeCode time_code) const;

template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, std::vector<int>& vec, pxr::UsdTimeCode time_code) const;
template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, std::vector<uint32_t>& vec, pxr::UsdTimeCode time_code) const;

bool UsdPrimHandle::getDataFromBson(const pxr::SdfPath& prim_path, SerializableDeformerDataBase* pDeformerData) const {
	assert(pDeformerData);
	BSON v_bson;
	if(!getBsonFromPrim(prim_path, uniqueDataName(this, pDeformerData), v_bson)) {
		return false;
	}

	bool result;

	{
		ScopedTimeMeasure _t("UsdPrimHandle::getDataFromBson deserialize " + pDeformerData->jsonDataKey());
		result = pDeformerData->deserialize(v_bson);
	}

	return result;
}

bool UsdPrimHandle::writeDataToBson(const pxr::SdfPath& prim_path, SerializableDeformerDataBase* pDeformerData) const {
	assert(pDeformerData);
	BSON v_bson;

	{
		ScopedTimeMeasure _t("UsdPrimHandle::writeDataToBson serialize " + pDeformerData->jsonDataKey());

		if(!pDeformerData->serialize(v_bson)) {
			LOG_ERR << "Error serializing data " << pDeformerData->jsonDataKey() << " !!!";
			return false;
		}
	}

	return setBsonToPrim(prim_path, uniqueDataName(this, pDeformerData), v_bson);
}

bool UsdPrimHandle::getBsonFromPrim(const pxr::SdfPath& prim_path, const std::string& identifier, BSON& v_bson) const {

	if(!isValid()) {
		LOG_ERR << "Unable to get BSON from invalid usd prim !";
		return false;
	}

	auto pStage = mPrim.GetStage();
	assert(pStage); 

	pxr::UsdPrim data_prim = pStage->GetPrimAtPath(prim_path);
	if(!data_prim.IsValid()) {
		LOG_ERR << "Stage has no Piston hidden data prim!";
		return false;
	}

	const pxr::TfToken key_path(identifier);

	if(!data_prim.HasCustomDataKey(key_path)) {
		LOG_ERR << "Error getting bson from prim " << data_prim.GetPath() << ". No custom data \"" << identifier << "\" exist !!!";
		return false;
	}

	auto _v = data_prim.GetCustomDataByKey(key_path);
	hex_string_to_bson(_v.Get<std::string>(), v_bson);
	return true;
}

void UsdPrimHandle::clearPrimBson(const pxr::SdfPath& prim_path, const std::string& identifier) const {
	auto pStage = mPrim.GetStage();
	assert(pStage); 

	pxr::UsdPrim data_prim = pStage->GetPrimAtPath(prim_path);

	const pxr::TfToken token(identifier);

	if(!data_prim.HasCustomDataKey(token)) {
		LOG_ERR << "No bson payload \"" << identifier << "\" exist in prim " << getPath();
		return;
	}

	data_prim.ClearCustomDataByKey(token);
}

std::string getStageName(pxr::UsdStageRefPtr pStage) {
	assert(pStage);
	const std::string identifier = pStage->GetRootLayer()->GetIdentifier();
	return pxr::TfGetBaseName(identifier);
}

bool clearAllPrimBson(const pxr::SdfPath& prim_path, pxr::UsdStageRefPtr pStage) {
	assert(pStage); 
	
	bool result = false;

	//LOG_DBG << "clearAllPrimBson " << getStageName(pStage);

	if(pStage->GetPrimAtPath(prim_path).IsValid()) {
		//LOG_DBG << "Got hidden prim";
		pxr::SdfLayerHandle editLayer = pStage->GetEditTarget().GetLayer();

		if (editLayer && editLayer->PermissionToEdit()) {
			//LOG_DBG << "Deleting";
			pxr::UsdNamespaceEditor editor(pStage);

			if (CurvesDeformerFactory::isDefaultDataPrimPath(prim_path) && editor.DeletePrimAtPath(prim_path)) {
				editor.ApplyEdits();
			}
			result = true;
		}
	}

	return result;
}

bool UsdPrimHandle::setBsonToPrim(const pxr::SdfPath& prim_path, const std::string& identifier, const BSON& v_bson) const {
	assert(!v_bson.empty() && "v_bson is empty");
	assert(isValid() && "prim is invalid");

	if(v_bson.empty() || !isValid()) {
		LOG_ERR << "Unable to set BSON to invalid UsdPrimHandle!";
		return false;
	}

	auto pStage = mPrim.GetStage();
	assert(pStage); 

	pxr::UsdPrim data_prim = pStage->GetPrimAtPath(prim_path);
	if(!data_prim.IsValid()) {
		data_prim = pStage->CreateClassPrim(prim_path);
		data_prim.SetHidden(true);
	}

	assert(data_prim.IsValid());

	const pxr::TfToken key_path(identifier);

	const bool storeAsMetadata = CurvesDeformerFactory::getDataStorageMethod() == CurvesDeformerFactory::DataToPrimStorageMethod::METADATA;

	if( storeAsMetadata) {
		// store deformer data as metadata
		if(data_prim.HasCustomDataKey(key_path)) {
			data_prim.ClearCustomDataByKey(key_path);
		}

		const pxr::VtValue _v(bson_to_hex_string(v_bson));
		data_prim.SetCustomDataByKey(key_path, _v);
	} else {
		// store deformer data as attribute
		pxr::UsdAttribute attr = data_prim.HasAttribute(key_path) ? data_prim.GetAttribute(key_path):  data_prim.CreateAttribute(key_path, pxr::SdfValueTypeNames->UCharArray);
		if(attr.GetTypeName() != pxr::SdfValueTypeNames->UCharArray) {
			LOG_ERR << "Error writing bson data to prim " << data_prim << ". Attribute type " << attr.GetTypeName() << "is unsupported!";
			return false;
		}

		attr.Set(v_bson);
	}
	return true;
}

bool UsdPrimHandle::operator==(const pxr::UsdPrim& prim) const {
	if(mPrim.IsValid() != prim.IsValid()) {
		return false;
	}

	if(mPrim.GetPath() != prim.GetPath()) {
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

void PointsList::resize(size_t new_size) {
	if(size() == new_size) return;

	mPoints.resize(new_size);
	mVtArray.resize(new_size);
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
	LOG_TRC << "bson_to_hex_string()";

	ScopedTimeMeasure _t("bson_to_hex_string");

	std::string result;
  	bytes_to_hexstr(bson, result);
	return result;
}

void hex_string_to_bson(const std::string& str, BSON& bson) {
	LOG_TRC << "hex_string_to_bson()";

	ScopedTimeMeasure _t("hex_string_to_bson");

	hexstr_to_bytes(str, bson);
}

} // namespace Piston
