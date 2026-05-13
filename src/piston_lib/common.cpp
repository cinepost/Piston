#include "common.h"
#include "deformer_factory.h"
#include "topology.h"
#include "serializable_data.h"
#include "simple_profiler.h"
#include "base_curves_deformer.h"
#include "logging.h"

#include <pxr/base/tf/token.h>
#include <pxr/base/tf/pathUtils.h>
#include <pxr/base/vt/value.h>
#include <pxr/base/vt/dictionary.h>
#include <pxr/usd/usd/namespaceEditor.h>

#include <stdio.h>
#include <stdint.h>

static auto compareVtArrays = [](const auto& a, const auto& b) {
    if (a == b) return 0; // Optimization: COW check
    return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end()) ? -1 : 1;
};

PXR_NAMESPACE_OPEN_SCOPE

bool operator<(const pxr::HdMeshTopology& lhs, const pxr::HdMeshTopology& rhs) {
    if (lhs.GetScheme() != rhs.GetScheme()) return lhs.GetScheme() < rhs.GetScheme();
    if (lhs.GetOrientation() != rhs.GetOrientation()) return lhs.GetOrientation() < rhs.GetOrientation();
    if (lhs.GetRefineLevel() != rhs.GetRefineLevel()) return lhs.GetRefineLevel() < rhs.GetRefineLevel();

    int res = ::compareVtArrays(lhs.GetFaceVertexCounts(), rhs.GetFaceVertexCounts());
    if (res != 0) return res < 0;

    res = ::compareVtArrays(lhs.GetFaceVertexIndices(), rhs.GetFaceVertexIndices());
    if (res != 0) return res < 0;

    return false;
}

bool operator<(const HdBasisCurvesTopology& lhs, const HdBasisCurvesTopology& rhs) {
    if (lhs.GetCurveType() != rhs.GetCurveType()) return lhs.GetCurveType() < rhs.GetCurveType();
    if (lhs.GetCurveBasis() != rhs.GetCurveBasis())  return lhs.GetCurveBasis() < rhs.GetCurveBasis();
    if (lhs.GetCurveWrap() != rhs.GetCurveWrap()) return lhs.GetCurveWrap() < rhs.GetCurveWrap();

    int res = ::compareVtArrays(lhs.GetCurveVertexCounts(), rhs.GetCurveVertexCounts());
    if (res != 0) return res < 0;

    res = ::compareVtArrays(lhs.GetCurveIndices(), rhs.GetCurveIndices());
    if (res != 0) return res < 0;

    return false;
}

PXR_NAMESPACE_CLOSE_SCOPE
	
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
			dblBytes = static_cast<double>(bytes) / 1024.0;
		}
	}

	static char output[200];
	sprintf(output, "%.02lf %s", dblBytes, suffix[i]);
	return output;
}

UsdPrimHandle::UsdPrimHandle(): mPrim(pxr::UsdPrim()), mpDeformer(nullptr), mpTopology(nullptr) {}

UsdPrimHandle::UsdPrimHandle(const pxr::UsdPrim& prim): UsdPrimHandle() {  
	mPrim = prim; 
}

UsdPrimHandle::UsdPrimHandle(const BaseCurvesDeformer::SharedPtr& pDeformer): UsdPrimHandle() {
	assert(pDeformer);
	mpDeformer = pDeformer;
}

UsdPrimHandle::UsdPrimHandle(UsdPrimHandle&& other) noexcept : mPrim(std::move(other.mPrim)), mpDeformer(std::move(other.mpDeformer)), mpTopology(std::move(other.mpTopology)) { }

UsdPrimHandle& UsdPrimHandle::operator=(UsdPrimHandle&& other) noexcept {
	if (this != &other) {
		mPrim = std::move(other.mPrim);
		mpDeformer = std::move(other.mpDeformer);
		mpTopology = std::move(other.mpTopology);
	}
	return *this;
}

const pxr::UsdPrim& UsdPrimHandle::getPrim() const {
	static const pxr::UsdPrim sNullPrim;
	if(mpDeformer) {
		return mpDeformer->getOutputPrimHandle().getPrim();
	}
	return mPrim.IsValid() ? mPrim : sNullPrim;
}

void UsdPrimHandle::clear() {
	mPrim = pxr::UsdPrim();
	mpDeformer = nullptr;
	mpTopology = nullptr;
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

bool UsdPrimHandle::prepareDataIfNeeded(pxr::UsdTimeCode time_code) const {
	if(mpDeformer && !mpDeformer->deform(time_code)) {
		LOG_FTL << "Unable to execute " << mpDeformer->getName() << ".deform(...) for " << getPath() << " !!!";
		return false;
	}
	return true;
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

	if(!prepareDataIfNeeded(time_code)) return false;

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

bool UsdPrimHandle::getPoints(pxr::VtArray<pxr::GfVec3f>& array, pxr::UsdTimeCode time_code) const {
	const auto& prim = getPrim();

	auto geom = pxr::UsdGeomPointBased(prim);
	if(!geom) {
		LOG_ERR << "Error getting point based geometry from " << getPath() << " !";
		return false;
	}

	if(!prepareDataIfNeeded(time_code)) return false;

	if(!geom.GetPointsAttr().Get(&array, time_code)) {
		LOG_ERR << "Error getting points from " << getPath() << " !";
		return false;
	}

	return true;
}

template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, pxr::VtArray<int>& array, pxr::UsdTimeCode time_code) const;
template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, pxr::VtArray<uint32_t>& array, pxr::UsdTimeCode time_code) const;
template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, pxr::VtArray<pxr::GfVec3f>& array, pxr::UsdTimeCode time_code) const;

template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, std::vector<int>& vec, pxr::UsdTimeCode time_code) const;
template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, std::vector<uint32_t>& vec, pxr::UsdTimeCode time_code) const;
template bool UsdPrimHandle::fetchAttributeValues(const std::string& attribute_name, std::vector<pxr::GfVec3f>& vec, pxr::UsdTimeCode time_code) const;

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

	auto pStage = getPrim().GetStage();
	assert(pStage); 

	pxr::UsdPrim data_prim = pStage->GetPrimAtPath(prim_path);
	if(!data_prim.IsValid()) {
		LOG_ERR << "Stage has no Piston hidden data prim!";
		return false;
	}

	const pxr::TfToken key_path(identifier);

	const bool readAsMetadata = CurvesDeformerFactory::getDataStorageMethod() == CurvesDeformerFactory::DataToPrimStorageMethod::METADATA;

	if(readAsMetadata) {
		if(!data_prim.HasCustomDataKey(key_path)) {
			LOG_ERR << "Error getting bson from prim " << data_prim << ". No custom data \"" << identifier << "\" exist !!!";
			return false;
		}

		auto _v = data_prim.GetCustomDataByKey(key_path);
		hex_string_to_bson(_v.Get<std::string>(), v_bson);
	} else {
		pxr::UsdAttribute attr = data_prim.HasAttribute(key_path) ? data_prim.GetAttribute(key_path):  data_prim.CreateAttribute(key_path, pxr::SdfValueTypeNames->UCharArray);
		if(attr.GetTypeName() != pxr::SdfValueTypeNames->UCharArray) {
			LOG_ERR << "Error getting bson data to prim " << data_prim << ". Attribute type " << attr.GetTypeName() << " is unsupported!";
			return false;
		}

		attr.Get(&v_bson);
	}

	return true;
}

void UsdPrimHandle::clearPrimBson(const pxr::SdfPath& prim_path, const std::string& identifier) const {
	auto pStage = getPrim().GetStage();
	assert(pStage); 

	pxr::UsdPrim data_prim = pStage->GetPrimAtPath(prim_path);

	const pxr::TfToken token(identifier);

	if(!data_prim.HasCustomDataKey(token)) {
		LOG_ERR << "No bson payload \"" << identifier << "\" exist in prim " << getPath();
		return;
	}

	data_prim.ClearCustomDataByKey(token);
}

const Topology& UsdPrimHandle::getTopology(pxr::UsdTimeCode time_code) const {
	assert(isValid());
	if(!mpTopology || (mpTopology->time_code != time_code)) {
		const auto& _prim = getPrim();
		if(isMeshGeoPrim()) {
			const auto topology = computeMeshTopology(pxr::UsdGeomMesh(_prim), time_code);
			const size_t topology_hash = topology.ComputeHash();
			mpTopology = std::make_unique<Topology>(topology_hash, std::move(topology));
		} else if(isBasisCurvesGeoPrim()) {
			const auto topology = computeCurvesTopology(pxr::UsdGeomBasisCurves(_prim), time_code);
			const size_t topology_hash = topology.ComputeHash();
			mpTopology = std::make_unique<Topology>(topology_hash, std::move(topology));
		} else {
			assert(false);
			LOG_FTL << "Unsupported usd primitive type: " <<  _prim.GetTypeName().GetText();
		}

		assert(mpTopology);
	}

	return *mpTopology.get();
}

size_t UsdPrimHandle::getTopologyHash(pxr::UsdTimeCode time_code) const {
	if(mpTopology && (mpTopology->time_code == time_code)) return mpTopology->topology_hash;

	return getTopology(time_code).topology_hash;
}

std::string getStageName(pxr::UsdStageRefPtr pStage) {
	assert(pStage);
	const std::string identifier = pStage->GetRootLayer()->GetIdentifier();
	return pxr::TfGetBaseName(identifier);
}

bool clearPistonDataFromStage(pxr::UsdStageRefPtr pStage) {
	assert(pStage);
	if(!pStage) {
		LOG_ERR << "Unable to clear Piston data from stage. Stage is invalid!";
		return false;
	}

	LOG_ERR << "UNIMPLEMENTED!!! clearPistonDataFromPrim()";
	return false;
}

bool clearPistonDataFromPrim(pxr::UsdStageRefPtr pStage, const pxr::SdfPath& prim_path) {
	assert(pStage); 
	
	bool result = false;

	LOG_ERR << "UNIMPLEMENTED!!! clearPistonDataFromPrim()";

	return result;
}

bool UsdPrimHandle::setBsonToPrim(const pxr::SdfPath& prim_path, const std::string& identifier, const BSON& v_bson) const {
	assert(!v_bson.empty() && "v_bson is empty");
	assert(isValid() && "prim is invalid");

	if(v_bson.empty() || !isValid()) {
		LOG_ERR << "Unable to set BSON to invalid UsdPrimHandle!";
		return false;
	}

	auto pStage = getPrim().GetStage();
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
	const pxr::UsdPrim& _prim = getPrim();

	if(_prim.IsValid() != prim.IsValid()) {
		return false;
	}

	if(_prim.GetPath() != prim.GetPath()) {
		return false;
	}

	if(_prim.GetStage() != prim.GetStage()) {
		return false;
	}

	return true;
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
