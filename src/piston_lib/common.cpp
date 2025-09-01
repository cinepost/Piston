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

pxr::UsdPrim UsdPrimHandle::getPrim() const {
	return mPrim.IsValid() ? mPrim : pxr::UsdPrim();
}

void UsdPrimHandle::clear() {
	mPrim = pxr::UsdPrim();
}

void UsdPrimHandle::clearPrimBson(const std::string& identifier) const {
	pxr::UsdPrim prim = getPrim();
	const pxr::TfToken token(identifier);

	if(!prim.HasCustomDataKey(token)) {
		std::cout << "No bson payload \"" << identifier << "\" exist in prim " << getPath();
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
	pxr::UsdPrim prim = getPrim();
	const pxr::TfToken token(identifier);

	if(!prim.HasCustomDataKey(token)) {
		std::cerr << "Error getting bson from prim " << getPath() << ". No payload \"" << identifier << "\" exist !!! ";
		return false;
	}

	v_bson = prim.GetCustomDataByKey(token).Get<std::vector<std::uint8_t>>();

	return true;
}

bool UsdPrimHandle::setBsonToPrim(const std::string& identifier, const std::vector<std::uint8_t>& v_bson) const {
	if(v_bson.empty()) {
		return false;
	}

	pxr::UsdPrim prim = getPrim();
	const pxr::TfToken token(identifier);

	if(prim.HasCustomDataKey(token)) {
		prim.ClearCustomDataByKey(token);
	}

	prim.SetCustomDataByKey(token, pxr::VtValue(v_bson));

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

} // namespace Piston
