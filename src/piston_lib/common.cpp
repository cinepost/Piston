#include "common.h"
#include "serializable_data.h"

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/value.h>


namespace Piston {

bool isMeshGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim->GetTypeName() != "Mesh") return false;
	return true;
}

bool isCurvesGeoPrim(pxr::UsdPrim* pGeoPrim) {
	if(pGeoPrim->GetTypeName() != "BasisCurves") return false;
	return true;
}

UsdPrimHandle::UsdPrimHandle(): mpStage(nullptr) {}

UsdPrimHandle::UsdPrimHandle(pxr::UsdStageWeakPtr pStage, const pxr::SdfPath& path): mpStage(pStage), mPath(path) {
	assert(mpStage);
}

UsdPrimHandle::UsdPrimHandle(const pxr::UsdPrim* pPrim) {
	assert(pPrim);

	mpStage = pPrim->GetStage();
	mPath = pPrim->GetPath();
}

pxr::UsdPrim UsdPrimHandle::getPrim() const {
	return mpStage ? mpStage->GetPrimAtPath(mPath) : pxr::UsdPrim();
}

void UsdPrimHandle::clear() {
	mpStage = nullptr;
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

bool UsdPrimHandle::operator==(const pxr::UsdPrim* pPrim) const {
	if(!pPrim) return false;
	return mpStage == pPrim->GetStage() && mPath == pPrim->GetPath();
}

} // namespace Piston
