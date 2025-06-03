#include "common.h"

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

bool UsdPrimHandle::operator==(const pxr::UsdPrim* pPrim) const {
	if(!pPrim) return false;
	return mpStage == pPrim->GetStage() && mPath == pPrim->GetPath();
}

} // namespace Piston
