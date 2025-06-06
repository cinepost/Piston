#ifndef PISTON_LIB_COMMON_H_
#define PISTON_LIB_COMMON_H_

#include "framework.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/primvarsAPI.h>

namespace Piston {

bool isMeshGeoPrim(pxr::UsdPrim* pGeoPrim);
bool isCurvesGeoPrim(pxr::UsdPrim* pGeoPrim);

class UsdPrimHandle {
	public:
		UsdPrimHandle();
		UsdPrimHandle(pxr::UsdStageWeakPtr pStage, const pxr::SdfPath& path);
		UsdPrimHandle(const pxr::UsdPrim* pPrim);

		pxr::UsdPrim getPrim() const;

		bool isMeshGeoPrim() const { return getPrim().GetTypeName() == "Mesh"; }
		bool isHairGeoPrim() const { return getPrim().GetTypeName() == "BasisCurves"; }

		const std::string&  	getName() const { return mPath.GetName(); }
		const pxr::SdfPath& 	getPath() const { return mPath; }
		pxr::UsdStageWeakPtr 	getStage() { return mpStage; }

		pxr::UsdGeomPrimvarsAPI getPrimvarsAPI() const { return pxr::UsdGeomPrimvarsAPI::Get(mpStage, mPath); }

		/* Invalidate handle */
		void                    clear();

		bool operator==(const pxr::UsdPrim* pPrim) const;
		explicit operator bool() const { return mpStage && mpStage->GetPrimAtPath(mPath).IsValid(); };

	private:
		pxr::UsdStageWeakPtr 	mpStage;
		pxr::SdfPath 			mPath;

};

} // namespace Piston

#endif // PISTON_LIB_COMMON_H_