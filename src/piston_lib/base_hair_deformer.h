#ifndef PISTON_LIB_BASE_HAIR_DEFORMER_H_
#define PISTON_LIB_BASE_HAIR_DEFORMER_H_

#include "framework.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>

class UsdPrimHandle {
	public:
		UsdPrimHandle();
		UsdPrimHandle(pxr::UsdStageWeakPtr pStage, const pxr::SdfPath& path);
		UsdPrimHandle(const pxr::UsdPrim* pPrim);

		pxr::UsdPrim getPrim() const;

		const std::string&  	getName() const { return mPath.GetName(); }
		const pxr::SdfPath& 	getPath() const { return mPath; }
		pxr::UsdStageWeakPtr 	getStage() { return mpStage; }
		
		/* Invalidate handle */
		void                    clear();

		bool operator==(const pxr::UsdPrim* pPrim) const;

	private:
		pxr::UsdStageWeakPtr 	mpStage;
		pxr::SdfPath 			mPath;

};

class BaseHairDeformer : public std::enable_shared_from_this<BaseHairDeformer> {
	public:
		using SharedPtr = std::shared_ptr<BaseHairDeformer>;

		enum class Type { 
			FAST, 
			INTERPOLATED, 
			VOLUMETRIC,
			UNKNOWN 
		};
		
	public:
		void setMeshGeoPrim(pxr::UsdPrim* pGeoPrim);
		void setHairGeoPrim(pxr::UsdPrim* pGeoPrim);
		
		virtual bool deform() = 0;

		virtual const std::string& toString() const;

	protected:
		BaseHairDeformer();
		virtual bool buildDeformerData() = 0;

	private:
		UsdPrimHandle mMeshGeoPrimHandle;
		UsdPrimHandle mHairGeoPrimHandle;
};

inline std::string to_string(BaseHairDeformer::Type mt) {
#define t2s(t_) case BaseHairDeformer::Type::t_: return #t_;
    switch (mt) {
        t2s(FAST);
        t2s(INTERPOLATED);
        t2s(VOLUMETRIC);
        default:
            should_not_get_here();
            return "UNKNOWN";
    }
#undef t2s
}

#endif // PISTON_LIB_BASE_HAIR_DEFORMER_H_