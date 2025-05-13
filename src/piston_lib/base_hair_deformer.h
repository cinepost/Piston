#ifndef PISTON_LIB_BASE_HAIR_DEFORMER_H_
#define PISTON_LIB_BASE_HAIR_DEFORMER_H_

#include "framework.h"

#include <memory>
#include <string>
#include <pxr/usd/usd/prim.h>

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
		void setRestGeoPrim(pxr::UsdPrim* pRestGeoPrim);
		virtual const std::string& greet() const;

	protected:
		BaseHairDeformer();

	private:
		Type mType = Type::UNKNOWN;
		pxr::UsdPrim *mpRestGeoPrim;
		pxr::UsdPrim *mpDeformedGeoPrim;
		pxr::UsdPrim *mpHairPrim;
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