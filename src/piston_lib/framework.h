#ifndef PISTON_LIB_FRAMEWORK_H_
#define PISTON_LIB_FRAMEWORK_H_

#include <memory>
#include <iostream>

#include <pxr/base/gf/matrix3f.h>

#ifndef should_not_get_here
#define should_not_get_here() assert(false)
#endif

#ifdef _DEBUG
#define dbg_printf(...) printf(__VA_ARGS__)
#else
#define dbg_printf(...) {}
#endif

#define PIXAR_POINTS_SYNAMIC_ARRAY_USE_PXR false

namespace Piston {

#if PIXAR_POINTS_SYNAMIC_ARRAY_USE_PXR
    using PixarPointsDynamicArray = pxr::VtArray<pxr::GfVec3f>;
#else
    using PixarPointsDynamicArray = std::vector<pxr::GfVec3f>;
#endif


// This is a helper class which should be used in case a class derives from a base class which derives from enable_shared_from_this
// If Derived will also inherit enable_shared_from_this, it will cause multiple inheritance from enable_shared_from_this, which results in a runtime errors because we have 2 copies of the WeakPtr inside shared_ptr
template<typename Base, typename Derived>
class inherit_shared_from_this {
public:
    typename std::shared_ptr<Derived> shared_from_this() {
        Base* pBase = static_cast<Derived*>(this);
        std::shared_ptr<Base> pShared = pBase->shared_from_this();
        return std::static_pointer_cast<Derived>(pShared);
    }

    typename std::shared_ptr<const Derived> shared_from_this() const {
        const Base* pBase = static_cast<const Derived*>(this);
        std::shared_ptr<const Base> pShared = pBase->shared_from_this();
        return std::static_pointer_cast<const Derived>(pShared);
    }
};

} // namespace Piston

#endif // PISTON_LIB_FRAMEWORK_H_