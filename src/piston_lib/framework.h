#ifndef PISTON_LIB_FRAMEWORK_H_
#define PISTON_LIB_FRAMEWORK_H_

#include <memory>
#include <iostream>

#include <pxr/base/vt/array.h>
#include <pxr/base/gf/matrix3f.h>

#ifndef should_not_get_here
#define should_not_get_here() assert(false)
#endif

#ifdef _DEBUG
#define dbg_printf(...) printf(__VA_ARGS__)
#else
asdfsdf
#define dbg_printf(...) {}
#endif


namespace Piston {

#define enum_class_operators(e_) \
    inline e_ operator& (e_ a, e_ b) { return static_cast<e_>(static_cast<int>(a)& static_cast<int>(b)); } \
    inline e_ operator| (e_ a, e_ b) { return static_cast<e_>(static_cast<int>(a)| static_cast<int>(b)); } \
    inline e_& operator|= (e_& a, e_ b) { a = a | b; return a; } \
    inline e_& operator&= (e_& a, e_ b) { a = a & b; return a; } \
    inline e_  operator~ (e_ a) { return static_cast<e_>(~static_cast<int>(a)); } \
    inline bool is_set(e_ val, e_ flag) { return (val & flag) != static_cast<e_>(0); } \
    inline void flip_bit(e_& val, e_ flag) { val = is_set(val, flag) ? (val & (~flag)) : (val | flag); }

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