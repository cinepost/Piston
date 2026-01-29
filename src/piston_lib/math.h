#ifndef PISTON_LIB_MATH_H_
#define PISTON_LIB_MATH_H_

#include <cmath>

namespace Piston {

inline bool floatsAreEqualRelative(float a, float b, float epsilon = 1e-5f) {
    if (a == b) return true; // Handles infinities
    float diff = std::fabs(a - b);
    float norm = std::min((std::fabs(a) + std::fabs(b)), std::numeric_limits<float>::max());
    return diff < epsilon * norm;
}

}

#endif  // PISTON_LIB_MATH_H_