/***************************************************************************
 # Copyright (c) 2015-24, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#ifndef PISTON_LIB_FLOAT16_H_
#define PISTON_LIB_FLOAT16_H_

#include "platform.h"

#include <cstdint>
#include <limits>

static float overflow() {
    volatile float f = 1e10;
    for (int i = 0; i < 10; ++i) {
        f *= f; // this will overflow before the for loop terminates
    }
    return f;
}

union uif32 {
    float f;
    unsigned int i;
};

static inline uint16_t float32ToFloat16(float value) {
    uif32 entry;
    entry.f = value;
    int i = static_cast<int>(entry.i);

    //
    // Our floating point number, f, is represented by the bit
    // pattern in integer i.  Disassemble that bit pattern into
    // the sign, s, the exponent, e, and the significand, m.
    // Shift s into the position where it will go in the
    // resulting half number.
    // Adjust e, accounting for the different exponent bias
    // of float and half (127 versus 15).
    //

    int s = (i >> 16) & 0x00008000;
    int e = ((i >> 23) & 0x000000ff) - (127 - 15);
    int m = i & 0x007fffff;

    //
    // Now reassemble s, e and m into a half:
    //

    if (e <= 0) {
        if (e < -10) {
            //
            // E is less than -10.  The absolute value of f is
            // less than half_MIN (f may be a small normalized
            // float, a denormalized float or a zero).
            //
            // We convert f to a half zero.
            //

            return uint16_t(s);
        }

        //
        // E is between -10 and 0.  F is a normalized float,
        // whose magnitude is less than __half_NRM_MIN.
        //
        // We convert f to a denormalized half.
        //

        m = (m | 0x00800000) >> (1 - e);

        //
        // Round to nearest, round "0.5" up.
        //
        // Rounding may cause the significand to overflow and make
        // our number normalized.  Because of the way a half's bits
        // are laid out, we don't have to treat this case separately;
        // the code below will handle it correctly.
        //

        if (m & 0x00001000)
            m += 0x00002000;

        //
        // Assemble the half from s, e (zero) and m.
        //

        return uint16_t(s | (m >> 13));
    }
    else if (e == 0xff - (127 - 15))
    {
        if (m == 0) {
            //
            // F is an infinity; convert f to a half
            // infinity with the same sign as f.
            //

            return uint16_t(s | 0x7c00);
        } else {
            //
            // F is a NAN; we produce a half NAN that preserves
            // the sign bit and the 10 leftmost bits of the
            // significand of f, with one exception: If the 10
            // leftmost bits are all zero, the NAN would turn
            // into an infinity, so we have to set at least one
            // bit in the significand.
            //

            m >>= 13;

            return uint16_t(s | 0x7c00 | m | (m == 0));
        }
    } else {
        //
        // E is greater than zero.  F is a normalized float.
        // We try to convert f to a normalized half.
        //

        //
        // Round to nearest, round "0.5" up
        //

        if (m & 0x00001000) {
            m += 0x00002000;

            if (m & 0x00800000) {
                m = 0;  // overflow in significand,
                e += 1; // adjust exponent
            }
        }

        //
        // Handle exponent overflow
        //

        if (e > 30) {
            overflow(); // Cause a hardware floating point overflow;
            return uint16_t(s | 0x7c00); // Return infinity with same sign as f.
        }

        //
        // Assemble the half from s, e and m.
        //

        return uint16_t(s | (e << 10) | (m >> 13));
    }
}

static inline float float16ToFloat32(uint16_t value) {
        int s = (value >> 15) & 0x00000001;
    int e = (value >> 10) & 0x0000001f;
    int m = value & 0x000003ff;

    if (e == 0) {
        if (m == 0) {
            //
            // Plus or minus zero
            //

            uif32 result;
            result.i = static_cast<unsigned int>(s << 31);
            return result.f;
        } else {
            //
            // Denormalized number -- renormalize it
            //

            while (!(m & 0x00000400)) {
                m <<= 1;
                e -= 1;
            }

            e += 1;
            m &= ~0x00000400;
        }
    } else if (e == 31) {
        if (m == 0) {
            //
            // Positive or negative infinity
            //

            uif32 result;
            result.i = static_cast<unsigned int>((s << 31) | 0x7f800000);
            return result.f;
        } else {
            //
            // Nan -- preserve sign and significand bits
            //

            uif32 result;
            result.i = static_cast<unsigned int>((s << 31) | 0x7f800000 | (m << 13));
            return result.f;
        }
    }

    //
    // Normalized number
    //

    e = e + (127 - 15);
    m = m << 13;

    //
    // Assemble s, e and m.
    //

    uif32 result;
    result.i = static_cast<unsigned int>((s << 31) | (e << 23) | m);
    return result.f;
}

struct float16_t {
    float16_t() = default;

    float16_t(uint32_t sign, uint32_t exponent, uint32_t fraction)
        : mBits((sign & 0x01) << 15 | (exponent & 0x1f) << 10 | (fraction & 0x03ff))
    {}

    explicit float16_t(float value) : mBits(float32ToFloat16(value)) {}

    template<typename T>
    explicit float16_t(T value) : mBits(float32ToFloat16(static_cast<float>(value)))
    {}

    operator float() const { return float16ToFloat32(mBits); }

    static constexpr float16_t fromBits(uint16_t bits) { return float16_t(bits, FromBits); }
    uint16_t toBits() const { return mBits; }

    float16_t& operator=(const float value) { mBits = float32ToFloat16(value); return *this; }

    bool operator==(const float16_t other) const { return mBits == other.mBits; }
    bool operator!=(const float16_t other) const { return mBits != other.mBits; }
    bool operator<(const float16_t other) const { return static_cast<float>(*this) < static_cast<float>(other); }
    bool operator<=(const float16_t other) const { return static_cast<float>(*this) <= static_cast<float>(other); }
    bool operator>(const float16_t other) const { return static_cast<float>(*this) > static_cast<float>(other); }
    bool operator>=(const float16_t other) const { return static_cast<float>(*this) >= static_cast<float>(other); }

    float16_t operator+() const { return *this; }
    float16_t operator-() const { return fromBits(mBits ^ 0x8000); }

    // TODO: Implement math operators in native fp16 precision. For now using fp32.
    float16_t operator+(const float16_t other) const { return float16_t(static_cast<float>(*this) + static_cast<float>(other)); }
    float16_t operator-(const float16_t other) const { return float16_t(static_cast<float>(*this) - static_cast<float>(other)); }
    float16_t operator*(const float16_t other) const { return float16_t(static_cast<float>(*this) * static_cast<float>(other)); }
    float16_t operator/(const float16_t other) const { return float16_t(static_cast<float>(*this) / static_cast<float>(other)); }

    float16_t operator+=(const float16_t other) { return *this = *this + other; }
    float16_t operator-=(const float16_t other) { return *this = *this - other; }
    float16_t operator*=(const float16_t other) { return *this = *this * other; }
    float16_t operator/=(const float16_t other) { return *this = *this / other; }

    constexpr bool isFinite() const noexcept { return exponent() < 31; }
    constexpr bool isInf() const noexcept { return exponent() == 31 && mantissa() == 0; }
    constexpr bool isNan() const noexcept { return exponent() == 31 && mantissa() != 0; }
    constexpr bool isNormalized() const noexcept { return exponent() > 0 && exponent() < 31; }
    constexpr bool isDenormalized() const noexcept { return exponent() == 0 && mantissa() != 0; }

//private:
    enum Tag {
        FromBits
    };

    constexpr float16_t(uint16_t bits, Tag) : mBits(bits) {}

    constexpr uint16_t mantissa() const noexcept { return mBits & 0x3ff; }
    constexpr uint16_t exponent() const noexcept { return (mBits >> 10) & 0x001f; }

    uint16_t mBits;
};

#if PISTON_MSVC
#pragma warning(push)
#pragma warning(disable : 4455) // disable warning about literal suffixes not starting with an underscore
#elif PISTON_CLANG
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wuser-defined-literals"
#elif PISTON_GCC
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wliteral-suffix"
#endif

/// h suffix for "half float" literals.
inline float16_t operator""h(long double value) {
    return float16_t(static_cast<float>(value));
}

#if PISTON_MSVC
#pragma warning(pop)
#elif PISTON_CLANG
#pragma clang diagnostic pop
#elif PISTON_GCC
#pragma GCC diagnostic pop
#endif

namespace std {

template<>
class numeric_limits<float16_t> {
public:
    static constexpr bool is_specialized = true;
    static constexpr float16_t min() noexcept { return float16_t::fromBits(0x0200); }
    static constexpr float16_t max() noexcept { return float16_t::fromBits(0x7bff); }
    static constexpr float16_t lowest() noexcept { return float16_t::fromBits(0xfbff); }
    static constexpr int digits = 11;
    static constexpr int digits10 = 3;
    static constexpr bool is_signed = true;
    static constexpr bool is_integer = false;
    static constexpr bool is_exact = false;
    static constexpr int radix = 2;
    static constexpr float16_t epsilon() noexcept { return float16_t::fromBits(0x1200); }
    static constexpr float16_t round_error() noexcept { return float16_t::fromBits(0x3c00); }
    static constexpr int min_exponent = -13;
    static constexpr int min_exponent10 = -4;
    static constexpr int max_exponent = 16;
    static constexpr int max_exponent10 = 4;
    static constexpr bool has_infinity = true;
    static constexpr bool has_quiet_NaN = true;
    static constexpr bool has_signaling_NaN = true;
    static constexpr float_denorm_style has_denorm = denorm_absent;
    static constexpr bool has_denorm_loss = false;
    static constexpr float16_t infinity() noexcept { return float16_t::fromBits(0x7c00); }
    static constexpr float16_t quiet_NaN() noexcept { return float16_t::fromBits(0x7fff); }
    static constexpr float16_t signaling_NaN() noexcept { return float16_t::fromBits(0x7dff); }
    static constexpr float16_t denorm_min() noexcept { return float16_t::fromBits(0); }
    static constexpr bool is_iec559 = false;
    static constexpr bool is_bounded = false;
    static constexpr bool is_modulo = false;
    static constexpr bool traps = false;
    static constexpr bool tinyness_before = false;
    static constexpr float_round_style round_style = round_to_nearest;
};

} // namespace std

#endif // SRC_PISTON_UTILS_MATH_FLOAT16_H_