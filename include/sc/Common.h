#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace StepperControl {

#ifdef NDEBUG

#ifdef __GNUG__
#define FORCE_INLINE __attribute__((always_inline))
#elif defined(_MSC_VER)
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE inline
#endif

#define scAssert(_Expression) ((void)0)
#define scExecute(_Expression) ((void)0)

#else /* NDEBUG */

#define FORCE_INLINE inline

#define scAssert(_Expression)                                                                      \
    do {                                                                                           \
        if (!(_Expression))                                                                        \
            throw std::logic_error((__FILE__ "(") + std::to_string(__LINE__) +                     \
                                   ("): " #_Expression " "));                                      \
    } while (false)

#endif /* NDEBUG */

inline int32_t lTruncTowardZero(float v) { return static_cast<int32_t>(v); }

inline int32_t lTruncTowardInf(float v) {
    return static_cast<int32_t>(v < 0.0f ? floor(v) : ceil(v));
}

FORCE_INLINE float inf() { return std::numeric_limits<float>::infinity(); }

template <typename T>
struct Clamp {
    inline Clamp(T minV, T maxV) : minVal(minV), maxVal(maxV) {}
    inline T operator()(T val) { return std::min(maxVal, std::max(minVal, val)); }
    T minVal, maxVal;
};
}