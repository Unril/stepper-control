#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <algorithm>
#include <exception>

namespace StepperControl {

#ifdef __MBED__
#define FORCE_INLINE __attribute__((always_inline))
#elif defined(_MSC_VER)
#define FORCE_INLINE __forceinline
#else
#define FORCE_INLINE inline
#endif

#ifdef NDEBUG

#define scAssert(_Expression) ((void)0)
#define scExecute(_Expression) ((void)0)

#elif __MBED__

#define scAssert(_Expression) MBED_ASSERT(_Expression)

#else

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

inline float inf() { return std::numeric_limits<float>::infinity(); }

template <typename T>
struct Clamp {
    Clamp(T minV, T maxV) : minVal(minV), maxVal(maxV) {}
    T operator()(T val) { return std::min(maxVal, std::max(minVal, val)); }
    T minVal, maxVal;
};

template <unsigned i>
struct UIntConst {};

const char eol = '\n';

struct Printer {
    virtual ~Printer() {}
    virtual void print(int n) = 0;
    virtual void print(char n) = 0;
    virtual void print(float n) = 0;
    virtual void print(const char *str) = 0;

    static Printer *instance() {
        struct Default : Printer {
            void print(int n) override { printf("%d", n); }
            void print(char n) override { putchar(n); }
            void print(float n) override { printf("%f", n); }
            void print(const char *str) override { printf("%s", str); }
        };
        static Default p;
        return &p;
    }
};

template <typename T, typename = std::enable_if<std::is_integral<T>::value>>
inline Printer &operator<<(Printer &p, T n) {
    p.print(static_cast<int>(n));
    return p;
}
inline Printer &operator<<(Printer &p, char n) {
    p.print(n);
    return p;
}
inline Printer &operator<<(Printer &p, float n) {
    p.print(n);
    return p;
}
inline Printer &operator<<(Printer &p, const char *str) {
    p.print(str);
    return p;
}
}
