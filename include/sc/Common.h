#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#ifdef __MBED__

#define FORCE_INLINE __attribute__((always_inline))
#define RESTRICT __restrict__
#define MEM_BARRIER() asm volatile("" : : : "memory")

#elif defined(_MSC_VER)

#define FORCE_INLINE __forceinline
#define RESTRICT __restrict
#define MEM_BARRIER()

inline void wait_us(int){};

#else
#error "Compiler isn't supported!"
#endif

#ifdef NDEBUG

#define scAssert(_Expression) ((void)0)
#define scExecute(_Expression) ((void)0)

#elif __MBED__

#define scAssert(_Expression) MBED_ASSERT(_Expression)

#else

#include <string>
#include <stdexcept>

#define scAssert(_Expression)                                                                      \
    do {                                                                                           \
        if (!(_Expression))                                                                        \
            throw std::logic_error((__FILE__ "(") + std::to_string(__LINE__) +                     \
                                   ("): " #_Expression " "));                                      \
    } while (false)

#endif /* NDEBUG */

namespace StepperControl {

inline auto lTruncTowardZero(float v) -> int32_t { return static_cast<int32_t>(v); }

inline auto lTruncTowardInf(float v) -> int32_t {
    return static_cast<int32_t>(v < 0.0f ? floor(v) : ceil(v));
}

constexpr auto inf() -> float { return std::numeric_limits<float>::infinity(); }

constexpr const auto *eol = "\r\n";

inline const char *sep(int i, int size) { return i == size - 1 ? "" : ", "; }

struct Printer {
    virtual ~Printer() {}
    virtual void print(const int32_t *n, int size) = 0;
    virtual void print(const float *n, int size) = 0;
    virtual void print(const char *str) = 0;

    static Printer *instance() {
        struct Default : Printer {
            void print(const float *n, int size) override {
                for (int i = 0; i < size; i++) {
                    printf("%f%s", n[i], sep(i, size));
                }
            }
            void print(const int32_t *n, int size) override {
                for (int i = 0; i < size; i++) {
                    printf("%ld%s", n[i], sep(i, size));
                }
            }
            void print(const char *str) override { printf("%s", str); }
        };
        static Default p;
        return &p;
    }
};

template <typename T>
auto operator<<(Printer &p, T n) ->
    typename std::enable_if<std::is_integral<T>::value, Printer &>::type {
    auto val = static_cast<int32_t>(n);
    p.print(&val, 1);
    return p;
}

inline auto operator<<(Printer &p, float n) -> Printer & {
    p.print(&n, 1);
    return p;
}

inline auto operator<<(Printer &p, const char *str) -> Printer & {
    p.print(str);
    return p;
}

enum class DistanceMode { Absolute, Relative };
}
