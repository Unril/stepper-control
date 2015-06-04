#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace StepperControl {

#ifdef NDEBUG

#define scAssert(_Expression) ((void)0)
#define scExecute(_Expression) ((void)0)

#else /* NDEBUG */

#define scAssert(_Expression)                                                                      \
    do {                                                                                           \
        if (!(_Expression))                                                                        \
            throw std::logic_error((__FILE__ "(") + std::to_string(__LINE__) +                     \
                                   ("): " #_Expression " "));                                      \
    } while (false)

#define scExecute(_Expression) do { _Expression ; } while(false)

#endif /* NDEBUG */

// Axes

template <typename T, size_t Size>
using Axes = std::array<T, Size>;

// Addition

template <typename T, size_t Size>
inline Axes<T, Size> &operator+=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] += b[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> &operator+=(Axes<T, Size> &a, T b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] += b;
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator+(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a += b;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator+(Axes<T, Size> a, T b) {
    return a += b;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator+(T a, Axes<T, Size> b) {
    return b += a;
}

// Subtraction and negation

template <typename T, size_t Size>
inline Axes<T, Size> operator-(Axes<T, Size> a) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = -a[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> &operator-=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] -= b[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> &operator-=(Axes<T, Size> &a, T b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] -= b;
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator-(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a -= b;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator-(Axes<T, Size> a, T b) {
    return a -= b;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator-(T a, Axes<T, Size> b) {
    return (-b) += a;
}

// Multiplication

template <typename T, size_t Size>
inline Axes<T, Size> &operator*=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] *= b[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> &operator*=(Axes<T, Size> &a, T b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] *= b;
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator*(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a *= b;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator*(Axes<T, Size> a, T b) {
    return a *= b;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator*(T a, Axes<T, Size> b) {
    return b *= a;
}

// Division

template <typename T, size_t Size>
inline Axes<T, Size> &operator/=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] /= b[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> &operator/=(Axes<T, Size> &a, T b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] /= b;
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator/(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a /= b;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator/(Axes<T, Size> a, T b) {
    return a /= b;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator/(T a, Axes<T, Size> b) {
    for (size_t i = 0; i < Size; ++i) {
        b[i] = a / b[i];
    }
    return b;
}

// Relation

// less than
template <typename T, size_t Size>
inline Axes<bool, Size> lt(Axes<T, Size> a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] < b[i];
    }
    return res;
}

// less or equal than
template <typename T, size_t Size>
inline Axes<bool, Size> le(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] <= b[i];
    }
    return res;
}

// greater than
template <typename T, size_t Size>
inline Axes<bool, Size> gt(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] > b[i];
    }
    return res;
}

// greater or equal than
template <typename T, size_t Size>
inline Axes<bool, Size> ge(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] >= b[i];
    }
    return res;
}

// equal to
template <typename T, size_t Size>
inline Axes<bool, Size> eq(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] == b[i];
    }
    return res;
}

// not equal to
template <typename T, size_t Size>
inline Axes<bool, Size> neq(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] != b[i];
    }
    return res;
}

// negate
template <size_t Size>
inline Axes<bool, Size> not(Axes<bool, Size> a) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = !a[i];
    }
    return a;
}

// are all true
template <size_t Size>
inline bool all(Axes<bool, Size> const &a) {
    for (size_t i = 0; i < Size; ++i) {
        if (!a[i]) {
            return false;
        }
    }
    return true;
}

// is any true
template <size_t Size>
inline bool any(Axes<bool, Size> const &a) {
    for (size_t i = 0; i < Size; ++i) {
        if (a[i]) {
            return true;
        }
    }
    return false;
}

// Other

inline float inf() { return std::numeric_limits<float>::infinity(); }

template <size_t Size, typename T>
Axes<T, Size> inline axesConstant(T v) {
    Axes<T, Size> a;
    a.fill(v);
    return a;
}

template <typename M, typename T, size_t Size>
inline Axes<M, Size> cast(Axes<T, Size> const &b) {
    Axes<M, Size> a;
    for (size_t i = 0; i < Size; ++i) {
        a[i] = static_cast<M>(b[i]);
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> axesAbs(Axes<T, Size> a) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = abs(a[i]);
    }
    return a;
}

template <typename T, size_t Size>
inline T normSqr(Axes<T, Size> const &a) {
    T n{};
    for (size_t i = 0; i < Size; ++i) {
        n += a[i] * a[i];
    }
    return n;
}

template <typename T, size_t Size>
inline T norm(Axes<T, Size> const &a) {
    return std::sqrt(normSqr(a));
}

template <typename T, size_t Size, typename F>
inline T accumulate(Axes<T, Size> const &a, F binaryFunc, T init = T{}) {
    for (size_t i = 0; i < Size; ++i) {
        init = binaryFunc(a[i], init);
    }
    return init;
}

template <typename T, size_t Size, typename F>
inline Axes<T, Size> &applyInplace(Axes<T, Size> &a, F unaryFunc) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = unaryFunc(a[i]);
    }
    return a;
}

template <typename T, size_t Size, typename F>
inline Axes<T, Size> apply(Axes<T, Size> a, F unaryFunc) {
    return applyInplace(a, unaryFunc);
}

template <typename TFrom, typename TTo, size_t Size, typename F>
inline void tansformOnlyFinite(Axes<TFrom, Size> const &src, Axes<TTo, Size> *dest, F unaryFunc) {
    for (size_t i = 0; i < Size; ++i) {
        if (std::isfinite(src[i])) {
            (*dest)[i] = unaryFunc(src[i]);
        }
    }
}

template <typename T, size_t Size>
inline void copyOnlyFinite(Axes<T, Size> const &src, Axes<T, Size> *dest) {
    tansformOnlyFinite(src, dest, [](T t) { return t; });
}
}

namespace std {
template <typename T, size_t Size>
inline std::ostream &operator<<(std::ostream &os, StepperControl::Axes<T, Size> const &obj) {
    for (auto a : obj) {
        os << a << " ";
    }
    return os;
}
}
