#pragma once

#include <array>
#include <cmath>

namespace StepperControl {
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
inline T accumulate(Axes<T, Size> const&a, F binaryFunc, T init = T{}) {
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

template <typename T, size_t Size>
inline std::ostream &operator<<(std::ostream &os, Axes<T, Size> const &obj) {
    for (auto a : obj) {
        os << a << " ";
    }
    return os;
}

}
