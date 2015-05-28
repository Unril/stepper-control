#pragma once

#include <array>
#include <cmath>

namespace StepperControl {

template <typename T, size_t Size>
using Axes = std::array<T, Size>;

inline float inf() { return std::numeric_limits<float>::infinity(); }

template <typename T, size_t Size>
inline Axes<T, Size> &operator+=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] += b[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator+(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a += b;
}

template <typename T, size_t Size>
inline Axes<T, Size> &operator-=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] -= b[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator-(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a -= b;
}

template <typename T, size_t Size>
inline Axes<T, Size> &operator*=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] *= b[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator*(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a *= b;
}

template <typename T, size_t Size>
inline Axes<T, Size> &operator/=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] /= b[i];
    }
    return a;
}

template <typename T, size_t Size>
inline Axes<T, Size> operator/(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a /= b;
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
}
