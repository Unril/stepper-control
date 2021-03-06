#pragma once

#include "Common.h"

#include <array>
#include <cmath>
#include <cstdint>

namespace StepperControl {

template <typename T, size_t Size>
using Axes = std::array<T, Size>;

// Addition

template <typename T, size_t Size>
Axes<T, Size> &operator+=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] += b[i];
    }
    return a;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> &operator+=(Axes<T, Size> &a, B b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] += b;
    }
    return a;
}

template <typename T, size_t Size>
Axes<T, Size> operator+(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a += b;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> operator+(Axes<T, Size> a, B b) {
    return a += b;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> operator+(B a, Axes<T, Size> b) {
    return b += a;
}

// Subtraction and negation

template <typename T, size_t Size>
Axes<T, Size> operator-(Axes<T, Size> a) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = -a[i];
    }
    return a;
}

template <typename T, size_t Size>
Axes<T, Size> &operator-=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] -= b[i];
    }
    return a;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> &operator-=(Axes<T, Size> &a, B b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] -= b;
    }
    return a;
}

template <typename T, size_t Size>
Axes<T, Size> operator-(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a -= b;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> operator-(Axes<T, Size> a, B b) {
    return a -= b;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> operator-(B a, Axes<T, Size> const &b) {
    Axes<T, Size> mb = -b;
    return mb += a;
}

// Multiplication

template <typename T, size_t Size>
Axes<T, Size> &operator*=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] *= b[i];
    }
    return a;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> &operator*=(Axes<T, Size> &a, B b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] *= b;
    }
    return a;
}

template <typename T, size_t Size>
Axes<T, Size> operator*(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a *= b;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> operator*(Axes<T, Size> a, B b) {
    return a *= b;
}

template <typename T, size_t Size, typename B>
Axes<T, Size> operator*(B a, Axes<T, Size> b) {
    return b *= a;
}

// Division

template <typename T, size_t Size>
Axes<T, Size> &operator/=(Axes<T, Size> &a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] /= b[i];
    }
    return a;
}

template <typename T, size_t Size>
Axes<T, Size> &operator/=(Axes<T, Size> &a, T b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] /= b;
    }
    return a;
}

template <typename T, size_t Size>
Axes<T, Size> operator/(Axes<T, Size> a, Axes<T, Size> const &b) {
    return a /= b;
}

template <typename T, size_t Size>
Axes<T, Size> operator/(Axes<T, Size> a, T b) {
    return a /= b;
}

template <typename T, size_t Size>
Axes<T, Size> operator/(T a, Axes<T, Size> b) {
    for (size_t i = 0; i < Size; ++i) {
        b[i] = a / b[i];
    }
    return b;
}

// Relation

// less than
template <typename T, size_t Size>
Axes<bool, Size> lt(Axes<T, Size> a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] < b[i];
    }
    return res;
}

// less or equal than
template <typename T, size_t Size>
Axes<bool, Size> le(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] <= b[i];
    }
    return res;
}

// greater than
template <typename T, size_t Size>
Axes<bool, Size> gt(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] > b[i];
    }
    return res;
}

// greater or equal than
template <typename T, size_t Size>
Axes<bool, Size> ge(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = a[i] >= b[i];
    }
    return res;
}

// equal to
template <typename T, size_t Size>
Axes<bool, Size> eq(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = (a[i] == b[i]);
    }
    return res;
}

template <typename T, size_t Size, typename B>
Axes<bool, Size> eq(Axes<T, Size> const &a, B b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = (a[i] == b);
    }
    return res;
}

// not equal to
template <typename T, size_t Size>
Axes<bool, Size> neq(Axes<T, Size> const &a, Axes<T, Size> const &b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = (a[i] != b[i]);
    }
    return res;
}

template <typename T, size_t Size, typename B>
Axes<bool, Size> neq(Axes<T, Size> const &a, B b) {
    Axes<bool, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = (a[i] != b);
    }
    return res;
}

// negate
template <size_t Size>
Axes<bool, Size> negate(Axes<bool, Size> a) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = !a[i];
    }
    return a;
}

// are all true
template <size_t Size>
bool all(Axes<bool, Size> const &a) {
    for (size_t i = 0; i < Size; ++i) {
        if (!a[i]) {
            return false;
        }
    }
    return true;
}

// is any true
template <size_t Size>
bool any(Axes<bool, Size> const &a) {
    for (size_t i = 0; i < Size; ++i) {
        if (a[i]) {
            return true;
        }
    }
    return false;
}

// Other

template <size_t Size, typename T>
Axes<T, Size> axConst(T v) {
    Axes<T, Size> a;
    a.fill(v);
    return a;
}

template <typename Ax, typename T>
Ax axConst(T v) {
    Ax a;
    a.fill(static_cast<typename Ax::value_type>(v));
    return a;
}

template <typename Ax>
Ax axZero() {
    return axConst<Ax>(0);
}

template <typename Ax>
Ax axInf() {
    return axConst<Ax>(inf());
}

template <typename M, typename T, size_t Size>
Axes<M, Size> axCast(Axes<T, Size> const &b) {
    Axes<M, Size> a;
    for (size_t i = 0; i < Size; ++i) {
        a[i] = static_cast<M>(b[i]);
    }
    return a;
}

template <typename T, size_t Size>
Axes<T, Size> axAbs(Axes<T, Size> a) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = abs(a[i]);
    }
    return a;
}

template <typename T, size_t Size>
T axMax(Axes<T, Size> const &a) {
    auto m = a[0];
    for (size_t i = 1; i < Size; ++i) {
        m = std::max(m, a[i]);
    }
    return m;
}

template <typename T, size_t Size>
T axMax(Axes<T, Size> a, Axes<T, Size> const &b) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = std::max(a[i], b[i]);
    }
    return a;
}

template <typename T, size_t Size>
Axes<T, Size> axRound(Axes<T, Size> a) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = round(a[i]);
    }
    return a;
}

template <typename T, size_t Size>
Axes<int32_t, Size> axLRound(Axes<T, Size> a) {
    Axes<int32_t, Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = static_cast<int32_t>(lround(a[i]));
    }
    return res;
}

template <typename T, size_t Size>
T normSqr(Axes<T, Size> const &a) {
    T n{};
    for (size_t i = 0; i < Size; ++i) {
        n += a[i] * a[i];
    }
    return n;
}

template <typename T, size_t Size>
T norm(Axes<T, Size> const &a) {
    return std::sqrt(normSqr(a));
}

template <typename T, size_t Size, typename F>
T accumulate(Axes<T, Size> const &a, F binaryFunc, T init) {
    for (size_t i = 0; i < Size; ++i) {
        init = binaryFunc(a[i], init);
    }
    return init;
}

template <typename T, size_t Size, typename F>
Axes<T, Size> &applyInplace(Axes<T, Size> &a, F unaryFunc) {
    for (size_t i = 0; i < Size; ++i) {
        a[i] = unaryFunc(a[i]);
    }
    return a;
}

template <typename T, size_t Size, typename F>
auto apply(Axes<T, Size> a, F unaryFunc) -> Axes<decltype(unaryFunc(T{})), Size> {
    Axes<decltype(unaryFunc(T{})), Size> res;
    for (size_t i = 0; i < Size; ++i) {
        res[i] = unaryFunc(a[i]);
    }
    return res;
}

template <typename TFrom, typename TTo, size_t Size, typename F>
void tansformOnlyFinite(Axes<TFrom, Size> const &src, Axes<TTo, Size> &dest, F unaryFunc) {
    for (size_t i = 0; i < Size; ++i) {
        if (std::isfinite(src[i])) {
            dest[i] = unaryFunc(src[i]);
        }
    }
}

template <typename T, size_t Size>
void copyOnlyFinite(Axes<T, Size> const &src, Axes<T, Size> &dest) {
    for (size_t i = 0; i < Size; ++i) {
        if (std::isfinite(src[i])) {
            dest[i] = src[i];
        }
    }
}

template <typename T, size_t Size>
Printer &operator<<(Printer &p, Axes<T, Size> const &a) {
    p.print(a.data(), Size);
    return p;
}

template <size_t size>
using TAf = Axes<float, size>;

template <size_t size>
using TAi = Axes<int32_t, size>;

struct DefaultAxesTraits {
    static const unsigned size = 9;
    static const char *names() { return "ABCUVWXYZ"; }
};
}

#ifndef __MBED__
namespace std {
template <typename T, size_t Size>
std::ostream &operator<<(std::ostream &os, StepperControl::Axes<T, Size> const &a) {
    for (size_t i = 0; i < Size - 1; ++i) {
        os << a[i] << ", ";
    }
    return os << a[Size - 1];
}
}
#endif
