#include "stdafx.h"

#include "../include/sc/Axes.h"

using namespace StepperControl;
using namespace testing;

namespace {

using Af2 = Axes<float, 2>;
using Af3 = Axes<float, 3>;

struct Axes_Should : Test {};

TEST_F(Axes_Should, initialized_from_list) {
    auto a = Af3{1, 2, 3};

    EXPECT_THAT(a, ElementsAre(1.f, 2.f, 3.f));
}

TEST_F(Axes_Should, initialze_missing_elements_to_zero) {
    auto a = Af3{1};

    EXPECT_THAT(a, ElementsAre(1.f, 0.f, 0.f));
}

TEST_F(Axes_Should, initialze_elements_by_inf) {
    auto a = Af3{1, inf(), 2};

    EXPECT_THAT(a, ElementsAre(1.f, inf(), 2.f));
}

TEST_F(Axes_Should, add) {
    auto a = Af2{1, 2};
    auto b = Af2{10, 20};
    a += b;

    EXPECT_THAT(a + Af2({100, 200}), ElementsAre(111.f, 222.f));
    EXPECT_THAT(Af2({100, 200}) + a, ElementsAre(111.f, 222.f));
    EXPECT_THAT(a, ElementsAre(11.f, 22.f));
    EXPECT_THAT(b, ElementsAre(10.f, 20.f));
}

TEST_F(Axes_Should, add_value) {
    auto a = Af2{1, 2};
    auto b = 1.f;
    a += b;

    EXPECT_THAT(a + 1.f, ElementsAre(3.f, 4.f));
    EXPECT_THAT(1.f + a, ElementsAre(3.f, 4.f));
    EXPECT_THAT(a, ElementsAre(2.f, 3.f));
    EXPECT_THAT(b, Eq(1.f));
}

TEST_F(Axes_Should, subtract) {
    auto a = Af2{10, 20};
    auto b = Af2{1, 2};
    a -= b;

    EXPECT_THAT(a - Af2({1, 1}), ElementsAre(8.f, 17.f));
    EXPECT_THAT(Af2({8, 17}) - a, ElementsAre(-1.f, -1.f));
    EXPECT_THAT(a, ElementsAre(9.f, 18.f));
    EXPECT_THAT(b, ElementsAre(1.f, 2.f));
}

TEST_F(Axes_Should, negate) {
    auto a = Af2{10, 20};
    auto b = -a;

    EXPECT_THAT(a, ElementsAre(10.f, 20.f));
    EXPECT_THAT(b, ElementsAre(-10.f, -20.f));
}

TEST_F(Axes_Should, subtract_value) {
    auto a = Af2{10, 20};
    auto b = 1.f;
    a -= b;

    EXPECT_THAT(a - 1.f, ElementsAre(8.f, 18.f));
    EXPECT_THAT(1.f - a, ElementsAre(-8.f, -18.f));
    EXPECT_THAT(a, ElementsAre(9.f, 19.f));
    EXPECT_THAT(b, Eq(1.f));
}

TEST_F(Axes_Should, multiply_elementwise) {
    auto a = Af2{10, 20};
    auto b = Af2{2, 3};
    a *= b;

    EXPECT_THAT(a * Af2({2, 3}), ElementsAre(40.f, 180.f));
    EXPECT_THAT(Af2({2, 3}) * a, ElementsAre(40.f, 180.f));
    EXPECT_THAT(a, ElementsAre(20.f, 60.f));
    EXPECT_THAT(b, ElementsAre(2.f, 3.f));
}

TEST_F(Axes_Should, multiply_on_value) {
    auto a = Af2{10, 20};
    auto b = 2.f;
    a *= b;

    EXPECT_THAT(a * 2.f, ElementsAre(40.f, 80.f));
    EXPECT_THAT(2.f * a, ElementsAre(40.f, 80.f));
    EXPECT_THAT(a, ElementsAre(20.f, 40.f));
    EXPECT_THAT(b, Eq(2.f));
}

TEST_F(Axes_Should, divide_elementwise) {
    auto a = Af2{10, 20};
    auto b = Af2{2, 4};
    a /= b;

    EXPECT_THAT(a / Af2({5, 5}), ElementsAre(1.f, 1.f));
    EXPECT_THAT(Af2({20, 30}) / a, ElementsAre(4.f, 6.f));
    EXPECT_THAT(a, ElementsAre(5.f, 5.f));
    EXPECT_THAT(b, ElementsAre(2.f, 4.f));
}

TEST_F(Axes_Should, divide_on_value) {
    auto a = Af2{20, 40};
    auto b = 2.f;
    a /= b;

    EXPECT_THAT(a / 2.f, ElementsAre(5.f, 10.f));
    EXPECT_THAT(20.f / a, ElementsAre(2.f, 1.f));
    EXPECT_THAT(a, ElementsAre(10.f, 20.f));
    EXPECT_THAT(b, Eq(2.f));
}

TEST_F(Axes_Should, cast_elements_to_another_type) {
    auto a = Af2{10.2f, 20.6f};
    auto b = axCast<int>(a);

    EXPECT_THAT(a, ElementsAre(10.2f, 20.6f));
    EXPECT_THAT(b, ElementsAre(10, 20));
}

TEST_F(Axes_Should, calculate_norm) {
    auto a = Af2{3.f, 4.f};

    EXPECT_THAT(normSqr(a), FloatEq(25.f));
    EXPECT_THAT(norm(a), FloatEq(5.f));
}

TEST_F(Axes_Should, apply_function) {
    auto a = Af2{3.f, 4.f};
    applyInplace(a, [](float v){return v*10;});

    EXPECT_THAT(apply(a, [](float v){return v*2;}), ElementsAre(60.f, 80.f));
    EXPECT_THAT(a, ElementsAre(30.f, 40.f));
}

TEST_F(Axes_Should, copy_finite_axes) {
    auto a = Af2{3.f, inf()};
    auto b = Af2{10.f, 10.f};
    copyOnlyFinite(a, b);

    EXPECT_THAT(a, ElementsAre(3.f, inf()));
    EXPECT_THAT(b, ElementsAre(3.f, 10.f));
}

TEST_F(Axes_Should, accumulate) {
    auto a = Af2{3.f, 4.f};
    auto b = accumulate(a, [](float curr, float acc) { return curr + acc;}, 1.f);

    EXPECT_THAT(a, ElementsAre(3.f, 4.f));
    EXPECT_THAT(b, Eq(8.f));
}

}