#include "stdafx.h"

#include "../include/sc/Axes.h"

using namespace StepperControl;
using namespace testing;

namespace {

using Af2 = Axes<float, 2>;
using Af3 = Axes<float, 3>;

struct Axes_Should : Test {};

TEST_F(Axes_Should, initialized_from_list) {
    Af3 a = {1, 2, 3};

    EXPECT_THAT(a, ElementsAre(1.f, 2.f, 3.f));
}

TEST_F(Axes_Should, initialze_missing_elements_to_zero) {
    Af3 a = {1};

    EXPECT_THAT(a, ElementsAre(1.f, 0.f, 0.f));
}

TEST_F(Axes_Should, initialze_elements_by_inf) {
    Af3 a = {1, inf(), 2};

    EXPECT_THAT(a, ElementsAre(1.f, inf(), 2.f));
}

TEST_F(Axes_Should, add) {
    Af2 a = {1, 2};
    Af2 b = {10, 20};
    a += b;

    EXPECT_THAT(a + Af2({100, 200}), ElementsAre(111.f, 222.f));
    EXPECT_THAT(Af2({100, 200}) + a, ElementsAre(111.f, 222.f));
    EXPECT_THAT(a, ElementsAre(11.f, 22.f));
    EXPECT_THAT(b, ElementsAre(10.f, 20.f));
}

TEST_F(Axes_Should, subtract) {
    Af2 a = {10, 20};
    Af2 b = {1, 2};
    a -= b;

    EXPECT_THAT(a - Af2({1, 1}), ElementsAre(8.f, 17.f));
    EXPECT_THAT(Af2({8, 17}) - a, ElementsAre(-1.f, -1.f));
    EXPECT_THAT(a, ElementsAre(9.f, 18.f));
    EXPECT_THAT(b, ElementsAre(1.f, 2.f));
}

TEST_F(Axes_Should, multiply_elementwise) {
    Af2 a = {10, 20};
    Af2 b = {2, 3};
    a *= b;

    EXPECT_THAT(a * Af2({2, 3}), ElementsAre(40.f, 180.f));
    EXPECT_THAT(Af2({2, 3}) * a, ElementsAre(40.f, 180.f));
    EXPECT_THAT(a, ElementsAre(20.f, 60.f));
    EXPECT_THAT(b, ElementsAre(2.f, 3.f));
}

TEST_F(Axes_Should, divide_elementwise) {
    Af2 a = {10, 20};
    Af2 b = {2, 4};
    a /= b;

    EXPECT_THAT(a / Af2({5, 5}), ElementsAre(1.f, 1.f));
    EXPECT_THAT(Af2({20, 30}) / a, ElementsAre(4.f, 6.f));
    EXPECT_THAT(a, ElementsAre(5.f, 5.f));
    EXPECT_THAT(b, ElementsAre(2.f, 4.f));
}

TEST_F(Axes_Should, cast_elements_to_another_type) {
    Af2 a = {10.2f, 20.6f};
    auto b = cast<int>(a);

    EXPECT_THAT(a, ElementsAre(10.2f, 20.6f));
    EXPECT_THAT(b, ElementsAre(10, 20));
}

TEST_F(Axes_Should, calculate_norm) {
    Af2 a = {3.f, 4.f};

    EXPECT_THAT(normSqr(a), FloatEq(25.f));
    EXPECT_THAT(norm(a), FloatEq(5.f));
}

}