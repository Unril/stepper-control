#include "stdafx.h"

#include "../include/sc/StepperControl.h"

using namespace StepperControl;
using namespace testing;
using namespace std;

namespace {

const size_t AxesSize = 1;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct SegmentsGenerator_Should : Test {

    using SegGen = SegmentsGenerator<AxesSize>;
    vector<Ai> path_;
    SegGen gen_;
};

TEST_F(SegmentsGenerator_Should, generate_one_linear_segment) {
    gen_.setPath({Ai{0}, Ai{100}});
    gen_.setDurations({100.f});
    gen_.setBlendDurations({0.f, 0.f});

    gen_.update();

    // ASSERT_THAT(gen_.segments(), SizeIs(1));
    // auto s0 = gen_.segments().front();
    // EXPECT_THAT(s0, Eq(0));
}

template <size_t AxesSize>
struct MotorMock {
    using Ai = Axes<int32_t, AxesSize>;

    struct Step {
        Step(Ai const &x, Ai const &step) : x(x), step(step) {}

        friend bool operator==(Step const &lhs, Step const &rhs) {
            return lhs.x == rhs.x && lhs.step == rhs.step;
        }

        friend bool operator!=(Step const &lhs, Step const &rhs) { return !(lhs == rhs); }

        friend std::ostream &operator<<(std::ostream &os, Step const &obj) {
            return os << endl << "x: " << obj.x << " step: " << obj.step;
        }

        Ai x;
        Ai step;
    };

    void write(Ai x, Ai step) { data.emplace_back(x, step); }

    vector<Step> data;
};

struct TickerMock {
    void attach_us(...) {}
    void detach() {}
};

struct SegmentsExecutor_Should : Test {
    using Exec = SegmentsExecutor<1, MotorMock, TickerMock>;
    using Sg = Segment<1>;
    using Mm = MotorMock<1>;
    Mm mm_;
    TickerMock t_;
    Exec::Segments segments_;
    Exec exec_{&mm_, &t_};

    void process() {
        exec_.setSegments(segments_);
        exec_.start();
        while (exec_.running()) {
            exec_.tick();
        }
    }
};

TEST_F(SegmentsExecutor_Should, execute_one_linear_segment) {
    segments_.push_back(Sg(10, {5}));
    process();

    vector<Mm::Step> expected{
        {{0}, {0}},
        {{1}, {1}},
        {{1}, {0}},
        {{2}, {1}},
        {{2}, {0}},
        {{3}, {1}},
        {{3}, {0}},
        {{4}, {1}},
        {{4}, {0}},
        {{5}, {1}},
        {{5}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_linear_segment_from_negative_position) {
    exec_.setPosition({-5});
    segments_.push_back(Sg(10, {5}));
    process();

    vector<Mm::Step> expected{
        {{-5}, {0}},
        {{-4}, {1}},
        {{-4}, {0}},
        {{-3}, {1}},
        {{-3}, {0}},
        {{-2}, {1}},
        {{-2}, {0}},
        {{-1}, {1}},
        {{-1}, {0}},
        {{0}, {1}},
        {{0}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_linear_segment_with_negative_slope) {
    exec_.setPosition({5});
    segments_.push_back(Sg(10, {-5}));
    process();

    vector<Mm::Step> expected{
        {{5}, {0}},
        {{4}, {-1}},
        {{4}, {0}},
        {{3}, {-1}},
        {{3}, {0}},
        {{2}, {-1}},
        {{2}, {0}},
        {{1}, {-1}},
        {{1}, {0}},
        {{0}, {-1}},
        {{0}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_short_linear_segment) {
    segments_.push_back(Sg(2, {1}));
    process();

    vector<Mm::Step> expected{
        /**/ {{0}, {0}},
        /**/ {{1}, {1}},
        /**/ {{1}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_horizontal_linear_segment) {
    segments_.push_back({Sg(4, {0})});
    process();

    vector<Mm::Step> expected{
        /**/ {{0}, {0}},
        /**/ {{0}, {0}},
        /**/ {{0}, {0}},
        /**/ {{0}, {0}},
        /**/ {{0}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_linear_segment_with_slow_negative_slope) {
    segments_.push_back(Sg(15, {-5}));
    process();

    vector<Mm::Step> expected{
        {{0}, {0}},
        {{0}, {0}},
        {{-1}, {-1}},
        {{-1}, {0}},
        {{-1}, {0}},
        {{-2}, {-1}},
        {{-2}, {0}},
        {{-2}, {0}},
        {{-3}, {-1}},
        {{-3}, {0}},
        {{-3}, {0}},
        {{-4}, {-1}},
        {{-4}, {0}},
        {{-4}, {0}},
        {{-5}, {-1}},
        {{-5}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_rising_parabolic_segment) {
    segments_.push_back(Sg(20, {5}, {0}));
    process();

    vector<Mm::Step> expected{
        {{0}, {0}},
        {{0}, {0}},
        {{1}, {1}},
        {{1}, {0}},
        {{2}, {1}},
        {{2}, {0}},
        {{3}, {1}},
        {{3}, {0}},
        {{3}, {0}},
        {{3}, {0}},
        {{4}, {1}},
        {{4}, {0}},
        {{4}, {0}},
        {{4}, {0}},
        {{5}, {1}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_falling_parabolic_segment) {
    exec_.setPosition({5});
    segments_.push_back(Sg(20, {0}, {-5}));
    process();

    vector<Mm::Step> expected{
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{4}, {-1}},
        {{4}, {0}},
        {{4}, {0}},
        {{4}, {0}},
        {{3}, {-1}},
        {{3}, {0}},
        {{3}, {0}},
        {{3}, {0}},
        {{2}, {-1}},
        {{2}, {0}},
        {{1}, {-1}},
        {{1}, {0}},
        {{0}, {-1}},
        {{0}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_rising_parabolic_segment_with_negative_curvature) {
    segments_.push_back(Sg(20, {-5}, {0}));
    process();

    vector<Mm::Step> expected{
        {{0}, {0}},
        {{0}, {0}},
        {{-1}, {-1}},
        {{-1}, {0}},
        {{-2}, {-1}},
        {{-2}, {0}},
        {{-3}, {-1}},
        {{-3}, {0}},
        {{-3}, {0}},
        {{-3}, {0}},
        {{-4}, {-1}},
        {{-4}, {0}},
        {{-4}, {0}},
        {{-4}, {0}},
        {{-5}, {-1}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_falling_parabolic_segment_with_negative_curvature) {
    exec_.setPosition({-5});
    segments_.push_back(Sg(20, {0}, {5}));
    process();

    vector<Mm::Step> expected{
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-5}, {0}},
        {{-4}, {1}},
        {{-4}, {0}},
        {{-4}, {0}},
        {{-4}, {0}},
        {{-3}, {1}},
        {{-3}, {0}},
        {{-3}, {0}},
        {{-3}, {0}},
        {{-2}, {1}},
        {{-2}, {0}},
        {{-1}, {1}},
        {{-1}, {0}},
        {{-0}, {1}},
        {{-0}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_short_parabolic_segment_) {
    segments_.push_back(Sg(4, {1}, {0}));
    process();

    vector<Mm::Step> expected{
        /**/ {{0}, {0}},
        /**/ {{0}, {0}},
        /**/ {{1}, {1}},
        /**/ {{1}, {0}},
        /**/ {{1}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, approximate_parabolic_curve_with_zero_curvature_as_line) {
    segments_.push_back(Sg(10, {2}, {2}));
    process();

    vector<Mm::Step> expected{
        {{0}, {0}},
        {{0}, {0}},
        {{1}, {1}},
        {{1}, {0}},
        {{2}, {1}},
        {{2}, {0}},
        {{2}, {0}},
        {{3}, {1}},
        {{3}, {0}},
        {{4}, {1}},
        {{4}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_two_linear_segments) {
    segments_.push_back(Sg(6, {3}));
    segments_.push_back(Sg(6, {-3}));
    process();
    vector<Mm::Step> expected{
        {{0}, {0}},
        {{1}, {1}},
        {{1}, {0}},
        {{2}, {1}},
        {{2}, {0}},
        {{3}, {1}},
        {{3}, {0}},
        {{2}, {-1}},
        {{2}, {0}},
        {{1}, {-1}},
        {{1}, {0}},
        {{0}, {-1}},
        {{0}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_parabolic_segment_with_gradient_change) {
    segments_.push_back(Sg(8, {2}, {-2}));
    process();
    vector<Mm::Step> expected{
        {{0}, {0}},
        {{0}, {0}},
        {{1}, {1}},
        {{1}, {0}},
        {{1}, {0}},
        {{1}, {0}},
        {{1}, {0}},
        {{0}, {-1}},
        {{0}, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_two_linear_segments_with_parabolic_blend) {
    segments_.push_back(Sg(8, {4}));
    segments_.push_back(Sg(8, {2}, {-2}));
    segments_.push_back(Sg(8, {-4}));
    process();
    vector<Mm::Step> expected{
        {{0}, {0}},
        {{1}, {1}},
        {{1}, {0}},
        {{2}, {1}},
        {{2}, {0}},
        {{3}, {1}},
        {{3}, {0}},
        {{4}, {1}},
        {{4}, {0}},
        {{4}, {0}},
        {{5}, {1}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{5}, {0}},
        {{4}, {-1}},
        {{4}, {0}},
        {{3}, {-1}},
        {{3}, {0}},
        {{2}, {-1}},
        {{2}, {0}},
        {{1}, {-1}},
        {{1}, {0}},
        {{0}, {-1}},
        {{0}, {0}},

    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

struct SegmentsExecutor2_Should : Test {
    using Exec = SegmentsExecutor<2, MotorMock, TickerMock>;
    using Sg = Segment<2>;
    using Mm = MotorMock<2>;
    Mm mm_;
    TickerMock t_;
    Exec::Segments segments_;
    Exec exec_{&mm_, &t_};

    void process() {
        exec_.setSegments(segments_);
        exec_.start();
        while (exec_.running()) {
            exec_.tick();
        }
    }
};

TEST_F(SegmentsExecutor2_Should, execute_one_linear_segment) {
    exec_.setPosition({0, 5});
    segments_.push_back(Sg(10, {5, -5}));
    process();

    vector<Mm::Step> expected{
        {{0, 5}, {0, 0}},
        {{1, 4}, {1, -1}},
        {{1, 4}, {0, 0}},
        {{2, 3}, {1, -1}},
        {{2, 3}, {0, 0}},
        {{3, 2}, {1, -1}},
        {{3, 2}, {0, 0}},
        {{4, 1}, {1, -1}},
        {{4, 1}, {0, 0}},
        {{5, 0}, {1, -1}},
        {{5, 0}, {0, 0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}
}