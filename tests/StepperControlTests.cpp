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
        Step(Ai const &x, int32_t t, Ai const &step) : x(x), t(t), step(step) {}

        friend bool operator==(Step const &lhs, Step const &rhs) {
            return lhs.x == rhs.x && lhs.t == rhs.t && lhs.step == rhs.step;
        }

        friend bool operator!=(Step const &lhs, Step const &rhs) { return !(lhs == rhs); }

        friend std::ostream &operator<<(std::ostream &os, Step const &obj) {
            return os << endl << "x: " << obj.x << " t: " << obj.t << " step: " << obj.step;
        }

        Ai x;
        int32_t t;
        Ai step;
    };

    void write(Ai x, int32_t t, Ai step) { data.emplace_back(x, t, step); }

    vector<Step> data;
};

struct SegmentsExecutor_Should : Test {
    using Exec = SegmentsExecutor<1, MotorMock>;
    using Sg = Segment<1>;
    using Mm = MotorMock<1>;
    Mm mm_;
    Exec::Segments segments_;
    Exec exec_;

    void process() {
        exec_.setSegments(segments_);
        exec_.t_ = &mm_;

        exec_.start();
        while (exec_.running()) {
            exec_.tick();
        }
    }
};

TEST_F(SegmentsExecutor_Should, execute_one_linear_segment) {
    segments_.push_back({Sg(0, {0}, 10, {5})});
    process();

    vector<Mm::Step> expected{
        {{0}, 0, {0}},
        {{1}, 1, {1}},
        {{1}, 2, {0}},
        {{2}, 3, {1}},
        {{2}, 4, {0}},
        {{3}, 5, {1}},
        {{3}, 6, {0}},
        {{4}, 7, {1}},
        {{4}, 8, {0}},
        {{5}, 9, {1}},
        {{5}, 10, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_linear_segment_from_negative_position) {
    segments_.push_back({Sg(0, {-5}, 10, {0})});
    process();

    vector<Mm::Step> expected{
        {{-5}, 0, {0}},
        {{-4}, 1, {1}},
        {{-4}, 2, {0}},
        {{-3}, 3, {1}},
        {{-3}, 4, {0}},
        {{-2}, 5, {1}},
        {{-2}, 6, {0}},
        {{-1}, 7, {1}},
        {{-1}, 8, {0}},
        {{0}, 9, {1}},
        {{0}, 10, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_linear_segment_with_negative_slope) {
    segments_.push_back({Sg(0, {5}, 10, {0})});
    process();

    vector<Mm::Step> expected{
        {{5}, 0, {0}},
        {{4}, 1, {-1}},
        {{4}, 2, {0}},
        {{3}, 3, {-1}},
        {{3}, 4, {0}},
        {{2}, 5, {-1}},
        {{2}, 6, {0}},
        {{1}, 7, {-1}},
        {{1}, 8, {0}},
        {{0}, 9, {-1}},
        {{0}, 10, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_short_linear_segment) {
    segments_.push_back({Sg(0, {0}, 2, {1})});
    process();

    vector<Mm::Step> expected{
        {{0}, 0, {0}}, {{1}, 1, {1}}, {{1}, 2, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_horizontal_linear_segment) {
    segments_.push_back({Sg(0, {0}, 4, {0})});
    process();

    vector<Mm::Step> expected{
        {{0}, 0, {0}}, {{0}, 1, {0}}, {{0}, 2, {0}}, {{0}, 3, {0}}, {{0}, 4, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_linear_segment_with_slow_negative_slope) {
    segments_.push_back({Sg(0, {0}, 15, {-5})});
    process();

    vector<Mm::Step> expected{
        {{0}, 0, {0}},
        {{0}, 1, {0}},
        {{-1}, 2, {-1}},
        {{-1}, 3, {0}},
        {{-1}, 4, {0}},
        {{-2}, 5, {-1}},
        {{-2}, 6, {0}},
        {{-2}, 7, {0}},
        {{-3}, 8, {-1}},
        {{-3}, 9, {0}},
        {{-3}, 10, {0}},
        {{-4}, 11, {-1}},
        {{-4}, 12, {0}},
        {{-4}, 13, {0}},
        {{-5}, 14, {-1}},
        {{-5}, 15, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_rising_parabolic_segment) {
    segments_.push_back({Sg(0, {0}, {5}, 20, {5})});
    process();

    vector<Mm::Step> expected{
        {{0}, 0, {0}},
        {{0}, 1, {0}},
        {{1}, 2, {1}},
        {{1}, 3, {0}},
        {{2}, 4, {1}},
        {{2}, 5, {0}},
        {{3}, 6, {1}},
        {{3}, 7, {0}},
        {{3}, 8, {0}},
        {{3}, 9, {0}},
        {{4}, 10, {1}},
        {{4}, 11, {0}},
        {{4}, 12, {0}},
        {{4}, 13, {0}},
        {{5}, 14, {1}},
        {{5}, 15, {0}},
        {{5}, 16, {0}},
        {{5}, 17, {0}},
        {{5}, 18, {0}},
        {{5}, 19, {0}},
        {{5}, 20, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_falling_parabolic_segment) {
    segments_.push_back({Sg(0, {5}, {5}, 20, {0})});
    process();

    vector<Mm::Step> expected{
        {{5}, 0, {0}},
        {{5}, 1, {0}},
        {{5}, 2, {0}},
        {{5}, 3, {0}},
        {{5}, 4, {0}},
        {{5}, 5, {0}},
        {{5}, 6, {0}},
        {{4}, 7, {-1}},
        {{4}, 8, {0}},
        {{4}, 9, {0}},
        {{4}, 10, {0}},
        {{3}, 11, {-1}},
        {{3}, 12, {0}},
        {{3}, 13, {0}},
        {{3}, 14, {0}},
        {{2}, 15, {-1}},
        {{2}, 16, {0}},
        {{1}, 17, {-1}},
        {{1}, 18, {0}},
        {{0}, 19, {-1}},
        {{0}, 20, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_rising_parabolic_segment_with_negative_curvature) {
    segments_.push_back({Sg(0, {0}, {-5}, 20, {-5})});
    process();

    vector<Mm::Step> expected{
        {{0}, 0, {0}},
        {{0}, 1, {0}},
        {{-1}, 2, {-1}},
        {{-1}, 3, {0}},
        {{-2}, 4, {-1}},
        {{-2}, 5, {0}},
        {{-3}, 6, {-1}},
        {{-3}, 7, {0}},
        {{-3}, 8, {0}},
        {{-3}, 9, {0}},
        {{-4}, 10, {-1}},
        {{-4}, 11, {0}},
        {{-4}, 12, {0}},
        {{-4}, 13, {0}},
        {{-5}, 14, {-1}},
        {{-5}, 15, {0}},
        {{-5}, 16, {0}},
        {{-5}, 17, {0}},
        {{-5}, 18, {0}},
        {{-5}, 19, {0}},
        {{-5}, 20, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_falling_parabolic_segment_with_negative_curvature) {
    segments_.push_back({Sg(0, {-5}, {-5}, 20, {0})});
    process();

    vector<Mm::Step> expected{
        {{-5}, 0, {0}},
        {{-5}, 1, {0}},
        {{-5}, 2, {0}},
        {{-5}, 3, {0}},
        {{-5}, 4, {0}},
        {{-5}, 5, {0}},
        {{-5}, 6, {0}},
        {{-4}, 7, {1}},
        {{-4}, 8, {0}},
        {{-4}, 9, {0}},
        {{-4}, 10, {0}},
        {{-3}, 11, {1}},
        {{-3}, 12, {0}},
        {{-3}, 13, {0}},
        {{-3}, 14, {0}},
        {{-2}, 15, {1}},
        {{-2}, 16, {0}},
        {{-1}, 17, {1}},
        {{-1}, 18, {0}},
        {{-0}, 19, {1}},
        {{-0}, 20, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_one_short_parabolic_segment_) {
    segments_.push_back({Sg(0, {0}, {1}, 4, {1})});
    process();

    vector<Mm::Step> expected{
        {{0}, 0, {0}}, {{0}, 1, {0}}, {{1}, 2, {1}}, {{1}, 3, {0}}, {{1}, 4, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, approximate_parabolic_curve_with_zero_curvature_as_line) {
    segments_.push_back({Sg(0, {0}, {2}, 10, {4})});
    process();

    vector<Mm::Step> expected{
        {{0}, 0, {0}},
        {{0}, 1, {0}},
        {{1}, 2, {1}},
        {{1}, 3, {0}},
        {{2}, 4, {1}},
        {{2}, 5, {0}},
        {{2}, 6, {0}},
        {{3}, 7, {1}},
        {{3}, 8, {0}},
        {{4}, 9, {1}},
        {{4}, 10, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_two_linear_segments) {
    segments_.push_back({Sg(0, {0}, 6, {3})});
    segments_.push_back({Sg(6, {3}, 12, {0})});
    process();
    vector<Mm::Step> expected{
        {{0}, 0, {0}},
        {{1}, 1, {1}},
        {{1}, 2, {0}},
        {{2}, 3, {1}},
        {{2}, 4, {0}},
        {{3}, 5, {1}},
        {{3}, 6, {0}},
        {{2}, 7, {-1}},
        {{2}, 8, {0}},
        {{1}, 9, {-1}},
        {{1}, 10, {0}},
        {{0}, 11, {-1}},
        {{0}, 12, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_parabolic_segment_with_gradient_change) {
    segments_.push_back({Sg(0, {0}, {2}, 8, {0})});
    process();
    vector<Mm::Step> expected{
        {{0}, 0, {0}},
        {{0}, 1, {0}},
        {{1}, 2, {1}},
        {{1}, 3, {0}},
        {{1}, 4, {0}},
        {{1}, 5, {0}},
        {{1}, 6, {0}},
        {{0}, 7, {-1}},
        {{0}, 8, {0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor_Should, execute_two_linear_segments_with_parabolic_blend) {
    segments_.push_back({Sg(0, {0}, 8, {4})});
    segments_.push_back({Sg(8, {4}, {6}, 16, {4})});
    segments_.push_back({Sg(16, {4}, 24, {0})});
    process();
    vector<Mm::Step> expected{
        {{0}, 0, {0}},
        {{1}, 1, {1}},
        {{1}, 2, {0}},
        {{2}, 3, {1}},
        {{2}, 4, {0}},
        {{3}, 5, {1}},
        {{3}, 6, {0}},
        {{4}, 7, {1}},
        {{4}, 8, {0}},
        {{4}, 9, {0}},
        {{5}, 10, {1}},
        {{5}, 11, {0}},
        {{5}, 12, {0}},
        {{5}, 13, {0}},
        {{5}, 14, {0}},
        {{4}, 15, {-1}},
        {{4}, 16, {0}},
        {{3}, 17, {-1}},
        {{3}, 18, {0}},
        {{2}, 19, {-1}},
        {{2}, 20, {0}},
        {{1}, 21, {-1}},
        {{1}, 22, {0}},
        {{0}, 23, {-1}},
        {{0}, 24, {0}},

    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}

struct SegmentsExecutor2_Should : Test {
    using Exec = SegmentsExecutor<2, MotorMock>;
    using Sg = Segment<2>;
    using Mm = MotorMock<2>;
    Mm mm_;
    Exec::Segments segments_;
    Exec exec_;

    void process() {
        exec_.setSegments(segments_);
        exec_.t_ = &mm_;

        exec_.start();
        while (exec_.running()) {
            exec_.tick();
        }
    }
};

TEST_F(SegmentsExecutor2_Should, execute_one_linear_segment) {
    segments_.push_back(Sg(0, {0, 5}, 10, {5, 0}));
    process();

    vector<Mm::Step> expected{
        {{0, 5}, 0, {0, 0}},
        {{1, 4}, 1, {1, -1}},
        {{1, 4}, 2, {0, 0}},
        {{2, 3}, 3, {1, -1}},
        {{2, 3}, 4, {0, 0}},
        {{3, 2}, 5, {1, -1}},
        {{3, 2}, 6, {0, 0}},
        {{4, 1}, 7, {1, -1}},
        {{4, 1}, 8, {0, 0}},
        {{5, 0}, 9, {1, -1}},
        {{5, 0}, 10, {0, 0}},
    };
    EXPECT_THAT(mm_.data, ContainerEq(expected));
}
}