#include "stdafx.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../include/sc/SegmentsExecutor.h"

using namespace StepperControl;
using namespace testing;
using namespace std;

namespace {

template <size_t AxesSize>
struct AxTr {
    static const int size = AxesSize;
    static const char *names() { return "ABC"; }
};

template <size_t AxesSize>
struct MotorMock {
    MotorMock() : isHit(axZero<Ai>()), dir(axZero<Ai>()), current{axZero<Ai>(), axZero<Ai>()} {}

    using Ai = Axes<int32_t, AxesSize>;

    struct Step {
        Step(Ai const &x, Ai const &step) : x(x), step(step) {}

        friend bool operator==(Step const &lhs, Step const &rhs) {
            return lhs.x == rhs.x && lhs.step == rhs.step;
        }

        friend bool operator!=(Step const &lhs, Step const &rhs) { return !(lhs == rhs); }

        friend ostream &operator<<(ostream &os, Step const &obj) {
            return os << endl << "x: " << obj.x << " step: " << obj.step;
        }

        Ai x, step;
    };

    template <int i, int reverse>
    void writeDirection(UIntConst<i>, UIntConst<reverse>) {
        dir[i] = (reverse ? -1 : 1);
    }

    template <int i, int edge>
    void writeStep(UIntConst<i>, UIntConst<edge>) {
        current.step[i] = (edge ? dir[i] : 0);
        current.x[i] += current.step[i];
    }

    static void begin() {}
    void end() { data.emplace_back(current); }

    void setPosition(Ai const &position) { current.x = position; }

    bool checkEndSwitchHit(size_t i) const { return isHit[i] != 0; }

    Ai isHit;
    Ai dir;
    Step current;
    vector<Step> data;
};

struct TickerMock {
    static void attach_us(...) {}

    static void detach() {}
};

template <size_t AxesSize>
struct SegmentsExecutorTestBase : Test {
    using Ai = Axes<int32_t, AxesSize>;
    using Af = Axes<float, AxesSize>;
    using Sg = Segment<AxesSize>;
    using Mm = MotorMock<AxesSize>;
    using Steps = vector<typename Mm::Step>;
    using Executor = SegmentsExecutor<Mm, TickerMock, AxTr<AxesSize>>;
    Mm motor;
    TickerMock ticker;
    typename Executor::Sgs segments;
    Executor executor;

    SegmentsExecutorTestBase() : executor{&motor, &ticker} {}

    void process() {
        executor.setSegments(segments);
        executor.start();
        while (executor.isRunning()) {
            executor.tick();
        }
    }
};

struct SegmentsExecutor1_Should : SegmentsExecutorTestBase<1> {};

TEST_F(SegmentsExecutor1_Should, execute_one_linear_segment) {
    segments.push_back(Sg(10, {5}));
    process();

    Steps expected{
        // {{0}, {0}}, // 0
        {{1}, {1}}, {{1}, {0}}, {{2}, {1}}, {{2}, {0}}, {{3}, {1}},
        {{3}, {0}}, {{4}, {1}}, {{4}, {0}}, {{5}, {1}}, {{5}, {0}}, // 10
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_linear_segment_from_negative_position) {
    motor.setPosition({-5});
    segments.push_back(Sg(10, {5}));
    process();

    Steps expected{
        // {{-5}, {0}}, // 0
        {{-4}, {1}}, {{-4}, {0}}, {{-3}, {1}}, {{-3}, {0}}, {{-2}, {1}},
        {{-2}, {0}}, {{-1}, {1}}, {{-1}, {0}}, {{0}, {1}},  {{0}, {0}}, // 10
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_linear_segment_with_negative_slope) {
    motor.setPosition({5});
    segments.push_back(Sg(10, {-5}));
    process();

    Steps expected{
        //  {{5}, {0}}, // 0
        {{4}, {-1}}, {{4}, {0}},  {{3}, {-1}}, {{3}, {0}},  {{2}, {-1}},
        {{2}, {0}},  {{1}, {-1}}, {{1}, {0}},  {{0}, {-1}}, {{0}, {0}}, // 10
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_short_linear_segment) {
    segments.push_back(Sg(2, {1}));
    process();

    Steps expected{
        // /**/ {{0}, {0}},
        /**/ {{1}, {1}},
        /**/ {{1}, {0}},
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_horizontal_linear_segment) {
    segments.push_back({Sg(4, {0})});
    process();

    Steps expected{
        //  /**/ {{0}, {0}}, // 0
        /**/ {{0}, {0}},
        /**/ {{0}, {0}},
        /**/ {{0}, {0}},
        /**/ {{0}, {0}}, // 4
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_linear_segment_with_slow_negative_slope) {
    segments.push_back(Sg(15, {-5}));
    process();

    Steps expected{
        //{{0}, {0}}, // 0
        {{0}, {0}},   {{-1}, {-1}}, {{-1}, {0}},  {{-1}, {0}},  {{-2}, {-1}},
        {{-2}, {0}},  {{-2}, {0}},  {{-3}, {-1}}, {{-3}, {0}},  {{-3}, {0}},
        {{-4}, {-1}}, {{-4}, {0}},  {{-4}, {0}},  {{-5}, {-1}}, {{-5}, {0}}, // 15
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_rising_parabolic_segment) {
    segments.push_back(Sg(20, {5}, {0}));
    process();

    Steps expected{
        // {{0}, {0}}, // 0
        {{0}, {0}}, {{1}, {1}}, {{1}, {0}}, {{2}, {1}}, {{2}, {0}},
        {{3}, {1}}, {{3}, {0}}, {{3}, {0}}, {{3}, {0}}, {{4}, {1}}, // 10
        {{4}, {0}}, {{4}, {0}}, {{4}, {0}}, {{5}, {1}}, {{5}, {0}},
        {{5}, {0}}, {{5}, {0}}, {{5}, {0}}, {{5}, {0}}, {{5}, {0}}, // 20
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_falling_parabolic_segment) {
    motor.setPosition({5});
    segments.push_back(Sg(20, {0}, {-5}));
    process();

    Steps expected{
        //{{5}, {0}}, // 0
        {{5}, {0}},  {{5}, {0}},  {{5}, {0}}, {{5}, {0}},  {{5}, {0}},
        {{5}, {0}},  {{4}, {-1}}, {{4}, {0}}, {{4}, {0}},  {{4}, {0}}, // 10
        {{3}, {-1}}, {{3}, {0}},  {{3}, {0}}, {{3}, {0}},  {{2}, {-1}},
        {{2}, {0}},  {{1}, {-1}}, {{1}, {0}}, {{0}, {-1}}, {{0}, {0}}, // 20
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_rising_parabolic_segment_with_negative_curvature) {
    segments.push_back(Sg(20, {-5}, {0}));
    process();

    Steps expected{
        // {{0}, {0}}, // 0
        {{0}, {0}},   {{-1}, {-1}}, {{-1}, {0}}, {{-2}, {-1}}, {{-2}, {0}},
        {{-3}, {-1}}, {{-3}, {0}},  {{-3}, {0}}, {{-3}, {0}},  {{-4}, {-1}}, // 10
        {{-4}, {0}},  {{-4}, {0}},  {{-4}, {0}}, {{-5}, {-1}}, {{-5}, {0}},
        {{-5}, {0}},  {{-5}, {0}},  {{-5}, {0}}, {{-5}, {0}},  {{-5}, {0}}, // 20
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_falling_parabolic_segment_with_negative_curvature) {
    motor.setPosition({-5});
    segments.push_back(Sg(20, {0}, {5}));
    process();

    Steps expected{
        // {{-5}, {0}}, // 0
        {{-5}, {0}}, {{-5}, {0}}, {{-5}, {0}}, {{-5}, {0}}, {{-5}, {0}},
        {{-5}, {0}}, {{-4}, {1}}, {{-4}, {0}}, {{-4}, {0}}, {{-4}, {0}}, // 10
        {{-3}, {1}}, {{-3}, {0}}, {{-3}, {0}}, {{-3}, {0}}, {{-2}, {1}},
        {{-2}, {0}}, {{-1}, {1}}, {{-1}, {0}}, {{-0}, {1}}, {{-0}, {0}}, // 20
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_one_short_parabolic_segment_) {
    segments.push_back(Sg(4, {1}, {0}));
    process();

    Steps expected{
        //   /**/ {{0}, {0}}, // 0
        /**/ {{0}, {0}},
        /**/ {{1}, {1}},
        /**/ {{1}, {0}},
        /**/ {{1}, {0}}, // 4
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, approximate_parabolic_curve_with_zero_curvature_as_line) {
    segments.push_back(Sg(10, {2}, {2}));
    process();

    Steps expected{
        // {{0}, {0}}, // 0
        {{0}, {0}}, {{1}, {1}}, {{1}, {0}}, {{2}, {1}}, {{2}, {0}},
        {{2}, {0}}, {{3}, {1}}, {{3}, {0}}, {{4}, {1}}, {{4}, {0}}, // 10
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_two_linear_segments) {
    segments.push_back(Sg(6, {3}));
    segments.push_back(Sg(6, {-3}));
    process();
    Steps expected{
        // {{0}, {0}}, // 0
        {{1}, {1}},  {{1}, {0}}, {{2}, {1}},  {{2}, {0}}, {{3}, {1}},  {{3}, {0}}, // 6
        {{2}, {-1}}, {{2}, {0}}, {{1}, {-1}}, {{1}, {0}}, {{0}, {-1}}, {{0}, {0}}, // 12
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_parabolic_segment_with_gradient_change) {
    segments.push_back(Sg(8, {2}, {-2}));
    process();
    Steps expected{
        // {{0}, {0}}, // 0
        {{0}, {0}}, {{1}, {1}}, {{1}, {0}},  {{1}, {0}},
        {{1}, {0}}, {{1}, {0}}, {{0}, {-1}}, {{0}, {0}}, // 8
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, execute_two_linear_segments_with_parabolic_blend) {
    segments.push_back(Sg(8, {4}));
    segments.push_back(Sg(8, {2}, {-2}));
    segments.push_back(Sg(8, {-4}));
    process();
    Steps expected{
        // {{0}, {0}}, // 0
        {{1}, {1}},  {{1}, {0}}, {{2}, {1}},  {{2}, {0}},
        {{3}, {1}},  {{3}, {0}}, {{4}, {1}},  {{4}, {0}}, // 8
        {{4}, {0}},  {{5}, {1}}, {{5}, {0}},  {{5}, {0}},
        {{5}, {0}},  {{5}, {0}}, {{4}, {-1}}, {{4}, {0}}, // 16
        {{3}, {-1}}, {{3}, {0}}, {{2}, {-1}}, {{2}, {0}},
        {{1}, {-1}}, {{1}, {0}}, {{0}, {-1}}, {{0}, {0}}, // 24
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, do_homing_and_other_commands) {
    segments.push_back(Sg(6, {3}));
    segments.push_back(Sg(Af{0.5f}));
    segments.push_back(Sg(6, {3}));
    executor.setSegments(segments);
    executor.start();

    while (executor.isRunning()) {
        executor.tick();
        if (motor.current.x[0] == 0) {
            motor.isHit[0] = 1;
        }
    }

    Steps expected{
        {{1}, {1}}, {{1}, {0}},  {{2}, {1}}, {{2}, {0}},  {{3}, {1}}, {{3}, {0}},

        {{3}, {0}}, {{2}, {-1}}, {{2}, {0}}, {{1}, {-1}}, {{1}, {0}}, {{0}, {-1}}, {{0}, {0}},

        {{1}, {1}}, {{1}, {0}},  {{2}, {1}}, {{2}, {0}},  {{3}, {1}}, {{3}, {0}},
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor1_Should, callback_on_stopped) {
    bool stopped = false;
    auto func = [](void *obj) { *static_cast<bool *>(obj) = true; };
    executor.setOnStopped(func, &stopped);
    segments.push_back(Sg(6, {3}));
    process();

    Steps expected{
        {{1}, {1}}, {{1}, {0}}, {{2}, {1}}, {{2}, {0}}, {{3}, {1}}, {{3}, {0}},
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
    EXPECT_THAT(stopped, Eq(true));
}

struct SegmentsExecutor2_Should : SegmentsExecutorTestBase<2> {};

TEST_F(SegmentsExecutor2_Should, execute_one_linear_segment) {
    motor.setPosition({0, 5});
    segments.push_back(Sg(10, {5, -5}));
    process();

    Steps expected{
        // {{0, 5}, {0, 0}}, // 0
        {{1, 4}, {1, -1}}, {{1, 4}, {0, 0}}, {{2, 3}, {1, -1}}, {{2, 3}, {0, 0}},
        {{3, 2}, {1, -1}}, {{3, 2}, {0, 0}}, {{4, 1}, {1, -1}}, {{4, 1}, {0, 0}},
        {{5, 0}, {1, -1}}, {{5, 0}, {0, 0}}, // 10
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
}

TEST_F(SegmentsExecutor2_Should, wait) {
    segments.push_back(Sg(10));

    process();

    EXPECT_THAT(motor.data, SizeIs(10));
    EXPECT_THAT(motor.data, Each(Eq(Mm::Step({0, 0}, {0, 0}))));
}

TEST_F(SegmentsExecutor2_Should, handle_zero_ticks_wait) {
    segments.push_back(Sg(0));

    process();

    EXPECT_THAT(motor.data, SizeIs(0));
}

TEST_F(SegmentsExecutor2_Should, do_homing) {
    segments.push_back(Sg({0.5f, 0.2f}));
    executor.setPosition({10, 20});
    executor.setSegments(segments);
    executor.start();

    while (executor.isRunning()) {
        executor.tick();
        if (motor.current.x[0] == -5) {
            motor.isHit[0] = 1;
        }
        if (motor.current.x[1] == -3) {
            motor.isHit[1] = 1;
        }
    }

    Steps expected{
        {{0, 0}, {0, 0}},    {{-1, 0}, {-1, 0}},  {{-1, -1}, {0, -1}}, {{-2, -1}, {-1, 0}},
        {{-2, -1}, {0, 0}},  {{-3, -1}, {-1, 0}}, {{-3, -1}, {0, 0}},  {{-4, -2}, {-1, -1}},
        {{-4, -2}, {0, 0}},  {{-5, -2}, {-1, 0}}, {{-5, -2}, {0, 0}},  {{-5, -2}, {0, 0}},
        {{-5, -3}, {0, -1}}, {{-5, -3}, {0, 0}},
    };
    EXPECT_THAT(motor.data, ContainerEq(expected));
    EXPECT_THAT(executor.position(), Eq(Ai{0, 0}));
}
}