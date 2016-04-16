#include "stdafx.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../include/sc/GCodeInterpreter.h"

using namespace StepperControl;
using namespace testing;
using namespace std;

namespace {
struct AxTr {
    static const int size = 2;

    static const char *names() { return "AB"; }
};

using Af = TAf<AxTr::size>;
using Ai = TAi<AxTr::size>;
using Sgs = TSgs<AxTr::size>;

struct SegmentsExecutorMock {
    void setTicksPerSecond(int32_t) {}

    void start() {}

    void stop() {}

    bool isRunning() const { return false; }

    Ai const &position() const { return pos; }

    void setPosition(Ai const &p) { pos = p; }

    void setTrajectory(Sgs const &s) { seg = s; }

    void setTrajectory(Sgs &&s) { seg = s; }

    Ai pos = axZero<Ai>();
    Sgs seg;
};

struct PrinterMock : Printer {
    void print(const float *n, int size) override {
        for (int i = 0; i < size; i++) {
            ss << n[i] << sep(i, size);
        }
    }
    void print(const int *n, int size) override {
        for (int i = 0; i < size; i++) {
            ss << n[i] << sep(i, size);
        }
    }
    void print(const char *str) { ss << str; }

    std::stringstream ss;
};

struct GCodeInterpreter_Should : Test {
    using Interp = GCodeInterpreter<SegmentsExecutorMock, AxTr>;
    SegmentsExecutorMock se;
    PrinterMock printer;
    Interp interp;

    GCodeInterpreter_Should() : interp(&se, &printer) {}

    std::vector<Ai> path() { return interp.path(); }
};

TEST_F(GCodeInterpreter_Should, add_one_linear_move_waypoint) {
    interp.linearMove(Af{200, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{200, 100}));
}

TEST_F(GCodeInterpreter_Should, add_many_linear_move_waypoints) {
    interp.linearMove(Af{0, 10}, inf());
    interp.linearMove(Af{1, 10}, inf());
    interp.linearMove(Af{0, 0}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{0, 10}, Ai{1, 10}, Ai{0, 0}));
}

TEST_F(GCodeInterpreter_Should, add_one_linear_move_from_different_start_position) {
    se.setPosition({10, 10});
    interp.linearMove(Af{200, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{10, 10}, Ai{200, 100}));
}

TEST_F(GCodeInterpreter_Should, add_only_valid_coordinates) {
    interp.linearMove(Af{inf(), 100}, inf());
    interp.linearMove(Af{200, inf()}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{0, 100}, Ai{200, 100}));
}

TEST_F(GCodeInterpreter_Should, add_only_different_coordinates) {
    interp.linearMove(Af{200, 100}, inf());
    interp.linearMove(Af{200, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{200, 100}));
}

TEST_F(GCodeInterpreter_Should, round_to_nearest_step) {
    interp.linearMove(Af{200.8f, 100.2f}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{201, 100}));
}

TEST_F(GCodeInterpreter_Should, override_steps_per_unit_lenght) {
    interp.m102StepsPerUnitLengthOverride(Af{200, 400});
    interp.linearMove(Af{0.1f, 2.5f}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{20, 1000}));
}

TEST_F(GCodeInterpreter_Should, override_only_valid_steps_per_unit_lenght) {
    interp.m102StepsPerUnitLengthOverride(Af{inf(), 400.f});
    EXPECT_THAT(interp.stepsPerUnitLength(), ElementsAre(1.f, 400.f));

    interp.m102StepsPerUnitLengthOverride(Af{100.f, inf()});
    EXPECT_THAT(interp.stepsPerUnitLength(), ElementsAre(100.f, 400.f));
}

TEST_F(GCodeInterpreter_Should, override_only_valid_max_accelerations) {
    interp.m101MaxAccelerationOverride(Af{inf(), 400.f});
    EXPECT_THAT(interp.maxAcceleration(), ElementsAre(1.f, 400.f));

    interp.m101MaxAccelerationOverride(Af{100.f, inf()});
    EXPECT_THAT(interp.maxAcceleration(), ElementsAre(100.f, 400.f));
}

TEST_F(GCodeInterpreter_Should, override_max_accelerations) {
    interp.setTicksPerSecond(10);
    interp.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp.m101MaxAccelerationOverride(Af{1000.f, 100.f});

    EXPECT_THAT(interp.maxAcceleration(), ElementsAre(10.f, 2.f));
}

TEST_F(GCodeInterpreter_Should, override_max_velocities) {
    interp.setTicksPerSecond(10000);
    interp.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp.m100MaxVelocityOverride(Af{100.f, 400.f});

    EXPECT_THAT(interp.maxVelocity(), ElementsAre(0.01f, 0.08f));
}

TEST_F(GCodeInterpreter_Should, override_max_accelerations_before_ticks_per_second) {
    interp.m101MaxAccelerationOverride(Af{1000.f, 100.f});

    interp.setTicksPerSecond(10);
    interp.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    EXPECT_THAT(interp.maxAcceleration(), ElementsAre(10.f, 2.f));
}

TEST_F(GCodeInterpreter_Should, override_max_velocities_before_ticks_per_second) {
    interp.m100MaxVelocityOverride(Af{100.f, 400.f});

    interp.setTicksPerSecond(10000);
    interp.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    EXPECT_THAT(interp.maxVelocity(), ElementsAre(0.01f, 0.08f));
}

TEST_F(GCodeInterpreter_Should, override_homing_velocity) {
    interp.setTicksPerSecond(10000);
    interp.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp.m103HomingVelocityOverride(Af{10.f, 40.f});

    EXPECT_THAT(interp.homingVelocity(), ElementsAre(0.001f, 0.008f));
}

TEST_F(GCodeInterpreter_Should, override_homing_velocity_before_ticks_per_second) {
    interp.m103HomingVelocityOverride(Af{10.f, 40.f});

    interp.setTicksPerSecond(10000);
    interp.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    EXPECT_THAT(interp.homingVelocity(), ElementsAre(0.001f, 0.008f));
}

TEST_F(GCodeInterpreter_Should, override_only_valid_max_velocities) {
    interp.m100MaxVelocityOverride(Af{inf(), 0.3f});
    EXPECT_THAT(interp.maxVelocity(), ElementsAre(1.f, 0.3f));

    interp.m100MaxVelocityOverride(Af{0.1f, inf()});
    EXPECT_THAT(interp.maxVelocity(), ElementsAre(0.1f, 0.3f));
}

TEST_F(GCodeInterpreter_Should, add_relative_positions) {
    interp.g90g91DistanceMode(DistanceMode::Relative);
    interp.linearMove(Af{10, 100}, inf());
    interp.linearMove(Af{20, inf()}, inf());
    interp.linearMove(Af{inf(), 200}, inf());
    interp.linearMove(Af{10, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{10, 100}, Ai{30, 100}, Ai{30, 300}, Ai{40, 400}));
}

TEST_F(GCodeInterpreter_Should, trim_max_velocity) {
    interp.setTicksPerSecond(1000);
    interp.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp.m100MaxVelocityOverride(Af{100.f, 1000.f});

    EXPECT_THAT(interp.maxVelocity(), ElementsAre(0.1f, 1.f));
}

TEST_F(GCodeInterpreter_Should, home_and_move) {
    interp.setTicksPerSecond(10);
    interp.m103HomingVelocityOverride({1.f, 1.f});
    interp.m100MaxVelocityOverride(Af{2.f, 2.f});
    interp.m101MaxAccelerationOverride(Af{1.f, 1.f});
    se.setPosition(Ai{10, 20});

    interp.g28RunHomingCycle();
    interp.linearMove({20.f, 20.f}, inf());
    interp.start();

    Sgs expected{
        {{0.1f, 0.1f}},

        {20, {0, 0}, {2, 2}},
        {80, {16, 16}},
        {20, {2, 2}, {0, 0}},
    };
    EXPECT_THAT(se.seg, ContainerEq(expected));
}

TEST_F(GCodeInterpreter_Should, move_and_wait) {
    interp.setTicksPerSecond(10);
    interp.m100MaxVelocityOverride(Af{2.f, 2.f});
    interp.m101MaxAccelerationOverride(Af{1.f, 1.f});
    se.setPosition(Ai{10, 20});

    interp.linearMove({20.f, 20.f}, inf());
    interp.g4Wait(2);
    interp.linearMove({10.f, 20.f}, inf());
    interp.start();

    Sgs expected{
        {20, {0, 0}, {2, 0}},
        {30, {6, 0}},
        {20, {2, 0}, {0, 0}},

        {20},

        {20, {0, 0}, {-2, 0}},
        {30, {-6, 0}},
        {20, {-2, 0}, {0, 0}},
    };
    EXPECT_THAT(se.seg, ContainerEq(expected));
}

TEST_F(GCodeInterpreter_Should, change_vel_between_moves) {
    interp.setTicksPerSecond(10);
    interp.m100MaxVelocityOverride(Af{2.f, 2.f});
    interp.m101MaxAccelerationOverride(Af{1.f, 1.f});
    se.setPosition(Ai{10, 20});

    interp.linearMove({20.f, 20.f}, inf());
    interp.m100MaxVelocityOverride(Af{1.f, 2.f});
    interp.m101MaxAccelerationOverride(Af{0.5f, 1.f});
    interp.linearMove({10.f, 20.f}, inf());
    interp.start();

    Sgs expected{
        {20, {0, 0}, {2, 0}},  {30, {6, 0}},  {20, {2, 0}, {0, 0}},

        {20, {0, 0}, {-1, 0}}, {80, {-8, 0}}, {20, {-1, 0}, {0, 0}},
    };
    EXPECT_THAT(se.seg, ContainerEq(expected));
}

TEST_F(GCodeInterpreter_Should, move_and_wait_0_sec) {
    interp.setTicksPerSecond(10);
    interp.m100MaxVelocityOverride(Af{2.f, 2.f});
    interp.m101MaxAccelerationOverride(Af{1.f, 1.f});
    se.setPosition(Ai{10, 20});

    interp.linearMove({20.f, 20.f}, inf());
    interp.g4Wait(0);
    interp.linearMove({10.f, 20.f}, inf());
    interp.start();

    Sgs expected{
        {20, {0, 0}, {2, 0}},  {30, {6, 0}},  {20, {2, 0}, {0, 0}},

        {20, {0, 0}, {-2, 0}}, {30, {-6, 0}}, {20, {-2, 0}, {0, 0}},
    };
    EXPECT_THAT(se.seg, ContainerEq(expected));
}

TEST_F(GCodeInterpreter_Should, set_max_position) {
    interp.m106MaxPositionOverride(Af{2.f, 30.f});

    interp.m102StepsPerUnitLengthOverride({1.f, 10.f});

    EXPECT_THAT(interp.maxPosition(), ElementsAre(2.f, 300.f));
}

TEST_F(GCodeInterpreter_Should, set_min_position) {
    interp.m105MinPositionOverride(Af{2.f, 30.f});

    interp.m102StepsPerUnitLengthOverride({1.f, 10.f});

    EXPECT_THAT(interp.minPosition(), ElementsAre(2.f, 300.f));
}

TEST_F(GCodeInterpreter_Should, trim_position) {
    interp.m106MaxPositionOverride(Af{inf(), 10.f});
    interp.m105MinPositionOverride(Af{-10.f, inf()});

    interp.linearMove(Af{20, 20}, inf());
    interp.linearMove(Af{-20, -20}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{20, 10}, Ai{-10, -20}));
}

// Responses

TEST_F(GCodeInterpreter_Should, print_current_position) {
    se.pos = {1, 2};
    interp.m102StepsPerUnitLengthOverride({10, 100});

    interp.printCurrentPosition();

    EXPECT_THAT(printer.ss.str(), StrEq("Position: 0.1, 0.02\r\n"));
}

TEST_F(GCodeInterpreter_Should, print_completed) {
    interp.printCompleted();

    EXPECT_THAT(printer.ss.str(), StrEq("Position: 0, 0\r\nCompleted\r\n"));
}

TEST_F(GCodeInterpreter_Should, print_axes_configuration) {
    interp.m110PrintAxesConfiguration();

    EXPECT_THAT(printer.ss.str(), StrEq("Axes: AB\r\n"));
}

TEST_F(GCodeInterpreter_Should, print_error) {
    interp.error("test", 0, "str");

    EXPECT_THAT(printer.ss.str(), StrEq("Error: test at 0 in str\r\n"));
}

TEST_F(GCodeInterpreter_Should, set_negative_spu) {
    interp.m102StepsPerUnitLengthOverride(Af{-200, -400});
    interp.linearMove(Af{0.1f, 2.5f}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{-20, -1000}));
}

TEST_F(GCodeInterpreter_Should, add_relative_positions_with_negative_spu) {
    interp.m102StepsPerUnitLengthOverride(Af{1, -1});
    interp.g90g91DistanceMode(DistanceMode::Relative);
    interp.linearMove(Af{10, 100}, inf());
    interp.linearMove(Af{20, inf()}, inf());
    interp.linearMove(Af{inf(), 200}, inf());
    interp.linearMove(Af{10, 100}, inf());
    EXPECT_THAT(path(),
                ElementsAre(Ai{0, 0}, Ai{10, -100}, Ai{30, -100}, Ai{30, -300}, Ai{40, -400}));
}

TEST_F(GCodeInterpreter_Should, trim_max_velocity_with_negative_spu) {
    interp.setTicksPerSecond(1000);
    interp.m102StepsPerUnitLengthOverride(Af{-1.f, -2.f});

    interp.m100MaxVelocityOverride(Af{100.f, 1000.f});

    EXPECT_THAT(interp.maxVelocity(), ElementsAre(-0.1f, -1.f));
}

TEST_F(GCodeInterpreter_Should, trim_max_homing_velocity_with_negative_spu) {
    interp.setTicksPerSecond(1000);
    interp.m102StepsPerUnitLengthOverride(Af{-1.f, -2.f});

    interp.m103HomingVelocityOverride(Af{100.f, 1000.f});

    EXPECT_THAT(interp.homingVelocity(), ElementsAre(-0.1f, -1.f));
}
}
