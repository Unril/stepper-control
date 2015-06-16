#include "stdafx.h"

#include "../include/sc/GCodeInterpreter.h"

using namespace StepperControl;
using namespace testing;

namespace {
const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct SegmentsExecutorMock : ISegmentsExecutor<AxesSize> {
    void setTicksPerSecond(int32_t) override {}

    void start() override {}

    void stop() override {}

    bool isRunning() const override { return false; }

    Ai const &position() const override { return pos; }

    void setPosition(Ai const &p) override { pos = p; }

    void setSegments(Segments const &s) override { seg = s; }

    void setSegments(Segments &&s) override { seg = s; }

    Ai pos = axZero<Ai>();
    Segments seg;
};

struct GCodeInterpreter_Should : Test {
    using Interp = GCodeInterpreter<AxesSize>;
    SegmentsExecutorMock se;
    Interp interp;

    GCodeInterpreter_Should() : interp(&se) {}

    std::vector<Ai> path() { return interp.path(); }
};

TEST_F(GCodeInterpreter_Should, add_one_linear_move_waypoint) {
    interp.linearMove(Af{200, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{200, 100}));
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
    interp.linearMove(Af{200.8, 100.2}, inf());
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
    EXPECT_THAT(interp.maxAcceleration(), ElementsAre(0.1f, 400.f));

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

TEST_F(GCodeInterpreter_Should, override_only_valid_max_velocities) {
    interp.m100MaxVelocityOverride(Af{inf(), 0.3f});
    EXPECT_THAT(interp.maxVelocity(), ElementsAre(0.5f, 0.3f));

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

TEST_F(GCodeInterpreter_Should, trim_max_velocity_to_one_half) {
    interp.setTicksPerSecond(1000);
    interp.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp.m100MaxVelocityOverride(Af{100.f, 400.f});

    EXPECT_THAT(interp.maxVelocity(), ElementsAre(0.1f, 0.5f));
}

TEST_F(GCodeInterpreter_Should, home_and_move) {
    interp.setTicksPerSecond(10);
    interp.m103HomingVelocityOverride({1.f, 2.f});
    interp.m100MaxVelocityOverride(Af{2.f, 2.f});
    interp.m101MaxAccelerationOverride(Af{1.f, 1.f});
    se.setPosition(Ai{10, 20});

    interp.g28RunHomingCycle();
    interp.linearMove({20.f, 20.f}, inf());
    interp.start();

    SegmentsExecutorMock::Segments expected{
        {{0.1f, 0.2f}},

        {20, {0, 0}, {2, 0}},
        {30, {6, 0}},
        {20, {2, 0}, {0, 0}},
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

    SegmentsExecutorMock::Segments expected{
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

}
