#include "stdafx.h"

#include "../include/sc/GCodeInterpreter.h"

using namespace StepperControl;
using namespace testing;

namespace {
const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;
using Cmd = Command<AxesSize>;

struct GCodeInterpreter_Should : Test {
    using Interp = GCodeInterpreter<AxesSize>;
    Interp interp_;
    std::vector<Ai> path() { return interp_.commands().back().path(); }
};

TEST_F(GCodeInterpreter_Should, add_one_linear_move_waypoint) {
    interp_.linearMove(Af{200, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{200, 100}));
}

TEST_F(GCodeInterpreter_Should, add_only_valid_coordinates) {
    interp_.linearMove(Af{inf(), 100}, inf());
    interp_.linearMove(Af{200, inf()}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{0, 100}, Ai{200, 100}));
}

TEST_F(GCodeInterpreter_Should, add_only_different_coordinates) {
    interp_.linearMove(Af{200, 100}, inf());
    interp_.linearMove(Af{200, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{200, 100}));
}

TEST_F(GCodeInterpreter_Should, round_to_nearest_step) {
    interp_.linearMove(Af{200.8, 100.2}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{201, 100}));
}

TEST_F(GCodeInterpreter_Should, override_steps_per_unit_lenght) {
    interp_.m102StepsPerUnitLengthOverride(Af{200, 400});
    interp_.linearMove(Af{0.1f, 2.5f}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{20, 1000}));
}

TEST_F(GCodeInterpreter_Should, override_only_valid_steps_per_unit_lenght) {
    interp_.m102StepsPerUnitLengthOverride(Af{inf(), 400.f});
    EXPECT_THAT(interp_.stepsPerUnitLength(), ElementsAre(1.f, 400.f));

    interp_.m102StepsPerUnitLengthOverride(Af{100.f, inf()});
    EXPECT_THAT(interp_.stepsPerUnitLength(), ElementsAre(100.f, 400.f));
}

TEST_F(GCodeInterpreter_Should, override_only_valid_max_accelerations) {
    interp_.m101MaxAccelerationOverride(Af{inf(), 400.f});
    EXPECT_THAT(interp_.maxAcceleration(), ElementsAre(0.1f, 400.f));

    interp_.m101MaxAccelerationOverride(Af{100.f, inf()});
    EXPECT_THAT(interp_.maxAcceleration(), ElementsAre(100.f, 400.f));
}

TEST_F(GCodeInterpreter_Should, override_max_accelerations) {
    interp_.setTicksPerSecond(10);
    interp_.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp_.m101MaxAccelerationOverride(Af{1000.f, 100.f});

    EXPECT_THAT(interp_.maxAcceleration(), ElementsAre(10.f, 2.f));
}

TEST_F(GCodeInterpreter_Should, override_max_velocities) {
    interp_.setTicksPerSecond(10000);
    interp_.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp_.m100MaxVelocityOverride(Af{100.f, 400.f});

    EXPECT_THAT(interp_.maxVelocity(), ElementsAre(0.01f, 0.08f));
}

TEST_F(GCodeInterpreter_Should, override_only_valid_max_velocities) {
    interp_.m100MaxVelocityOverride(Af{inf(), 0.3f});
    EXPECT_THAT(interp_.maxVelocity(), ElementsAre(0.5f, 0.3f));

    interp_.m100MaxVelocityOverride(Af{0.1f, inf()});
    EXPECT_THAT(interp_.maxVelocity(), ElementsAre(0.1f, 0.3f));
}

TEST_F(GCodeInterpreter_Should, add_relative_positions) {
    interp_.g90g91DistanceMode(Relative);
    interp_.linearMove(Af{10, 100}, inf());
    interp_.linearMove(Af{20, inf()}, inf());
    interp_.linearMove(Af{inf(), 200}, inf());
    interp_.linearMove(Af{10, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{10, 100}, Ai{30, 100}, Ai{30, 300}, Ai{40, 400}));
}

TEST_F(GCodeInterpreter_Should, trim_max_velocity_to_one_half) {
    interp_.setTicksPerSecond(1000);
    interp_.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp_.m100MaxVelocityOverride(Af{100.f, 400.f});

    EXPECT_THAT(interp_.maxVelocity(), ElementsAre(0.1f, 0.5f));
}

TEST_F(GCodeInterpreter_Should, home_and_move) {
    interp_.setTicksPerSecond(10);
    interp_.m103HomingVelocityOverride({1.f, 2.f});
    interp_.m100MaxVelocityOverride(Af{2.f, 4.f});
    interp_.m101MaxAccelerationOverride(Af{2.f, 2.f});
    interp_.setCurrentPosition(Ai{10, 20});

    interp_.g28RunHomingCycle();
    interp_.linearMove({10.f, 10.f}, inf());
    interp_.linearMove({20.f, 20.f}, inf());

    auto cmds = interp_.commands();
    ASSERT_THAT(cmds, SizeIs(2));

    EXPECT_THAT(cmds[0].type(), Eq(Interp::Cmd::Homing));
    EXPECT_THAT(cmds[0].homingVelocity(), Eq(Af{0.1f, 0.2f}));

    EXPECT_THAT(cmds[1].type(), Eq(Interp::Cmd::Move));
    EXPECT_THAT(cmds[1].maxVelocity(), Eq(Af{0.2f, 0.4f}));
    EXPECT_THAT(cmds[1].maxAcceleration(), Eq(Af{0.02f, 0.02f}));
    EXPECT_THAT(cmds[1].path(), ElementsAre(Ai{10, 20}, Ai{10, 10}, Ai{20, 20}));
}

TEST_F(GCodeInterpreter_Should, move_and_wait) {
    interp_.setTicksPerSecond(10);
    interp_.m100MaxVelocityOverride(Af{2.f, 4.f});
    interp_.m101MaxAccelerationOverride(Af{2.f, 2.f});
    interp_.setCurrentPosition(Ai{10, 20});

    interp_.linearMove({10.f, 10.f}, inf());

    interp_.g4Wait(10);

    interp_.m100MaxVelocityOverride(Af{1.f, 2.f});
    interp_.m101MaxAccelerationOverride(Af{4.f, 4.f});
    interp_.linearMove({20.f, 20.f}, inf());

    auto cmds = interp_.commands();
    ASSERT_THAT(cmds, SizeIs(3));

    EXPECT_THAT(cmds[0].type(), Eq(Interp::Cmd::Move));
    EXPECT_THAT(cmds[0].maxVelocity(), Eq(Af{0.2f, 0.4f}));
    EXPECT_THAT(cmds[0].maxAcceleration(), Eq(Af{0.02f, 0.02f}));
    EXPECT_THAT(cmds[0].path(), ElementsAre(Ai{10, 20}, Ai{10, 10}));

    EXPECT_THAT(cmds[1].type(), Eq(Interp::Cmd::Wait));
    EXPECT_THAT(cmds[1].waitDuration(), Eq(100));

    EXPECT_THAT(cmds[2].type(), Eq(Interp::Cmd::Move));
    EXPECT_THAT(cmds[2].maxVelocity(), Eq(Af{0.1f, 0.2f}));
    EXPECT_THAT(cmds[2].maxAcceleration(), Eq(Af{0.04f, 0.04f}));
    EXPECT_THAT(cmds[2].path(), ElementsAre(Ai{10, 10}, Ai{20, 20}));
}

TEST(Command_Should, be_moved) {
    Cmd c1({Ai{1, 2}}, Af{1, 1}, Af{2, 2});
    Cmd c2 = std::move(c1);

    EXPECT_THAT(c1.path(), IsEmpty());
    EXPECT_THAT(c2.path(), Not(IsEmpty()));
}

}