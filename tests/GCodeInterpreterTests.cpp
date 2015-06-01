#include "stdafx.h"

#include "../include/sc/GCodeInterpreter.h"

using namespace StepperControl;
using namespace testing;

namespace {
const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct GCodeInterpreter_Should : Test {
    using Interp = GCodeInterpreter<AxesSize>;
    Interp interp_;
    std::vector<Ai> path() {
        return interp_.path();
    }
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
    EXPECT_THAT(interp_.maxAcceleration(), ElementsAre(1.f, 400.f));

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
    interp_.setTicksPerSecond(10);
    interp_.m102StepsPerUnitLengthOverride(Af{1.f, 2.f});

    interp_.m100MaxVelocityOverride(Af{100.f, 400.f});

    EXPECT_THAT(interp_.maxVelocity(), ElementsAre(10.f, 80.f));
}

TEST_F(GCodeInterpreter_Should, override_only_valid_max_velocities) {
    interp_.m100MaxVelocityOverride(Af{inf(), 400.f});
    EXPECT_THAT(interp_.maxVelocity(), ElementsAre(1.f, 400.f));

    interp_.m100MaxVelocityOverride(Af{100.f, inf()});
    EXPECT_THAT(interp_.maxVelocity(), ElementsAre(100.f, 400.f));
}

TEST_F(GCodeInterpreter_Should, add_relative_positions) {
    interp_.g90g91DistanceMode(Relative);
    interp_.linearMove(Af{10, 100}, inf());
    interp_.linearMove(Af{20, inf()}, inf());
    interp_.linearMove(Af{inf(), 200}, inf());
    interp_.linearMove(Af{10, 100}, inf());
    EXPECT_THAT(path(), ElementsAre(Ai{0, 0}, Ai{10, 100}, Ai{30, 100}, Ai{30, 300}, Ai{40, 400}));
}

}