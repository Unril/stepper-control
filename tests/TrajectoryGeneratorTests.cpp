#include "stdafx.h"

#include "../include/sc/TrajectoryGenerator.h"

using namespace StepperControl;
using namespace testing;

namespace {
const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct TrajectoryGenerator_Should : Test {
    using TrajGen = TrajectoryGenerator<AxesSize>;
    std::vector<Ai> path_;
    TrajGen gen_;

    TrajectoryGenerator_Should() {
        gen_.setMaxAcceleration({10, 10});
        gen_.setMaxVelocity({20, 20});
    }

    void update() {
        gen_.setPath(path_);
        gen_.update();
    }
};

TEST_F(TrajectoryGenerator_Should, get_trajectory_for_one_axis_and_two_points) {
    path_.push_back({0, 0});
    path_.push_back({100, 0});

    update();

    EXPECT_THAT(gen_.velocities(), ElementsAre(Af{20, 0}));
    EXPECT_THAT(gen_.durations(), ElementsAre(5.f));
    EXPECT_THAT(gen_.accelerations(), ElementsAre(Af{10, 0}, Af{-10, 0}));
    EXPECT_THAT(gen_.blendDurations(), ElementsAre(2.f, 2.f));
}

TEST_F(TrajectoryGenerator_Should, get_trajectory_for_two_axes_and_two_points) {
    path_.push_back({0, 0});
    path_.push_back({100, 200});

    update();

    EXPECT_THAT(gen_.velocities(), ElementsAre(Af{10, 20}));
    EXPECT_THAT(gen_.durations(), ElementsAre(10.f));
    EXPECT_THAT(gen_.accelerations(), ElementsAre(Af{5, 10}, Af{-5, -10}));
    EXPECT_THAT(gen_.blendDurations(), ElementsAre(2.f, 2.f));
}

TEST_F(TrajectoryGenerator_Should, apply_slowdown) {
    path_.push_back({0, 0});
    path_.push_back({100, 200});
    gen_.setMaxVelocity(Af{50, 50});
    gen_.setMaxAcceleration(Af{1, 2});

    update();

    EXPECT_THAT(gen_.velocities(), ElementsAre(Af{10, 20}));
    EXPECT_THAT(gen_.durations(), ElementsAre(10.f));
    EXPECT_THAT(gen_.accelerations(), ElementsAre(Af{1, 2}, Af{-1, -2}));
    EXPECT_THAT(gen_.blendDurations(), ElementsAre(10.f, 10.f));
}

TEST_F(TrajectoryGenerator_Should, move_to_different_directions) {
    path_.push_back({100, 0});
    path_.push_back({0, 100});

    update();

    EXPECT_THAT(gen_.velocities(), ElementsAre(Af{-20, 20}));
    EXPECT_THAT(gen_.durations(), ElementsAre(5.f));
    EXPECT_THAT(gen_.accelerations(), ElementsAre(Af{-10, 10}, Af{10, -10}));
    EXPECT_THAT(gen_.blendDurations(), ElementsAre(2.f, 2.f));
}
}