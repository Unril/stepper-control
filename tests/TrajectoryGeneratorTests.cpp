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
    std::vector<Ai> path;
    TrajGen gen;

    TrajectoryGenerator_Should() {
        gen.setMaxAcceleration({10, 10});
        gen.setMaxVelocity({20, 20});
    }

    void update() {
        gen.setPath(path);
        gen.update();
    }
};

TEST_F(TrajectoryGenerator_Should, get_trajectory_for_one_axis_and_two_points) {
    path.push_back({0, 0});
    path.push_back({100, 0});

    update();

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{20, 0}));
    EXPECT_THAT(gen.durations(), ElementsAre(5));
    EXPECT_THAT(gen.accelerations(), ElementsAre(Af{10, 0}, Af{-10, 0}));
    EXPECT_THAT(gen.blendDurations(), ElementsAre(2, 2));
}

TEST_F(TrajectoryGenerator_Should, get_trajectory_for_two_axes_and_two_points) {
    path.push_back({0, 0});
    path.push_back({100, 200});

    update();

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{10, 20}));
    EXPECT_THAT(gen.durations(), ElementsAre(10));
    EXPECT_THAT(gen.accelerations(), ElementsAre(Af{5, 10}, Af{-5, -10}));
    EXPECT_THAT(gen.blendDurations(), ElementsAre(2, 2));
}

TEST_F(TrajectoryGenerator_Should, apply_slowdown) {
    path.push_back({0, 0});
    path.push_back({100, 200});
    gen.setMaxVelocity(Af{50, 50});
    gen.setMaxAcceleration(Af{1, 2});

    update();

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{10, 20}));
    EXPECT_THAT(gen.durations(), ElementsAre(10));
    EXPECT_THAT(gen.accelerations(), ElementsAre(Af{1, 2}, Af{-1, -2}));
    EXPECT_THAT(gen.blendDurations(), ElementsAre(10, 10));
}

TEST_F(TrajectoryGenerator_Should, move_to_different_directions) {
    path.push_back({100, 0});
    path.push_back({0, 100});

    update();

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{-20, 20}));
    EXPECT_THAT(gen.durations(), ElementsAre(5));
    EXPECT_THAT(gen.accelerations(), ElementsAre(Af{-10, 10}, Af{10, -10}));
    EXPECT_THAT(gen.blendDurations(), ElementsAre(2, 2));
}

TEST_F(TrajectoryGenerator_Should, remove_points_with_all_axes_close_than_threshold) {
    path.push_back({0, 0});
    path.push_back({100, -100});
    path.push_back({108, -104});
    path.push_back({0, 0});
    gen.setPath(path);

    gen.removeCloseWaypoints({10, 5});

    std::vector<Ai> expected{
        {0, 0}, {104, -102}, {0, 0},
    };
    EXPECT_THAT(gen.path(), ContainerEq(expected));
}

TEST_F(TrajectoryGenerator_Should, not_remove_points_with_not_all_axes_close_than_threshold) {
    path.push_back({0, 0});
    path.push_back({100, 100});
    path.push_back({111, 100});
    path.push_back({111, 106});
    path.push_back({0, 0});
    gen.setPath(path);

    gen.removeCloseWaypoints({10, 5});

    EXPECT_THAT(gen.path(), ContainerEq(path));
}

TEST_F(TrajectoryGenerator_Should, not_change_first_or_last_points) {
    path.push_back({0, 0});
    path.push_back({1, -1});
    path.push_back({99, -99});
    path.push_back({100, -100});
    gen.setPath(path);

    gen.removeCloseWaypoints({10, 5});

    std::vector<Ai> expected{
        {0, 0}, {100, -100},
    };
    EXPECT_THAT(gen.path(), ContainerEq(expected));
}
}