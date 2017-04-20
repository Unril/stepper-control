#include "stdafx.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../include/sc/PathToTrajectoryConverter.h"

using namespace StepperControl;
using namespace testing;

namespace {
const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct PathToTrajectoryConverter_Should : Test {
    using TrajGen = PathToTrajectoryConverter<AxesSize>;
    std::vector<Ai> path;
    TrajGen gen{path};

    PathToTrajectoryConverter_Should() {
        gen.setMaxAcceleration({10, 10});
        gen.setMaxVelocity({20, 20});
    }

    void update() { gen.update(); }
};

TEST_F(PathToTrajectoryConverter_Should, get_trajectory_for_one_axis_and_two_points) {
    path.push_back({0, 0});
    path.push_back({100, 0});

    update();

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{20, 0}));
    EXPECT_THAT(gen.durations(), ElementsAre(5.f));
    EXPECT_THAT(gen.accelerations(), ElementsAre(Af{10, 0}, Af{-10, 0}));
    EXPECT_THAT(gen.blendDurations(), ElementsAre(2.f, 2.f));
}

TEST_F(PathToTrajectoryConverter_Should, get_trajectory_for_two_axes_and_two_points) {
    path.push_back({0, 0});
    path.push_back({100, 200});

    update();

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{10, 20}));
    EXPECT_THAT(gen.durations(), ElementsAre(10.f));
    EXPECT_THAT(gen.accelerations(), ElementsAre(Af{5, 10}, Af{-5, -10}));
    EXPECT_THAT(gen.blendDurations(), ElementsAre(2.f, 2.f));
}

TEST_F(PathToTrajectoryConverter_Should, apply_slowdown) {
    path.push_back({0, 0});
    path.push_back({100, 200});
    gen.setMaxVelocity(Af{50, 50});
    gen.setMaxAcceleration(Af{1, 2});

    update();

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{10, 20}));
    EXPECT_THAT(gen.durations(), ElementsAre(10.f));
    EXPECT_THAT(gen.accelerations(), ElementsAre(Af{1, 2}, Af{-1, -2}));
    EXPECT_THAT(gen.blendDurations(), ElementsAre(10.f, 10.f));
}

TEST_F(PathToTrajectoryConverter_Should, move_to_different_directions) {
    path.push_back({100, 0});
    path.push_back({0, 100});

    update();

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{-20, 20}));
    EXPECT_THAT(gen.durations(), ElementsAre(5.f));
    EXPECT_THAT(gen.accelerations(), ElementsAre(Af{-10, 10}, Af{10, -10}));
    EXPECT_THAT(gen.blendDurations(), ElementsAre(2.f, 2.f));
}

TEST_F(PathToTrajectoryConverter_Should, remove_points_with_all_axes_close_than_threshold) {
    path.push_back({0, 0});
    path.push_back({100, -100});
    path.push_back({108, -104});
    path.push_back({0, 0});

    gen.removeCloseWaypoints({10, 5});

    std::vector<Ai> expected{
        {0, 0}, {104, -102}, {0, 0},
    };
    EXPECT_THAT(path, ContainerEq(expected));
}

TEST_F(PathToTrajectoryConverter_Should, not_remove_points_with_not_all_axes_close_than_threshold) {
    path.push_back({0, 0});
    path.push_back({100, 100});
    path.push_back({111, 100});
    path.push_back({111, 106});
    path.push_back({0, 0});

    gen.removeCloseWaypoints({10, 5});

    EXPECT_THAT(path, ContainerEq(path));
}

TEST_F(PathToTrajectoryConverter_Should, not_change_first_or_last_points) {
    path.push_back({0, 0});
    path.push_back({1, -1});
    path.push_back({99, -99});
    path.push_back({100, -100});

    gen.removeCloseWaypoints({10, 5});

    std::vector<Ai> expected{
        {0, 0}, {100, -100},
    };
    EXPECT_THAT(path, ContainerEq(expected));
}

TEST_F(PathToTrajectoryConverter_Should, corner_case_1) {
    double vMax = 0.5f, aMax = 0.003f;
    int x = 40;
    gen.setMaxVelocity(axConst<Af>(vMax));
    gen.setMaxAcceleration(axConst<Af>(aMax));

    path.push_back({0, 0});
    path.push_back({0, x});

    update();

    auto DT = x / vMax;
    auto tb = vMax / aMax;
    while (tb > DT) {
        auto f = sqrt(DT / tb);
        DT = DT / f;
        vMax = f * vMax;
        tb = vMax / aMax;
    }

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{0, static_cast<float>(vMax)}));
    EXPECT_THAT(gen.durations(), ElementsAre(static_cast<float>(tb)));
    EXPECT_THAT(gen.accelerations(),
                ElementsAre(Af{0, static_cast<float>(aMax)}, Af{0, -static_cast<float>(aMax)}));
    EXPECT_THAT(gen.blendDurations(),
                ElementsAre(FloatEq(static_cast<float>(tb)), FloatEq(static_cast<float>(tb))));
}

TEST_F(PathToTrajectoryConverter_Should, corner_case_2) {
    double vMax = 0.45f, aMax = 0.003f;
    int x = 40;
    gen.setMaxVelocity(axConst<Af>(vMax));
    gen.setMaxAcceleration(axConst<Af>(aMax));

    path.push_back({0, 0});
    path.push_back({0, x});

    update();

    auto DT = x / vMax;
    auto tb = vMax / aMax;
    while (tb > DT) {
        auto f = sqrt(DT / tb);
        DT = DT / f;
        vMax = f * vMax;
        tb = vMax / aMax;
    }

    EXPECT_THAT(gen.velocities(), ElementsAre(Af{0, static_cast<float>(vMax)}));
    EXPECT_THAT(gen.durations(), ElementsAre(static_cast<float>(tb)));
    EXPECT_THAT(gen.accelerations(),
                ElementsAre(Af{0, static_cast<float>(aMax)}, Af{0, -static_cast<float>(aMax)}));
    EXPECT_THAT(gen.blendDurations(),
                ElementsAre(FloatEq(static_cast<float>(tb)), FloatEq(static_cast<float>(tb))));
}
}