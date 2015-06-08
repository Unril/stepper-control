#include "stdafx.h"

#include "../include/sc/TrajectoryToSegmentsConverter.h"

using namespace StepperControl;
using namespace testing;
using namespace std;

namespace {

const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct TrajectoryToSegmentsConverter_Should : Test {
    using SegGen = TrajectoryToSegmentsConverter<AxesSize>;
    using Sg = Segment<AxesSize>;
    SegGen::Segments segments;
    vector<Ai> path;
    SegGen gen;

    void update() {
        gen.setPath(path);
        gen.update();
        segments = gen.segments();
    }
};

TEST(lTruncTowardZero_Should, trunc_toward_zero) {
    EXPECT_THAT(lTruncTowardZero(2), Eq(2));
    EXPECT_THAT(lTruncTowardZero(1.7f), Eq(1));
    EXPECT_THAT(lTruncTowardZero(1.5f), Eq(1));
    EXPECT_THAT(lTruncTowardZero(1.2f), Eq(1));
    EXPECT_THAT(lTruncTowardZero(0), Eq(0));
    EXPECT_THAT(lTruncTowardZero(-1.2f), Eq(-1));
    EXPECT_THAT(lTruncTowardZero(-1.5f), Eq(-1));
    EXPECT_THAT(lTruncTowardZero(-1.7f), Eq(-1));
    EXPECT_THAT(lTruncTowardZero(-2), Eq(-2));
}

TEST(lTruncTowardInf_Should, trunc_away_from_zero) {
    EXPECT_THAT(lTruncTowardInf(2), Eq(2));
    EXPECT_THAT(lTruncTowardInf(1.7f), Eq(2));
    EXPECT_THAT(lTruncTowardInf(1.5f), Eq(2));
    EXPECT_THAT(lTruncTowardInf(1.2f), Eq(2));
    EXPECT_THAT(lTruncTowardInf(0), Eq(0));
    EXPECT_THAT(lTruncTowardInf(-1.2f), Eq(-2));
    EXPECT_THAT(lTruncTowardInf(-1.5f), Eq(-2));
    EXPECT_THAT(lTruncTowardInf(-1.7f), Eq(-2));
    EXPECT_THAT(lTruncTowardInf(-2), Eq(-2));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_one_linear_segment) {
    path.push_back({0, 50});
    path.push_back({40, 0});
    gen.setDurations({100});
    gen.setBlendDurations({0, 0});

    update();

    ASSERT_THAT(segments, ElementsAre(Sg(100, {40, -50})));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_two_linear_segments) {
    path.push_back({0, 10});
    path.push_back({40, 50});
    path.push_back({100, 110});
    gen.setDurations({100, 120});
    gen.setBlendDurations({0, 0, 0});

    update();

    ASSERT_THAT(segments, ElementsAre(Sg(100, {40, 40}), Sg(120, {60, 60})));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_one_blend_segment_at_beginning) {
    path.push_back({0, 10});
    path.push_back({10, 0});
    gen.setDurations({20});
    gen.setBlendDurations({40, 0});

    update();

    ASSERT_THAT(segments, ElementsAre(Sg(40, {0, 0}, {10, -10})));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_one_blend_segment_at_end) {
    path.push_back({10, 0});
    path.push_back({0, 10});
    gen.setDurations({20});
    gen.setBlendDurations({0, 40});

    update();

    ASSERT_THAT(segments, ElementsAre(Sg(40, {-10, 10}, {0, 0})));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_two_blend_segments_at_the_middle) {
    path.push_back({0, 0});
    path.push_back({20, 0});
    path.push_back({0, 0});
    gen.setDurations({40, 40});
    gen.setBlendDurations({0, 40, 0});

    update();

    vector<Sg> expected{
        /**/ Sg(20, {10, 0}),
        /**/ Sg(40, {10, 0}, {-10, 0}),
        /**/ Sg(20, {-10, 0}),
    };
    ASSERT_THAT(segments, ContainerEq(expected));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_two_linear_segments_with_blends) {
    path.push_back({0, 0});
    path.push_back({15, -15});
    path.push_back({5, -5});
    gen.setDurations({30, 40});
    gen.setBlendDurations({20, 20, 40});

    update();

    vector<Sg> expected{
        /**/ Sg(20, {0, 0}, {5, -5}),
        /**/ Sg(10, {5, -5}),
        /**/ Sg(20, {5, -5}, {-3, 3}),
        /**/ Sg(10, {-2, 2}),
        /**/ Sg(40, {-5, 5}, {0, 0}),
    };
    ASSERT_THAT(segments, ContainerEq(expected));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_blends_with_one_half_ratio) {
    path.push_back({0, 0});
    path.push_back({10, -10});
    path.push_back({0, 0});
    gen.setDurations({40, 40});
    gen.setBlendDurations({20, 20, 20});

    update();

    vector<Sg> expected{
        /**/ Sg(20, {0, 0}, {3, -3}),
        /**/ Sg(20, {4, -4}),
        /**/ Sg(20, {3, -3}, {-3, 3}),
        /**/ Sg(20, {-4, 4}),
        /**/ Sg(20, {-3, 3}, {0, 0}),
    };
    ASSERT_THAT(segments, ContainerEq(expected));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_blends_without_linear_segments) {
    path.push_back({0, 0});
    path.push_back({10, -10});
    path.push_back({0, 0});
    gen.setDurations({40, 40});
    gen.setBlendDurations({60, 20, 60});

    update();

    vector<Sg> expected{
        /**/ Sg(60, {0, 0}, {8, -8}),
        /**/ Sg(20, {3, -3}, {-3, 3}),
        /**/ Sg(60, {-8, 8}, {0, 0}),
    };
    ASSERT_THAT(segments, ContainerEq(expected));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_blends_with_one_third_ratio) {
    path.push_back({0, 0});
    path.push_back({10, -10});
    path.push_back({0, 0});
    gen.setDurations({30, 30});
    gen.setBlendDurations({20, 20, 20});

    update();

    vector<Sg> expected{
        /**/ Sg(20, {0, 0}, {3, -3}),
        /**/ Sg(10, {4, -4}),
        /**/ Sg(20, {3, -3}, {-3, 3}),
        /**/ Sg(10, {-4, 4}),
        /**/ Sg(20, {-3, 3}, {0, 0}),
    };
    ASSERT_THAT(segments, ContainerEq(expected));
}

TEST_F(TrajectoryToSegmentsConverter_Should, generate_short_segments) {
    path.push_back({0, 0});
    path.push_back({10, -10});
    path.push_back({0, 0});
    gen.setDurations({20, 20});
    gen.setBlendDurations({10, 10, 10});

    update();

    vector<Sg> expected{
        /**/ Sg(12, {0, 0}, {3, -3}),
        /**/ Sg(10, {4, -4}),
        /**/ Sg(12, {3, -3}, {-3, 3}),
        /**/ Sg(10, {-4, 4}),
        /**/ Sg(12, {-3, 3}, {0, 0}),
    };
    ASSERT_THAT(segments, ContainerEq(expected));
}
}