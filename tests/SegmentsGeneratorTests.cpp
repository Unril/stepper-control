#include "stdafx.h"

#include "../include/sc/StepperControl.h"

using namespace StepperControl;
using namespace testing;
using namespace std;

namespace {

const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct SegmentsGenerator_Should : Test {
    using SegGen = SegmentsGenerator<AxesSize>;
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

TEST_F(SegmentsGenerator_Should, generate_one_linear_segment) {
    path.push_back({0, 50});
    path.push_back({40, 0});
    gen.setDurations({100.f});
    gen.setBlendDurations({0.f, 0.f});

    update();

    ASSERT_THAT(segments, ElementsAre(Sg(100, {40, -50})));
}

TEST_F(SegmentsGenerator_Should, generate_two_linear_segments) {
    path.push_back({0, 10});
    path.push_back({40, 50});
    path.push_back({100, 110});
    gen.setDurations({100.f, 120.f});
    gen.setBlendDurations({0.f, 0.f, 0.f});

    update();

    ASSERT_THAT(segments, ElementsAre(Sg(100, {40, 40}), Sg(120, {60, 60})));
}

TEST_F(SegmentsGenerator_Should, generate_one_blend_segment_at_beginning) {
    path.push_back({0, 10});
    path.push_back({10, 0});
    gen.setDurations({20.f});
    gen.setBlendDurations({40.f, 0.f});

    update();

    ASSERT_THAT(segments, ElementsAre(Sg(40, {0, 0}, {10, -10})));
}

TEST_F(SegmentsGenerator_Should, generate_one_blend_segment_at_end) {
    path.push_back({10, 0});
    path.push_back({0, 10});
    gen.setDurations({20.f});
    gen.setBlendDurations({0.f, 40.f});

    update();

    ASSERT_THAT(segments, ElementsAre(Sg(40, {-10, 10}, {0, 0})));
}

TEST_F(SegmentsGenerator_Should, generate_two_blend_segments_at_the_middle) {
    path.push_back({0, 0});
    path.push_back({20, 0});
    path.push_back({0, 0});
    gen.setDurations({40.f, 40.f});
    gen.setBlendDurations({0.f, 40.f, 0.f});

    update();

    vector<Sg> expected {
        Sg(20, {10, 0}), Sg(40, {10, 0}, {-10, 0}), Sg(20, {-10, 0})
    };
    ASSERT_THAT(segments, ContainerEq(expected));
}

TEST_F(SegmentsGenerator_Should, generate_two_linear_segments_with_blends) {
    path.push_back({0, 0});
    path.push_back({15, -15});
    path.push_back({5, -5});
    gen.setDurations({30.f, 40.f});
    gen.setBlendDurations({20.f, 20.f, 40.f});

    update();

    vector<Sg> expected {
        Sg(20, {0, 0}, {5, -5}), 
        Sg(10, {5, -5}),
        Sg(20, {5, -5}, {-3, 3}), 
        Sg(10, {-2, 2}),
        Sg(40, {-5, 5}, {0, 0}), 
    };
    ASSERT_THAT(segments, ContainerEq(expected));
}
}