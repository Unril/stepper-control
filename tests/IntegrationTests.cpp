#include "stdafx.h"

#include "../include/sc/GCodeParser.h"
#include "../include/sc/GCodeInterpreter.h"
#include "../include/sc/PathToTrajectoryConverter.h"
#include "../include/sc/TrajectoryToSegmentsConverter.h"
#include "../include/sc/SegmentsExecutor.h"
#include "../include/sc/CommandsExecutor.h"

#include <fstream>

using namespace StepperControl;
using namespace testing;
using namespace std;

namespace {

const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct MotorMock {
    MotorMock() : current(axZero<Ai>()), dir(axZero<Ai>()) {}

    template <int i, int reverse>
    void writeDirection() {
        dir[i] = (reverse == 0 ? 1 : -1);
    }

    template <int i, int step>
    void writeStep() {
        current[i] += (step ? dir[i] : 0);
    }

    void update() { data.emplace_back(current); }

    void setPosition(Ai const &position) { current = position; }

    bool checkEndSwitchHit(size_t i) {
        return false;
    }

    Ai current;
    Ai dir;
    vector<Ai> data;
};

struct TickerMock {
    static void attach_us(...) {}

    static void detach() {}
};

struct Integration_Should : Test {
    Integration_Should() : parser(&interpreter), executor(&mm, &tm) {}

    GCodeInterpreter<AxesSize> interpreter;
    GCodeParser<AxesSize> parser;
    PathToTrajectoryConverter<AxesSize> trajGen;
    TrajectoryToSegmentsConverter<AxesSize> segGen;
    MotorMock mm;
    TickerMock tm;
    SegmentsExecutor<AxesSize, MotorMock, TickerMock> executor;

    void update() {
        auto cmd = interpreter.commands().back();
        trajGen.setPath(cmd.path());
        trajGen.setMaxVelocity(cmd.maxVelocity());
        trajGen.setMaxAcceleration(cmd.maxAcceleration());
        trajGen.update();

        segGen.setPath(trajGen.path());
        segGen.setBlendDurations(trajGen.blendDurations());
        segGen.setDurations(trajGen.durations());
        segGen.update();

        executor.setSegments(segGen.segments());
        executor.start();
        while (executor.running()) {
            executor.tick();
        }
    }
};

TEST_F(Integration_Should, create_an_execute_trajectory_from_random_path_points) {
    interpreter.m100MaxVelocityOverride({0.5f, 0.1f});
    interpreter.m101MaxAccelerationOverride({0.1f, 0.002f});

    for (int i = 0; i < 20; ++i) {
        stringstream ss;
        ss << "A" << (rand() % 100 - 50) << "B" << (rand() % 60 - 30) << endl;
        parser.parseLine(ss.str().c_str());
    }
    parser.parseLine("A0B0\n");

    update();

    EXPECT_THAT(mm.data.size(), Gt(0));
    EXPECT_THAT(mm.current, Eq(Ai{0, 0}));

#if 0
    ofstream fs("data.csv");
    for (auto &a : mm.data) {
        fs << a[0] << ", " << a[1] << endl;
    }
#endif
}

// cmds.push_back(Cmd({Ai{0}, Ai{10}}, Af{0.5f}, Af{0.025f}));
// EXPECT_THAT(segments, ElementsAre(Seg(20, Ai{0}, Ai{5}), Seg(20, Ai{5}, Ai{0})));

struct CommandsExecutor_Should : Test {
    using Ce = CommandsExecutor<AxesSize, MotorMock, TickerMock>;
    MotorMock mm;
    TickerMock tm;
    Ce ce;

    CommandsExecutor_Should() : ce(10, &mm, &tm) {}
};

TEST_F(CommandsExecutor_Should, execute_move) {
    ce.update();
}
}