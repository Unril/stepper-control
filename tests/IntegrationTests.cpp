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
//
//const size_t AxesSize = 2;
//
//using Af = Axes<float, AxesSize>;
//using Ai = Axes<int32_t, AxesSize>;
//
//struct MotorMock {
//    MotorMock() : current(axZero<Ai>()), dir(axZero<Ai>()) {}
//
//    template <int i, int reverse>
//    void writeDirection(UIntConst<i>, UIntConst<reverse>) {
//        dir[i] = (reverse == 0 ? 1 : -1);
//    }
//
//    template <int i, int edge>
//    void writeStep(UIntConst<i>, UIntConst<edge>) {
//        current[i] += (edge ? dir[i] : 0);
//    }
//
//    static void begin() { }
//    void end() { data.emplace_back(current); }
//
//    void setPosition(Ai const &position) { current = position; }
//
//    static bool checkEndSwitchHit(size_t i) {
//        return false;
//    }
//
//    Ai current;
//    Ai dir;
//    vector<Ai> data;
//};
//
//struct TickerMock {
//    static void attach_us(...) {}
//
//    static void detach() {}
//};
//
//struct Integration_Should : Test {
//    GCodeInterpreter<AxesSize> interpreter;
//    GCodeParser<AxesSize> parser;
//    PathToTrajectoryConverter<AxesSize> trajGen;
//    TrajectoryToSegmentsConverter<AxesSize> segGen;
//    MotorMock mm;
//    TickerMock tm;
//    SegmentsExecutor<AxesSize, MotorMock, TickerMock> executor;
//
//    Integration_Should() : parser(&interpreter), executor(&mm, &tm) {
//        auto ticksPerSecond = 100000;
//        interpreter.setTicksPerSecond(ticksPerSecond);
//        executor.setTicksPerSecond(ticksPerSecond);
//    }
//
//    void update() {
//        auto cmd = interpreter.commands().back();
//        trajGen.setPath(cmd.path());
//        trajGen.setMaxVelocity(cmd.maxVelocity());
//        trajGen.setMaxAcceleration(cmd.maxAcceleration());
//        trajGen.update();
//
//        segGen.setPath(trajGen.path());
//        segGen.setBlendDurations(trajGen.blendDurations());
//        segGen.setDurations(trajGen.durations());
//        segGen.update();
//
//        executor.setSegments(segGen.segments());
//        executor.start();
//        while (executor.running()) {
//            executor.tick();
//        }
//    }
//};
//
//TEST_F(Integration_Should, create_an_execute_trajectory_from_random_path_points) {
//   interpreter.m100MaxVelocityOverride(axConst<Af>(30.f));
//   interpreter.m101MaxAccelerationOverride(axConst<Af>(100.f));
//
//    for (int i = 0; i < 2; ++i) {
//        stringstream ss;
//        ss << "A" << rand() % 50 << "B" << rand() % 10 << endl;
//        parser.parseLine(ss.str().c_str());
//    }
//    parser.parseLine("A0B0\n");
//
//    update();
//
//    EXPECT_THAT(mm.data.size(), Gt(0));
//    EXPECT_THAT(mm.current, Eq(Ai{0, 0}));
//
//#if 0
//    ofstream fs("data.csv");
//    for (auto &a : mm.data) {
//        fs << a[0] << ", " << a[1] << endl;
//    }
//#endif
//}

// cmds.push_back(Cmd({Ai{0}, Ai{10}}, Af{0.5f}, Af{0.025f}));
// EXPECT_THAT(segments, ElementsAre(Seg(20, Ai{0}, Ai{5}), Seg(20, Ai{5}, Ai{0})));
//
//struct CommandsExecutor_Should : Test {
//    using Ce = CommandsExecutor<AxesSize, MotorMock, TickerMock>;
//    MotorMock mm;
//    TickerMock tm;
//    Ce ce;
//
//    CommandsExecutor_Should() : ce(10, &mm, &tm) {}
//};
//
//TEST_F(CommandsExecutor_Should, execute_move) {
//    ce.update();
//}
}