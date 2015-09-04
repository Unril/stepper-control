#include "stdafx.h"

#include "../include/sc/GCodeParser.h"
#include "../include/sc/GCodeInterpreter.h"
#include "../include/sc/SegmentsExecutor.h"

#include <fstream>

using namespace StepperControl;
using namespace testing;
using namespace std;

namespace {

struct AxTr {
    static const int size = 2;
    static const char *names() { return "AB"; }
};

using Af = TAf<AxTr::size>;
using Ai = TAi<AxTr::size>;

struct MotorMock {
    MotorMock() : current(axZero<Ai>()), dir(axZero<Ai>()) {}

    template <int i, int reverse>
    void writeDirection(UIntConst<i>, UIntConst<reverse>) {
        dir[i] = (reverse == 0 ? 1 : -1);
    }

    template <int i, int edge>
    void writeStep(UIntConst<i>, UIntConst<edge>) {
        current[i] += (edge ? dir[i] : 0);
    }

    static void begin() {}
    void end() { data.emplace_back(current); }

    void setPosition(Ai const &position) { current = position; }

    static bool checkEndSwitchHit(size_t i) { return false; }

    Ai current;
    Ai dir;
    vector<Ai> data;
};

struct TickerMock {
    static void attach_us(...) {}

    static void detach() {}
};

struct Integration_Should : Test {
    MotorMock mm;
    TickerMock tm;
    SegmentsExecutor<MotorMock, TickerMock, AxTr> executor;
    GCodeInterpreter<AxTr> interpreter;
    GCodeParser<AxTr> parser;

    Integration_Should() : executor(&mm, &tm), interpreter(&executor), parser(&interpreter) {
        interpreter.setTicksPerSecond(100000);
    }

    void update() {
        interpreter.start();
        while (executor.isRunning()) {
            executor.tick();
        }
    }
};

TEST_F(Integration_Should, create_an_execute_trajectory_from_random_path_points) {
    interpreter.m100MaxVelocityOverride(axConst<Af>(30.f));
    interpreter.m101MaxAccelerationOverride(axConst<Af>(100.f));

    for (int i = 0; i < 2; ++i) {
        stringstream ss;
        ss << "A" << rand() % 50 << "B" << rand() % 10 << endl;
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
}