#include "stdafx.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "../include/sc/GCodeInterpreter.h"
#include "../include/sc/GCodeParser.h"
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

    template <int i>
    void writeDirection(StepperNumber<i>, bool reverse) {
        dir[i] = (reverse == 0 ? 1 : -1);
    }

    template <int i>
    void writeStep(StepperNumber<i>, bool edge) {
        current[i] += (edge ? dir[i] : 0);
    }

    void begin() {}

    void end() { data.emplace_back(current); }

    void setPosition(Ai const &position) { current = position; }
    
    bool checkEndSwitchHit(size_t i) {       
        current[i] = 0; 
        return true;
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
    MotorMock mm;
    TickerMock tm;
    using Exec = SegmentsExecutor<MotorMock, TickerMock, AxTr>;
    using Interp = GCodeInterpreter<Exec, AxTr>;
    Exec executor;
    Interp interpreter;
    GCodeParser<Interp, AxTr> parser;

    Integration_Should() : executor(&mm, &tm), interpreter(&executor), parser(&interpreter) {
        interpreter.setTicksPerSecond(10000);        
    interpreter.m100MaxVelocityOverride(axConst<Af>(30.f));
    interpreter.m103HomingVelocityOverride(axConst<Af>(30.f));
    interpreter.m101MaxAccelerationOverride(axConst<Af>(100.f));
    }

    void update() {
        interpreter.start();
        while (executor.isRunning()) {
            executor.tick();
        }
    }
};

TEST_F(Integration_Should, create_an_execute_trajectory_from_random_path_points) {
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

TEST_F(Integration_Should, move_after_homing) {
    executor.setPosition({10, 0});

    parser.parseLine("A40\n");
    parser.parseLine("G28\n");
    parser.parseLine("A30\n");

    update();

    EXPECT_THAT(mm.current, Eq(Ai{30, 0}));
}

TEST_F(Integration_Should, move_after_waiting) {   
    parser.parseLine("A10\n");
    parser.parseLine("G4 P1\n");
    parser.parseLine("A20\n");
    parser.parseLine("G4 P0\n");
    parser.parseLine("A30\n");

    update();

    EXPECT_THAT(mm.current, Eq(Ai{30, 0}));
}

TEST_F(Integration_Should, move_to_zero_without_spaces) {   
    parser.parseLine("a2.1b2.1\n");
    parser.parseLine("a0.0b0.0\n");
    parser.parseLine("a2b2\n");
    parser.parseLine("a0b0\n");

    update();

    EXPECT_THAT(mm.current, Eq(Ai{0, 0}));
}
}