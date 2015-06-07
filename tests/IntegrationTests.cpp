#include "stdafx.h"

#include "../include/sc/GCodeParser.h"
#include "../include/sc/GCodeInterpreter.h"
#include "../include/sc/TrajectoryGenerator.h"
#include "../include/sc/StepperControl.h"
#include <fstream>

using namespace StepperControl;
using namespace testing;
using namespace std;

namespace {

const size_t AxesSize = 2;

using Af = Axes<float, AxesSize>;
using Ai = Axes<int32_t, AxesSize>;

struct MotorMock {
    MotorMock() : current(axZero<Ai>()) {}

    template <int i, int step>
    void write() {
        current[i] += step;
    }

    void update() { data.emplace_back(current); }

    void setPosition(Ai const &position) { current = position; }

    Ai current;
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
    TrajectoryGenerator<AxesSize> trajGen;
    SegmentsGenerator<AxesSize> segGen;
    MotorMock mm;
    TickerMock tm;
    SegmentsExecutor<AxesSize, MotorMock, TickerMock> executor;

    void update() {
        trajGen.setAll(interpreter);
        trajGen.update();
        segGen.setAll(trajGen);
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
        ss << "A" << (rand() % 100 - 50) << "B" << (rand() % 60 - 30)<< endl;
        parser.parseLine(ss.str().c_str());
    }
    parser.parseLine("A0B0\n");

    update();

    EXPECT_THAT(mm.data.size(), Gt(0));
    EXPECT_THAT(mm.current, Eq(Ai{0, 0}));

    //ofstream fs("data.csv");
    //for (auto &a : mm.data) {
    //    fs << a[0] << ", " << a[1] << endl;
    //}
}
}