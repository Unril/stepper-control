#include "mbed.h"
#include "FastIO.h"

#include <GCodeParser.h>
#include <GCodeInterpreter.h>
#include <SegmentsExecutor.h>

using namespace StepperControl;

struct TestAxesTraits {
    static const int size = 5;
    inline static const char *names() { return "AXYZB"; }
};

static Serial pc(SERIAL_TX, SERIAL_RX);

auto const SwitchPull = PullNone;

// Stepper 0
static FastOut<PC_1> s0Step;
static FastOut<PC_0> s0Dir;
static FastIn<PB_8, SwitchPull> s0Switch;

// Stepper 1
static FastOut<PA_4> s1Step;
static FastOut<PB_0> s1Dir;
static FastIn<PC_9, SwitchPull> s1Switch;

// Stepper 2
static FastOut<PA_0> s2Step;
static FastOut<PA_1> s2Dir;
static FastIn<PC_5, SwitchPull> s2Switch;

// Stepper 2
static FastOut<PC_2> s3Step;
static FastOut<PC_3> s3Dir;
static FastIn<PC_6, SwitchPull> s3Switch;
// Stepper 2
static FastOut<PA_14> s4Step;
static FastOut<PA_15> s4Dir;
static FastIn<PC_8, SwitchPull> s4Switch;

static FastIn<PC_13, PullNone> stopButton; // UserButton pulls up

struct Motor {
    FORCE_INLINE static void writeStep(UIntConst<0>, UIntConst<0>) { s0Step.clear(); }
    FORCE_INLINE static void writeStep(UIntConst<0>, UIntConst<1>) { s0Step.set(); }
    FORCE_INLINE static void writeDirection(UIntConst<0>, UIntConst<0>) { s0Dir.clear(); }
    FORCE_INLINE static void writeDirection(UIntConst<0>, UIntConst<1>) { s0Dir.set(); }

    FORCE_INLINE static void writeStep(UIntConst<1>, UIntConst<0>) { s1Step.clear(); }
    FORCE_INLINE static void writeStep(UIntConst<1>, UIntConst<1>) { s1Step.set(); }
    FORCE_INLINE static void writeDirection(UIntConst<1>, UIntConst<0>) { s1Dir.clear(); }
    FORCE_INLINE static void writeDirection(UIntConst<1>, UIntConst<1>) { s1Dir.set(); }

    FORCE_INLINE static void writeStep(UIntConst<2>, UIntConst<0>) { s2Step.clear(); }
    FORCE_INLINE static void writeStep(UIntConst<2>, UIntConst<1>) { s2Step.set(); }
    FORCE_INLINE static void writeDirection(UIntConst<2>, UIntConst<0>) { s2Dir.clear(); }
    FORCE_INLINE static void writeDirection(UIntConst<2>, UIntConst<1>) { s2Dir.set(); }

    FORCE_INLINE static void writeStep(UIntConst<3>, UIntConst<0>) { s3Step.clear(); }
    FORCE_INLINE static void writeStep(UIntConst<3>, UIntConst<1>) { s3Step.set(); }
    FORCE_INLINE static void writeDirection(UIntConst<3>, UIntConst<0>) { s3Dir.clear(); }
    FORCE_INLINE static void writeDirection(UIntConst<3>, UIntConst<1>) { s3Dir.set(); }

    FORCE_INLINE static void writeStep(UIntConst<4>, UIntConst<0>) { s4Step.clear(); }
    FORCE_INLINE static void writeStep(UIntConst<4>, UIntConst<1>) { s4Step.set(); }
    FORCE_INLINE static void writeDirection(UIntConst<4>, UIntConst<0>) { s4Dir.clear(); }
    FORCE_INLINE static void writeDirection(UIntConst<4>, UIntConst<1>) { s4Dir.set(); }

    FORCE_INLINE static void begin() {}
    FORCE_INLINE static void end() {}

    FORCE_INLINE static bool checkEndSwitchHit(size_t i) {
        switch (i) {
        case 0:
            return s0Switch;
        case 1:
            return s1Switch;
        case 2:
            return s2Switch;
        case 3:
            return s3Switch;
        case 4:
            return s4Switch;
        default:
            return !stopButton;
        }
    }
};

static Motor motor;
static Ticker ticker;

int32_t getTicksPerSecond(unsigned sz) {
    switch (sz) {
    case 1:
    case 2:
        return 160000;
    case 3:
    case 4:
        return 140000;
    case 5:
    case 6:
        return 120000;
    default:
        scAssert(!"Wrong axes count.");
        return 0;
    }
}

const unsigned axesSize = TestAxesTraits::size;
const int32_t ticksPerSecond = getTicksPerSecond(axesSize);
const float Pi = 3.14159265358979323846f;
const int notifyPositionIntervalMs = 200;

static SegmentsExecutor<Motor, Ticker, TestAxesTraits> executor(&motor, &ticker);
static GCodeInterpreter<TestAxesTraits> interpreter(&executor);
static GCodeParser<TestAxesTraits> parser(&interpreter);

static char buffer[128];

static Timer timer;

static bool justStopped = false;
static void onStopped(void *) { justStopped = true; }

static void execute() {
    executor.setOnStopped(&onStopped, nullptr);

    interpreter.setTicksPerSecond(ticksPerSecond);

    auto perRot = 1.f / (2.f * Pi);
    auto spu = 10000.f / 5.f;
    interpreter.m102StepsPerUnitLengthOverride({10*64*200*perRot, spu, spu, spu, 20*64*200*perRot});

    auto rotVel = 20.f;
    auto rotAcc = 20.f;
    interpreter.m100MaxVelocityOverride({0.5, rotVel, rotVel, rotVel, 1});
    interpreter.m101MaxAccelerationOverride({1, rotAcc, rotAcc, rotAcc, 2});

    timer.start();

    while (true) {
        if (!stopButton && interpreter.isRunning()) {
            interpreter.stop();
        }

        if (pc.readable()) {
            fgets(buffer, sizeof(buffer), pc);
            parser.parseLine(buffer);
        }

        if (timer.read_ms() >= notifyPositionIntervalMs) {
            timer.reset();

            if (executor.isRunning() && !executor.isHoming()) {
                interpreter.printCurrentPosition();
            }
        }

        if (justStopped) {
            justStopped = false;

            interpreter.printCompleted();
        }
    }
}

void buttonTest() {
    bool prevState[5] = {};
    while (true) {
        for (int i = 0; i <= 5; ++i) {
            bool newState = motor.checkEndSwitchHit(i);
            if (prevState[i] != newState) {
                printf("Switch %d: %d -> %d\r\n", i, prevState[i], newState);
                prevState[i] = newState;
            }
        }
    }
}

//#define BUTTON_TEST

int main() {
    pc.baud(115200);
    printf("Started\n");

#ifdef BUTTON_TEST
    buttonTest();
#else
    execute();
#endif
}
