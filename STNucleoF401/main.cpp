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

// Stepper 0
static FastOut<PA_4> s0Step;
static FastOut<PB_0> s0Dir;
static FastIn<USER_BUTTON, PullUp> btn;

// Stepper 1
static FastOut<PA_0> s1Step;
static FastOut<PA_1> s1Dir;

// Stepper 2
static FastOut<PC_2> s2Step;
static FastOut<PC_3> s2Dir;

static FastIn<D12> stopButton;

struct Motor {
    template <unsigned i, unsigned edge>
    FORCE_INLINE static void writeStep(UIntConst<i>, UIntConst<edge>) {}

    template <unsigned i, unsigned dir>
    FORCE_INLINE static void writeDirection(UIntConst<i>, UIntConst<dir>) {}

    // Stepper 0

    FORCE_INLINE static void writeStep(UIntConst<1>, UIntConst<0>) { s0Step.clear(); }

    FORCE_INLINE static void writeStep(UIntConst<1>, UIntConst<1>) { s0Step.set(); }

    FORCE_INLINE static void writeDirection(UIntConst<1>, UIntConst<0>) { s0Dir.clear(); }

    FORCE_INLINE static void writeDirection(UIntConst<1>, UIntConst<1>) { s0Dir.set(); }

    // Stepper 1

    FORCE_INLINE static void writeStep(UIntConst<2>, UIntConst<0>) { s1Step.clear(); }

    FORCE_INLINE static void writeStep(UIntConst<2>, UIntConst<1>) { s1Step.set(); }

    FORCE_INLINE static void writeDirection(UIntConst<2>, UIntConst<0>) { s1Dir.clear(); }

    FORCE_INLINE static void writeDirection(UIntConst<2>, UIntConst<1>) { s1Dir.set(); }

    // Stepper 2

    FORCE_INLINE static void writeStep(UIntConst<3>, UIntConst<0>) { s2Step.clear(); }

    FORCE_INLINE static void writeStep(UIntConst<3>, UIntConst<1>) { s2Step.set(); }

    FORCE_INLINE static void writeDirection(UIntConst<3>, UIntConst<0>) { s2Dir.clear(); }

    FORCE_INLINE static void writeDirection(UIntConst<3>, UIntConst<1>) { s2Dir.set(); }

    // Tick pin

    FORCE_INLINE static void begin() {}

    FORCE_INLINE static void end() {}

    FORCE_INLINE static bool checkEndSwitchHit(size_t i) {
        return !btn;
        switch (i) {
        case 0:
            return !btn;
        case 1:

            break;
        case 2:

            break;
        case 3:

            break;
        case 4:

            break;
        }
        return false;
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
void onStopped(void *) { justStopped = true; }

int main() {
    pc.baud(115200);
    printf("Started\n");

    executor.setOnStopped(&onStopped, nullptr);

    interpreter.setTicksPerSecond(ticksPerSecond);

    interpreter.m102StepsPerUnitLengthOverride(axConst<axesSize>(3200.f));

    auto perRot = 1.f / (2.f * Pi);
    interpreter.m102StepsPerUnitLengthOverride({20 * 6400 * perRot, 6400.f / 5, 20 * 6400 * perRot,
                                                10 * 6400 * perRot, 10 * 6400 * perRot});

    auto rotVel = 0.5f;
    auto rotAcc = 0.5f;
    interpreter.m100MaxVelocityOverride({rotVel, 50.f, rotVel, rotVel, rotVel});
    interpreter.m101MaxAccelerationOverride({rotAcc, 100.f, rotAcc, rotAcc, rotAcc});

    timer.start();

    while (true) {
        if (stopButton.read()) {
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
