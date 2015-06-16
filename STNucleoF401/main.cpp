#include "mbed.h"
#include "FastIO.h"

#include <GCodeParser.h>
#include <GCodeInterpreter.h>
#include <SegmentsExecutor.h>

using namespace StepperControl;

static Serial pc(SERIAL_TX, SERIAL_RX);

// Stepper 0
static FastOut<A0> s0Step;
static FastOut<A1> s0Dir;
static FastIn<USER_BUTTON, PullUp> btn;

// Stepper 1
static FastOut<A2> s1Step;
static FastOut<A3> s1Dir;

// Stepper 2
static FastOut<A4> s2Step;
static FastOut<A5> s2Dir;

// Tick pin.
static FastOut<D12> tickPin;

struct Motor {
    template <size_t i, size_t edge>
    FORCE_INLINE static void writeStep(UIntConst<i>, UIntConst<edge>) {
        __NOP();
        __NOP();
    }

    template <size_t i, size_t dir>
    FORCE_INLINE static void writeDirection(UIntConst<i>, UIntConst<dir>) {
        __NOP();
        __NOP();
    }

    // Stepper 0

    FORCE_INLINE static void writeStep(UIntConst<0>, UIntConst<0>) { s0Step.clear(); }

    FORCE_INLINE static void writeStep(UIntConst<0>, UIntConst<1>) { s0Step.set(); }

    FORCE_INLINE static void writeDirection(UIntConst<0>, UIntConst<0>) { s0Dir.clear(); }

    FORCE_INLINE static void writeDirection(UIntConst<0>, UIntConst<1>) { s0Dir.set(); }

    // Stepper 1

    FORCE_INLINE static void writeStep(UIntConst<1>, UIntConst<0>) { s1Step.clear(); }

    FORCE_INLINE static void writeStep(UIntConst<1>, UIntConst<1>) { s1Step.set(); }

    FORCE_INLINE static void writeDirection(UIntConst<1>, UIntConst<0>) { s1Dir.clear(); }

    FORCE_INLINE static void writeDirection(UIntConst<1>, UIntConst<1>) { s1Dir.set(); }

    // Stepper 2

    FORCE_INLINE static void writeStep(UIntConst<2>, UIntConst<0>) { s2Step.clear(); }

    FORCE_INLINE static void writeStep(UIntConst<2>, UIntConst<1>) { s2Step.set(); }

    FORCE_INLINE static void writeDirection(UIntConst<2>, UIntConst<0>) { s2Dir.clear(); }

    FORCE_INLINE static void writeDirection(UIntConst<2>, UIntConst<1>) { s2Dir.set(); }

    // Tick pin

    FORCE_INLINE static void begin() { tickPin.set(); }

    FORCE_INLINE static void end() { tickPin.clear(); }

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

const float Pi = 3.14159265358979323846f;
const int32_t ticksPerSecond = 120000;
const int32_t notifyPositionIntervalMs = 200;
const size_t axesSize = 5;

static SegmentsExecutor<axesSize, Motor, Ticker> executor(&motor, &ticker);
static GCodeInterpreter<axesSize> interpreter(&executor);
static GCodeParser<axesSize> parser(&interpreter);

static char buffer[128];

static Timer timer;

static bool justStopped = false;
void onStopped(void *) { justStopped = true; }

int main() {
    pc.baud(115200);
    printf("Started\n");

    executor.setOnStopped(&onStopped, nullptr);

    interpreter.setTicksPerSecond(ticksPerSecond);

    auto perRot = 1.f / (2.f * Pi);
    interpreter.m102StepsPerUnitLengthOverride({1600.f / 5, 20 * 1600 * perRot, 10 * 1600 * perRot,
                                                10 * 1600 * perRot, 10 * 1600 * perRot});

    auto rotVel = 0.3f;
    auto rotAcc = 0.1f;
    interpreter.m100MaxVelocityOverride({30.f, rotVel, rotVel, rotVel, rotVel});
    interpreter.m101MaxAccelerationOverride({20.f, rotAcc, rotAcc, rotAcc, rotAcc});

    timer.start();

    while (true) {
        if (pc.readable()) {
            fgets(buffer, sizeof(buffer), pc);
            parser.parseLine(buffer);
        }

        if (timer.read_ms() >= notifyPositionIntervalMs) {
            timer.reset();

            if (executor.isRunning() && !executor.isHoming()) {
                axPrintf(interpreter.toUnits(executor.position()));
                printf("\n");
            }
        }

        if (justStopped) {
            justStopped = false;

            axPrintf(interpreter.toUnits(executor.position()));
            printf("\nCompleted\n");
        }
    }
}
