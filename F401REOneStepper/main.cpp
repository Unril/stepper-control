#include "mbed.h"
#include "FastIO.h"
#include <atomic>
#include <cstdio>

#include <GCodeInterpreter.h>
#include <GCodeParser.h>
#include <SegmentsExecutor.h>

using namespace StepperControl;

struct TestAxesTraits {
    static const int size = 1;
    inline static const char *names() { return "X"; }
};

/*

PC_10 s4Step     PC_11                  PC_9 s1Switch    PC_8 s4Switch
PC_12 s4Dir      PD_2                   PB_8 s0Switch    PC_6 s3Switch
VDD              E5V                    PB_9             PC_5 s2Switch
BOOT0            GND                    AVDD             U5V
NC               NC                     GND              NC
NC               IOREF                  PA_5 LED         PA_12
STLink           RESET                  PA_6             PA_11
STLink           +3V3                   PA_7             PB_12
PA_15            +5V                    PB_6             NC
GND              GND                    PC_7             GND
PB_7             GND                    PA_9             PB_2
PC_13 stopSwitch VIN                    PA_8             PB_1
NC               NC                     PB_10            PB_15
NC               PA_0 s2Step            PB_4             PB_14
NC               PA_1 s2Dir             PB_5             PB_13
PH_1             PA_4 s1Step            PB_3             AGND
VBAT             PB_0 s1Dir             PA_10            PC_4
PC_2 s3Step      PC_1 s0Step            PA_2 SERIAL_TX   NC
PC_3 s3Dir       PC_0 s0Dir             PA_3 SERIAL_RX   NC

*/

static Serial pc(SERIAL_TX, SERIAL_RX);

template <unsigned i>
struct Stepper {};

static FastOut<PA_0> Step0;
static FastOut<PA_1> Dir0;
static FastIn<A5, PullDown
> Switch0;

static FastIn<USER_BUTTON> stopSwitch; // UserButton pulls up
static FastOut<LED1> led;

static const int bufferSize = 512;
static char buffer[bufferSize];

struct SerialPrinter : Printer {
    void print(const float *n, int size) override {
        if (size == 5) {
            pc.printf("%f, %f, %f, %f, %f", n[0], n[1], n[2], n[3], n[4]);
        } else if (size == 1) {
            pc.printf("%f", n[0]);
        } else {
            for (int i = 0; i < size; i++) {
                pc.printf("%f%s", n[i], sep(i, size));
            }
        }
    }

    void print(const int32_t *n, int size) override {
        if (size == 5) {
            pc.printf("%ld, %ld, %ld, %ld, %ld", n[0], n[1], n[2], n[3], n[4]);
        } else if (size == 1) {
            pc.printf("%ld", *n);
        } else {
            for (int i = 0; i < size; i++) {
                pc.printf("%ld%s", n[i], sep(i, size));
            }
        }
    }

    void print(const char *str) override { pc.puts(str); }
} static serialPrinter;

struct Motor {
    FORCE_INLINE void writeStep(StepperNumber<0>, bool lvl) { Step0 = lvl; }
    FORCE_INLINE void writeDirection(StepperNumber<0>, bool dir) { Dir0 = dir; }

    FORCE_INLINE void begin() {}
    FORCE_INLINE void end() {}

    FORCE_INLINE bool checkEndSwitchHit(size_t i) {
        switch (i) {
        case 0:
            return Switch0;
        default:
            return !stopSwitch;
        }
    }
} static motor;

static Ticker ticker;

static const unsigned axesSize = TestAxesTraits::size;
static const int ticksPerSecond = 120 * 1000;
static const float Pi = 3.14159265358979323846f;
static const int notifyPositionIntervalMs = 200;

using Executor = SegmentsExecutor<Motor, Ticker, TestAxesTraits>;
using Interpreter = GCodeInterpreter<Executor, TestAxesTraits>;
using Parser = GCodeParser<Interpreter, TestAxesTraits>;

static Executor executor(&motor, &ticker);
static Interpreter interpreter(&executor, &serialPrinter);
static Parser parser(&interpreter);

static void onStarted(void *) {
    led.set();
}

static std::atomic<bool> stopped{false};

static void onStopped(void *) {
    led.clear();
    stopped = true;
}

static void execute() {
    executor.setOnStopped(&onStopped, nullptr);
    executor.setOnStarted(&onStarted, nullptr);

    interpreter.setTicksPerSecond(ticksPerSecond);

    auto perRot = 1.f / (2.f * Pi);
    auto perRotPrism = 1.f / 5.f;
    interpreter.m102StepsPerUnitLengthOverride({8000 * perRotPrism});

    Interpreter::Af v = {20};
    interpreter.m100MaxVelocityOverride(v);
    interpreter.m101MaxAccelerationOverride({40});
    interpreter.m103HomingVelocityOverride(v * 0.5f);

    int32_t notifyInterval = ticksPerSecond * notifyPositionIntervalMs / 1000;
    int32_t nextNotifyTick = notifyInterval;

    while (true) {
        while (pc.readable()) {
            if (fgets(buffer, bufferSize, pc)) {
                parser.parseLine(buffer);
            }
        }

        if (executor.currentTick() >= nextNotifyTick) {
            interpreter.printCurrentPosition();
            nextNotifyTick = executor.currentTick() + notifyInterval;
        }

        if (stopped == true) {
            stopped = false;
            nextNotifyTick = notifyInterval;
            interpreter.printCompleted();
        }
    }
}

void buttonTest() {
    bool prevState[5] = {};
    while (true) {
        if (pc.readable()) {
            fgets(buffer, sizeof(buffer), pc);
            for (int i = 0; i <= 5; ++i) {
                bool newState = motor.checkEndSwitchHit(i);
                printf("Switch %d: %d\r\n", i, newState);
            }
        }

        for (int i = 0; i <= 5; ++i) {
            bool newState = motor.checkEndSwitchHit(i);
            if (prevState[i] != newState) {
                printf("Switch %d: %d -> %d\r\n", i, prevState[i], newState);
                prevState[i] = newState;
            }
        }
    }
}

int main() {
    pc.baud(9600);

    pc.printf("SystemCoreClock = %d Hz\n\r", HAL_RCC_GetSysClockFreq());

    pc.printf("Started\n");

    execute();
}
