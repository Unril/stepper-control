#define MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE 115200

#include "mbed.h"
#include "FastIO.h"

#include <cstdio>
#include <vector>

#include <GCodeInterpreter.h>
#include <GCodeParser.h>
#include <SegmentsExecutor.h>

using namespace StepperControl;

struct TestAxesTraits {
	static const int size = 2;
	inline static const char *names() {
		return "AX";
	}
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

template<unsigned i>
struct Stepper {
};

static FastOut<PC_1> Step0;
static FastOut<PC_0> Dir0;
static FastIn<PB_8, PullDown> Switch0;

static FastOut<PA_4> Step1;
static FastOut<PB_0> Dir1;
static FastIn<PC_9, PullDown> Switch1;

static FastOut<PA_0> Step2;
static FastOut<PA_1> Dir2;
static FastIn<PC_5, PullDown> Switch2;

static FastOut<PC_2> Step3;
static FastOut<PC_3> Dir3;
static FastIn<PC_6, PullDown> Switch3;

static FastOut<PC_10> Step4;
static FastOut<PC_12> Dir4;
static FastIn<PC_8, PullDown> Switch4;

static FastIn<PC_13> stopSwitch; // UserButton pulls up
static FastOut<LED1> led;

static const int bufferSize = 512;
static char buffer[bufferSize];

struct SerialPrinter: Printer {
	void print(const float *n, int size) override {
		if (size == 5) {
			pc.printf("%f, %f, %f, %f, %f", n[0], n[1], n[2], n[3], n[4]);
		} else if (size == 2) {
			pc.printf("%f, %f", n[0], n[1]);
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
		} else if (size == 2) {
			pc.printf("%ld, %ld", n[0], n[1]);
		} else if (size == 1) {
			pc.printf("%ld", *n);
		} else {
			for (int i = 0; i < size; i++) {
				pc.printf("%ld%s", n[i], sep(i, size));
			}
		}
	}

	void print(const char *str) override {
		pc.puts(str);
	}
}static serialPrinter;

struct Motor {
	FORCE_INLINE void writeStep(StepperNumber<0>, bool lvl) {
		Step0 = lvl;
	}
	FORCE_INLINE void writeStep(StepperNumber<1>, bool lvl) {
		Step1 = lvl;
	}
	FORCE_INLINE void writeStep(StepperNumber<2>, bool lvl) {
		Step2 = lvl;
	}
	FORCE_INLINE void writeStep(StepperNumber<3>, bool lvl) {
		Step3 = lvl;
	}
	FORCE_INLINE void writeStep(StepperNumber<4>, bool lvl) {
		Step4 = lvl;
	}
	FORCE_INLINE void writeDirection(StepperNumber<0>, bool dir) {
		Dir0 = !dir;
	}
	FORCE_INLINE void writeDirection(StepperNumber<1>, bool dir) {
		Dir1 = !dir;
	}
	FORCE_INLINE void writeDirection(StepperNumber<2>, bool dir) {
		Dir2 = dir;
	}
	FORCE_INLINE void writeDirection(StepperNumber<3>, bool dir) {
		Dir3 = !dir;
	}
	FORCE_INLINE void writeDirection(StepperNumber<4>, bool dir) {
		Dir4 = !dir;
	}

	FORCE_INLINE void begin() {
	}
	FORCE_INLINE void end() {
	}

	FORCE_INLINE bool checkEndSwitchHit(size_t i) {
		return Switch1.read();
		switch (i) {
		case 0:
			return Switch0.read();
		case 1:
			return Switch1.read();
		case 2:
			return Switch2.read();
		case 3:
			return Switch3.read();
		case 4:
			return Switch4.read();
		default:
			return !stopSwitch;
		}
	}
}static motor;

static Ticker ticker;

constexpr unsigned axesSize = TestAxesTraits::size;
constexpr float Pi = 3.14159265358979323846f;
constexpr int notifyPositionIntervalMs = 200;

using Executor = SegmentsExecutor<Motor, Ticker, TestAxesTraits>;
using Interpreter = GCodeInterpreter<Executor, TestAxesTraits>;
using Parser = GCodeParser<Interpreter, TestAxesTraits>;

static Executor executor(&motor, &ticker);
static Interpreter interpreter(&executor, &serialPrinter);
static Parser parser(&interpreter);

static void onStarted(void *) {
	led.set();
}

static volatile bool stopped { false };

static void onStopped(void *) {
	led.clear();
	stopped = true;
}

static void execute() {
	executor.setOnStopped(&onStopped, nullptr);
	executor.setOnStarted(&onStarted, nullptr);

	constexpr int ticksPerSecond = 100 * 1000;
	interpreter.setTicksPerSecond(ticksPerSecond);

	auto reduction = 1.f/1.f; // 4.8
	auto perRot = 2.f * reduction * 1.f / (2.f * Pi);
	auto perRotPrism = 1.f / 5.f;
	Interpreter::Af spu { 12800 * perRot, 12800 * perRotPrism, };
    Interpreter::Af v { 0.66f, 20.f, };
	Interpreter::Af a { 3.0f, 100.f, };

	interpreter.m102StepsPerUnitLengthOverride(spu);
	interpreter.m100MaxVelocityOverride(v);
	interpreter.m101MaxAccelerationOverride(a);
	interpreter.m103HomingVelocityOverride(v*0.5f);

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

// Full step from zero rounding
// TODO: check step loss.
// TODO: add test switch command
int main() {
	pc.baud(MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE);

	pc.printf("SystemCoreClock = %d Hz\n\r", HAL_RCC_GetSysClockFreq());

	pc.printf("Started\n");

	execute();
	return 0;
}
