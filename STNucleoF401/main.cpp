#include "mbed.h"

#include "FastIO.h"

#include "../include/sc/CommandsExecutor.h"

using namespace StepperControl;

const float Pi = 3.14159265358979323846f;
const int ticksPerSecond = 40000;

static Serial pc(SERIAL_TX, SERIAL_RX);

// Stepper 0
static FastOut<LED1> myled;
//static FastOut<LED1> myled;
static FastIn<USER_BUTTON, PullUp> btn;

struct Motor
{
    template <size_t i, size_t reverse>
    FORCE_INLINE static void writeDirection(UIntConst<i>, UIntConst<reverse>)
    {

    }

    FORCE_INLINE static void writeStep(UIntConst<0>, UIntConst<0>)
    {
        myled.clear();
    }

    FORCE_INLINE static void writeStep(UIntConst<0>, UIntConst<1>)
    {
        myled.set();
    }

    FORCE_INLINE static void update() { }

    FORCE_INLINE static bool checkEndSwitchHit(size_t i)
    {
        switch(i)
        {
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

const size_t axesSize = 1;

static CommandsExecutor<axesSize, Motor, Ticker> executor {10, &motor, &ticker};

static char buffer[128];

int main()
{
    pc.baud(115200);
    printf("Started\n");

    while(true)
    {
        if(pc.readable())
        {
            fgets(buffer, sizeof(buffer), pc);
            executor.executeLine(buffer);
        }
        executor.update();
    }
}
