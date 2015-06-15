#pragma once

#include "Axes.h"

namespace StepperControl {

const int32_t int32Max = std::numeric_limits<int32_t>::max();
const int64_t int64Max = std::numeric_limits<int64_t>::max();

// Contains data for Bresenham's algorithm.
template <size_t AxesSize>
struct Segment {
    using Ai = Axes<int32_t, AxesSize>;
    using Al = Axes<int64_t, AxesSize>;

    // Homing segment.
    // Linear motion for max time at constant homing velocity.
    // Duration is set to negative number to determine this type of segment.
    Segment(Axes<float, AxesSize> const &homingVelocity) {
        auto dtL = static_cast<int64_t>(int32Max);
        auto dx = dtL / axCast<int64_t>(-1.f / homingVelocity);

        // dx <= dt/2
        scAssert(all(le(axAbs(dx) * 2, axConst<Al>(dtL))));

        dt = -1;
        denominator = 2 * dtL;
        velocity = 2 * dx;
        acceleration.fill(0);
        error.fill(0);
    }

    // Wait segment.
    // dt - wait duration in ticks.
    Segment(int32_t dt) : dt(dt) {
        scAssert(dt >= 0);

        denominator = 1;
        velocity.fill(0);
        acceleration.fill(0);
        error.fill(0);
    }

    /* Linear segment.
       It is set by start and end points p0-p1.
    x  ^
       |
   x1  +-------p1
       |     / |    dt = t1 - t0, ticks
       |   /   |    dx = x1 - x0, steps
       | /     |
   x0  p0------+---> t
       t0        t1
    */
    Segment(int32_t dt, Ai const &dx) : dt(dt) {
        auto dtL = static_cast<int64_t>(dt);

        // Overwflow check.
        scAssert(all(le(axCast<int64_t>(axAbs(dx)), axConst<Al>(int64Max / 2))));

        scAssert(dt > 0);
        // dx <= dt/2
        scAssert(all(le(axAbs(dx) * 2, axConst<Ai>(dt))));

        denominator = 2 * dtL;
        velocity = 2 * axCast<int64_t>(dx);
        acceleration.fill(0);
        error.fill(0);
    }

    /* Parabolic segment.
       It is set by endpoints of two tangent to parabola segments p0-p1 and p1-p2.
    x  ^
   x1  +----p1         twiceDt = t2 - t0, ticks
       |   /| \        dx1 = x1 - x0, steps
       |  / |   \      dx2 = x2 - x1, steps
   x2  +-/--|-----p2
       |/   |     |
   x0  p0---+-----+---> t
       t0   t1    t2
    */
    Segment(int32_t twiceDt, Ai const &dx1, Ai const &dx2) : dt(twiceDt) {
        auto twiceDtL = static_cast<int64_t>(twiceDt);
        auto halfA = (dx2 - dx1);

        // Overwflow check.
        scAssert(twiceDtL <= int64Max / twiceDtL);
        scAssert(all(le(axCast<int64_t>(axAbs(dx1)), axConst<Al>(int64Max / (2 * twiceDtL)))));
        scAssert(all(le(axCast<int64_t>(axAbs(dx2)), axConst<Al>(int64Max / (2 * twiceDtL)))));
        scAssert(all(le(axAbs(halfA), axConst<Ai>(int32Max / 2))));

        scAssert(twiceDt > 0);
        // dx1 <= dt1/2 && dx2 <= dt2/2
        scAssert(all(le(axAbs(dx1) * 4, axConst<Ai>(twiceDt))));
        scAssert(all(le(axAbs(dx2) * 4, axConst<Ai>(twiceDt))));

        denominator = twiceDtL * twiceDtL;
        velocity = 2 * twiceDtL * axCast<int64_t>(dx1);
        acceleration = 2 * halfA;
        error.fill(0);

        // First half step integration to make area under the velocity profile equal it's real
        // value at the end of integraion.
        velocity += axCast<int64_t>(halfA);
    }

    friend bool operator==(Segment const &lhs, Segment const &rhs) {
        return lhs.dt == rhs.dt && lhs.denominator == rhs.denominator &&
               lhs.velocity == rhs.velocity && lhs.acceleration == rhs.acceleration &&
               lhs.error == rhs.error;
    }

    friend bool operator!=(Segment const &lhs, Segment const &rhs) { return !(lhs == rhs); }

    friend std::ostream &operator<<(std::ostream &os, Segment const &obj) {
        return os << std::endl
                  << "dt: " << obj.dt << " denominator: " << obj.denominator
                  << " velocity: " << obj.velocity << " halfAcceleration: " << obj.acceleration
                  << " error: " << obj.error;
    }

    int32_t dt;
    Ai acceleration;
    Al velocity;
    int64_t denominator;
    Al error;
};
}