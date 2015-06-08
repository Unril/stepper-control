#pragma once

#include "Axes.h"

namespace StepperControl {

// Contains data for Bresenham's algorithm.
template <size_t AxesSize>
struct Segment {
    using Ai = Axes<int32_t, AxesSize>;

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
        scAssert(dt > 0);
        // dx <= dt/2
        scAssert(all(le(axAbs(dx) * 2, axConst<Ai>(dt))));

        denominator = 2 * dt;
        velocity = 2 * dx;
        halfAcceleration.fill(0);
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
        scAssert(twiceDt > 0);
        // dx1 <= dt1/2 && dx2 <= dt2/2
        scAssert(all(le(axAbs(dx1) * 4, axConst<Ai>(twiceDt))));
        scAssert(all(le(axAbs(dx2) * 4, axConst<Ai>(twiceDt))));

        denominator = twiceDt * twiceDt;
        velocity = 2 * dx1 * twiceDt;
        halfAcceleration = dx2 - dx1;
        error.fill(0);
    }

    friend bool operator==(Segment const &lhs, Segment const &rhs) {
        return lhs.dt == rhs.dt && lhs.denominator == rhs.denominator &&
               lhs.velocity == rhs.velocity && lhs.halfAcceleration == rhs.halfAcceleration &&
               lhs.error == rhs.error;
    }

    friend bool operator!=(Segment const &lhs, Segment const &rhs) { return !(lhs == rhs); }

    friend std::ostream &operator<<(std::ostream &os, Segment const &obj) {
        return os << std::endl
                  << "dt: " << obj.dt << " denominator: " << obj.denominator
                  << " velocity: " << obj.velocity << " halfAcceleration: " << obj.halfAcceleration
                  << " error: " << obj.error;
    }

    int32_t dt, denominator;
    Ai velocity, halfAcceleration, error;
};
}