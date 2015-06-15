#pragma once

#include "Segment.h"

namespace StepperControl {

enum class ExecutorState : uint8_t {

};

// Starts timer and generates steps using provided linear or parabolic segments.
// Uses modified Bresenham's line drawing algorithm.
template <size_t AxesSize, typename TMotor, typename TTicker>
class SegmentsExecutor {
  public:
    using Sg = Segment<AxesSize>;
    using Segments = std::vector<Sg>;
    using Ai = Axes<int32_t, AxesSize>;

    SegmentsExecutor(TMotor *motor, TTicker *ticker)
        : motor_(motor), ticker_(ticker), position_(axZero<Ai>()), ticksPerSecond_(1) {
        scAssert(motor_ && ticker_);
        it_ = segments_.end();
    }

    inline int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t ticksPerSecond) {
        scAssert(ticksPerSecond > 0);
        ticksPerSecond_ = ticksPerSecond;
    }

    void setSegments(Segments const &segments) {
        segments_ = segments;
        it_ = segments_.end();
    }

    void setSegments(Segments &&segments) {
        segments_ = move(segments);
        it_ = segments_.end();
    }

    Segments const &segments() const { return segments_; }

    void start() {
        if (segments_.empty()) {
            return;
        }
        it_ = segments_.begin();

        ticker_->attach_us(this, &SegmentsExecutor::tick, 1000000 / ticksPerSecond_);
    }

    inline void tick() {
        auto const dt = it_->dt;
        if (dt > 0) {
            // Integrate next interval.
            tick0();
        } else if (dt == 0 && ++it_ != segments_.end()) {
            // dt == 0
            // If there is next segment then integrate it's first interval.
            tick0();
        } else if (dt < 0) {
            // It is a homing cycle.
            // TODO: check maximum frequency it can work on. Skip cycles if neccesary.
            if (any(neq(it_->velocity, 0))) {
                // If any of switches is not hit then integrate next interval.
                tick0();

                // Check end switch for every axis and stop if hit.
                for (size_t i = 0; i < AxesSize; i++) {
                    if (motor_->checkEndSwitchHit(i)) {
                        it_->velocity[i] = 0;
                    }
                }
            } else {
                // Stop and reset position when all switches are hit.
                it_->dt = 0;
                position_.fill(0);
            }
        } else {
            // No segments left.
            stop();
        }
    }

    inline bool isRunning() const { return it_ != segments_.end(); }

    inline void stop() { ticker_->detach(); }

    inline Ai const &position() const { return position_; }

    inline void setPosition(Ai const &position) { position_ = position; }

  private:
    FORCE_INLINE void tick0() {
        motor_->begin();

        // Update time.
        --it_->dt;

        tickI(UIntConst<0>());

        // Notify motor about integration end.
        motor_->end();
    }

    // Integrate i-th axis.
    template <size_t i>
    FORCE_INLINE void tickI(UIntConst<i>) {
        auto v = it_->velocity[i];

        // Direction.
        if (v >= 0) {
            // Positive or zero slope.
            motor_->writeDirection(UIntConst<i>(), UIntConst<0>());

            auto const denominator = it_->denominator;
            auto error = it_->error[i];

            // Update difference between rounded and actual position.
            error += v;

            v += it_->acceleration[i];

            it_->velocity[i] = v;

            // error >= 0.5
            if (2 * error >= denominator) {
                // error -= 1
                error -= denominator;
                // Rising edge.
                ++position_[i];
                motor_->writeStep(UIntConst<i>(), UIntConst<1>());
            } else {
                // Falling edge.
                motor_->writeStep(UIntConst<i>(), UIntConst<0>());
            }
            it_->error[i] = error;
        } else {
            // Negative slope.
            motor_->writeDirection(UIntConst<i>(), UIntConst<1>());

            // Duplicate code to make delay between direction and step writes.
            auto const denominator = it_->denominator;
            auto error = it_->error[i];

            // Update difference between rounded and actual position.
            error += v;

            v += it_->acceleration[i];

            it_->velocity[i] = v;

            // error <= -0.5
            if (-2 * error >= denominator) {
                // error += 1
                error += denominator;
                // Rising edge.
                --position_[i];
                motor_->writeStep(UIntConst<i>(), UIntConst<1>());
            } else {
                // Falling edge.
                motor_->writeStep(UIntConst<i>(), UIntConst<0>());
            }
            it_->error[i] = error;
        }

        // Integrate next axis.
        tickI(UIntConst<i + 1>());
    }

    // All axes were integrated.
    FORCE_INLINE void tickI(UIntConst<AxesSize>) {}

    typename Segments::iterator it_;
    Segments segments_;
    TMotor *motor_;
    TTicker *ticker_;
    Ai position_;
    int32_t ticksPerSecond_;
};
}
