#pragma once

#include "Segment.h"

namespace StepperControl {

// Starts timer and generates steps using provided linear or parabolic segments.
// Uses modified Bresenham's line drawing algorithm.
template <size_t AxesSize, typename TMotor, typename TTicker>
class SegmentsExecutor {
  public:
    using Sg = Segment<AxesSize>;
    using Segments = std::vector<Sg>;
    using Ai = Axes<int32_t, AxesSize>;

    SegmentsExecutor(TMotor *motor, TTicker *ticker)
        : motor_(motor), position_(axZero<Ai>()), tick_(0), running_(false), ticker_(ticker),
          ticksPerSecond_(1) {
        scAssert(motor_ && ticker_);
    }

    inline int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t ticksPerSecond) {
        scAssert(ticksPerSecond > 0);
        ticksPerSecond_ = ticksPerSecond;
    }

    void setSegments(Segments const &segments) { segments_ = segments; }

    void setSegments(Segments &&segments) { segments_ = move(segments); }

    Segments const &segments() const { return segments_; }

    void start() {
        if (segments_.empty()) {
            return;
        }
        running_ = true;
        tick_ = 0;
        sg_ = segments_.begin();

        // Start timer.
        ticker_->attach_us(this, &SegmentsExecutor::tick, 1000000 / ticksPerSecond_);
    }

    // Execute segment until all end switches are hit.
    // There should be one linear segment with small negative velocity.
    void startHoming() {
        if (segments_.empty()) {
            return;
        }
        running_ = true;
        tick_ = 0;
        sg_ = segments_.begin();

        // Start timer with lower frequency because we need to check switches in loop.
        ticker_->attach_us(this, &SegmentsExecutor::tickHoming, 2000000 / ticksPerSecond_);
    }

    inline void tick() {
        // Integrate next interval.
        if (sg_->dt != 0) {
            tickI(UIntConst<0>());
            return;
        }

        // If there is next segment then integrate it's first interval.
        if (++sg_ != segments_.end()) {
            tickI(UIntConst<0>());
            return;
        }

        // No segments left.
        stop();
    }

    inline void tickHoming() {
        // Check end switch for every axis and stop if hit.
        for (size_t i = 0; i < AxesSize; i++) {
            if (motor_->checkEndSwitchHit(i)) {
                sg_->velocity[i] = 0;
            }
        }

        // If any of switches is not hit then integrate next interval.
        if (any(neq(sg_->velocity, 0))) {
            tickI(UIntConst<0>());
            return;
        }

        // Set position and stop when all switches are hit.
        position_.fill(0);
        stop();
    }

    inline bool running() const { return running_; }

    void stop() {
        ticker_->detach();
        running_ = false;
        tick_ = 0;
    }

    inline Ai const &position() const { return position_; }

    inline void setPosition(Ai const &position) { position_ = position; }

    inline int32_t ticks() const { return tick_; }

  private:
    // Integrate i-th axis.
    template <size_t i>
    FORCE_INLINE void tickI(UIntConst<i>) {
        auto const denominator = sg_->denominator;
        auto v = sg_->velocity[i];
        auto const aHalf = sg_->halfAcceleration[i];
        auto error = sg_->error[i];

        // Integrate fist half of interval
        v += aHalf;

        // Direction.
        if (v >= 0) {
            // Positive or zero slope.
            motor_->writeDirection(UIntConst<i>(), UIntConst<0>());

            // Update difference between rounded and actual position.
            error += v;

            // Integrate second half of interval
            v += aHalf;

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
        } else {
            // Negative slope.
            motor_->writeDirection(UIntConst<i>(), UIntConst<1>());

            // Duplicate code to make delay between direction and step writes.
            // Update difference between rounded and actual position.
            error += v;

            // Integrate second half of interval
            v += aHalf;

            // -error >= 0.5
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
        }

        sg_->velocity[i] = v;
        sg_->error[i] = error;

        // Integrate next axis.
        tickI(UIntConst<i + 1>());
    }

    // All axes were integrated.
    FORCE_INLINE void tickI(UIntConst<AxesSize>) {
        // Update time.
        --sg_->dt;
        ++tick_;

        // Notify motor about integration end.
        motor_->update();
    }

    typename Segments::iterator sg_;
    TMotor *motor_;
    Ai position_;
    int32_t tick_;
    bool running_;
    TTicker *ticker_;
    Segments segments_;
    int32_t ticksPerSecond_;
};
}
