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
        : running_(false), motor_(motor), ticker_(ticker), ticksPerSecond_(1) {
        scAssert(motor_ && ticker_);
    }

    int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t ticksPerSecond) {
        scAssert(ticksPerSecond > 0);
        ticksPerSecond_ = ticksPerSecond;
    }

    void setSegments(Segments const &segments) { segments_ = segments; }
    void setSegments(Segments &&segments) { segments_ = move(segments); }

    void start() {
        if (!segments_.empty()) {
            running_ = true;
        }
        sg_ = segments_.begin();

        // Start timer.
        ticker_->attach_us(this, &SegmentsExecutor::tick, 1000000 / ticksPerSecond_);
    }

    void startHoming() {
        if (!segments_.empty()) {
            running_ = true;
        }
        // There should be one linear segment with small negative velocity.
        sg_ = segments_.begin();

        // Start timer with lower frequency because we need to check switches in loop.
        ticker_->attach_us(this, &SegmentsExecutor::tickHoming, 2000000 / ticksPerSecond_);
    }

    void tick() {
        // Integrate next interval.
        if (sg_->dt != 0) {
            tickI<0>();
            return;
        }

        // If there is next segment then integrate it's first interval.
        if (++sg_ != segments_.end()) {
            tickI<0>();
            return;
        }

        // No segments left.
        stop();
    }

    void tickHoming() {
        // Check end switch for every axis and stop if hit.
        for (size_t i = 0; i < AxesSize; i++) {
            if (motor_->checkEndSwitchHit(i)) {
                sg_->velocity[i] = 0;
            }
        }

        // If any switch is not hit then integrate next interval.
        if (any(neq(sg_->velocity, 0))) {
            tickI<0>();
            return;
        }

        // Stop when all switches are hit.
        stop();
    }

    bool running() const { return running_; }

    void stop() {
        running_ = false;
        ticker_->detach();
    }

  private:
    // Integrate i-th axis.
    template <size_t i>
    FORCE_INLINE void tickI() {
        // Integrate fist half of interval
        sg_->velocity[i] += sg_->halfAcceleration[i];

        // Update difference between rounded and actual position.
        sg_->error[i] += sg_->velocity[i];

        if (sg_->velocity[i] >= 0) {
            // Positive or zero slope.
            // error >= 0.5
            if (2 * sg_->error[i] >= sg_->denominator) {
                // error -= 1
                sg_->error[i] -= sg_->denominator;
                // Step in positive direcion.
                motor_->template write<i, 1>();
            } else {
                // No steps
                motor_->template write<i, 0>();
            }
        } else {
            // Negative slope.
            // -error >= 0.5
            if (-2 * sg_->error[i] >= sg_->denominator) {
                // error += 1
                sg_->error[i] += sg_->denominator;
                // Step in negative direction
                motor_->template write<i, -1>();
            } else {
                // No steps
                motor_->template write<i, 0>();
            }
        }
        // Integrate second half of interval
        sg_->velocity[i] += sg_->halfAcceleration[i];

        // Integrate next axis.
        tickI<i + 1>();
    }

    // All axes were integreted.
    template <>
    FORCE_INLINE void tickI<AxesSize>() {
        // Update time.
        --sg_->dt;

        // Notify motor about integration end.
        motor_->update();
    }

    bool running_;
    TMotor *motor_;
    TTicker *ticker_;
    Segments segments_;
    typename Segments::iterator sg_;
    int32_t ticksPerSecond_;
};
}
