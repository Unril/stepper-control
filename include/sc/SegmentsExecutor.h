#pragma once

#include "Interfaces.h"

#include <atomic>

namespace StepperControl {
template <unsigned i>
struct StepperNumber {
    static const unsigned value = i;
};

// Starts timer and generates steps using provided linear or parabolic trajectory.
// Uses modified Bresenham's line drawing algorithm.
template <typename TMotor, typename TTicker, typename AxesTraits = DefaultAxesTraits>
class SegmentsExecutor {
  public:
    static const unsigned size = AxesTraits::size;
    using Ai = TAi<size>;
    using Sg = TSg<size>;
    using Sgs = TSgs<size>;
    using Callback = void (*)(void *);

    SegmentsExecutor(TMotor *motor, TTicker *ticker)
        : running_(false), motor_(motor), ticker_(ticker), position_(axZero<Ai>()),
          ticksPerSecond_(1), onStarted_(nullptr, nullptr), onStopped_(nullptr, nullptr) {
        scAssert(motor_ && ticker_);
        it_ = trajectory_.end();
    }

    int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t ticksPerSecond) {
        scAssert(ticksPerSecond > 0);
        ticksPerSecond_ = ticksPerSecond;
    }

    void setTrajectory(Sgs const &segments) {
        trajectory_ = segments;
        it_ = trajectory_.end();
    }

    void setTrajectory(Sgs &&segments) {
        trajectory_ = move(segments);
        it_ = trajectory_.end();
    }

    Sgs const &segments() const { return trajectory_; }

    void setOnStarted(Callback func, void *payload) { onStarted_ = std::make_pair(func, payload); }

    void setOnStopped(Callback func, void *payload) { onStopped_ = std::make_pair(func, payload); }

    void start() {
        if (trajectory_.empty()) {
            return;
        }
        it_ = trajectory_.begin();
        running_ = true;
        if (onStarted_.first) {
            onStarted_.first(onStarted_.second);
        }
        ticker_->attach_us(this, &SegmentsExecutor::tick, 1000000 / ticksPerSecond_);
    }

    void tick() {
        auto const dt = it_->dt;
        if (dt > 0) {
            // Integrate next interval.
            tick0();
        } else if (dt == 0 && ++it_ != trajectory_.end()) {
            // If there is next segment then integrate it's first interval.
            tick0();
        } else if (dt < 0) {
            // It is a homing cycle.
            if (any(neq(it_->velocity, 0))) {  // all velocities != 0 -- still homing
                // If any of switches is not hit then integrate next interval.
                tick0();

                // Check end switch for every axis and stop if hit.
                for (unsigned i = 0; i < AxesTraits::size; i++) {
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
            // No trajectory left.
            stop();
        }
    }

    bool isRunning() const { return running_; }

    void stop() {
        running_ = false;
        ticker_->detach();
        it_ = trajectory_.end();
        if (onStopped_.first) {
            onStopped_.first(onStopped_.second);
        }
    }

    Ai const &position() const { return position_; }

    void setPosition(Ai const &position = axZero<Ai>()) { position_ = position; }

  private:
    FORCE_INLINE void tick0() {
        motor_->begin();

        // Update time.
        --it_->dt;

        // Dir and step writing are separated by error updates to provide delay between them.
        writeDir(StepperNumber<0>{});
        bool stepRising[size];
        updateError(StepperNumber<0>{}, stepRising);
        writeStep(StepperNumber<0>{}, stepRising);

        // Notify motor about integration end.
        motor_->end();
    }

    template <unsigned i>
    FORCE_INLINE void writeDir(StepperNumber<i>) {
        motor_->writeDirection(StepperNumber<i>{}, it_->velocity[i] < 0);
        writeDir(StepperNumber<i + 1>{});
    }

    FORCE_INLINE void writeDir(StepperNumber<size>) {}

    // Integrate i-th axis.
    template <unsigned i>
    FORCE_INLINE void updateError(StepperNumber<i>, bool (&stepRising)[size]) {
        auto v = it_->velocity[i];
        auto error = it_->error[i];

        // Update difference between rounded and actual position.
        error += v;
        v += it_->acceleration[i];

        int sign = v >= 0 ? 1 : -1;
        // +1 -- positive or zero slope; -1 -- negative slope.
        //       error >= 0.5                  error <= -0.5
        if ((stepRising[i] = (2 * sign * error >= it_->denominator))) {
            //   error -= 1                    error += 1
            error -= it_->denominator * sign;
            // Rising edge.
            position_[i] += sign;
        }

        it_->velocity[i] = v;
        it_->error[i] = error;

        // Integrate next axis.
        updateError(StepperNumber<i + 1>{}, stepRising);
    }

    // All axes were integrated.
    FORCE_INLINE void updateError(StepperNumber<size>, bool[size]) {}

    template <unsigned i>
    FORCE_INLINE void writeStep(StepperNumber<i>, bool (&stepRising)[size]) {
        motor_->writeStep(StepperNumber<i>{}, stepRising[i]);
        writeStep(StepperNumber<i + 1>{}, stepRising);
    }

    FORCE_INLINE void writeStep(StepperNumber<size>, bool[size]) {}

    std::atomic<bool> running_{false};
    typename Sgs::iterator it_;
    Sgs trajectory_;
    TMotor *motor_;
    TTicker *ticker_;
    Ai position_;
    int32_t ticksPerSecond_;
    std::pair<Callback, void *> onStarted_;
    std::pair<Callback, void *> onStopped_;
};
}
