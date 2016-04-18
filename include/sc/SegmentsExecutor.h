#pragma once

#include "Segment.h"

namespace StepperControl {
template <int i>
struct StepperNumber {
    static const int value = i;
};

// Starts timer and generates steps using provided linear or parabolic trajectory.
// Uses modified Bresenham's line drawing algorithm.
template <typename TMotor, typename TTicker, typename AxesTraits = DefaultAxesTraits>
class SegmentsExecutor {
  public:
    static const int size = AxesTraits::size;
    using Ai = TAi<size>;
    using Sg = TSg<size>;
    using Sgs = TSgs<size>;
    using Callback = void (*)(void *);

    SegmentsExecutor(TMotor *motor, TTicker *ticker)
        : running_(false), motor_(motor), ticker_(ticker), position_(axZero<Ai>()),
          ticksPerSecond_(1), onStarted_(nullptr, nullptr), onStopped_(nullptr, nullptr) {
        scAssert(motor_ && ticker_);
        it_ = end();
    }

    int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t ticksPerSecond) {
        scAssert(ticksPerSecond > 0);
        ticksPerSecond_ = ticksPerSecond;
    }

    void setTrajectory(Sgs const &segments) {
        trajectory_ = segments;
        it_ = end();
    }

    void setTrajectory(Sgs &&segments) {
        trajectory_ = move(segments);
        it_ = end();
    }

    Sgs const &segments() const { return trajectory_; }

    void setOnStarted(Callback func, void *payload) { onStarted_ = std::make_pair(func, payload); }

    void setOnStopped(Callback func, void *payload) { onStopped_ = std::make_pair(func, payload); }

    void start() {
        if (onStarted_.first) {
            onStarted_.first(onStarted_.second);
        }
        it_ = begin();
        running_ = true;
        currentTick_ = 0;
        for (int i = 0; i < size; ++i) {
            dir_[i] = false;
        }
        writeDir(StepperNumber<0>{});
        if (!trajectory_.empty()) {
            ticker_->attach_us(this, &SegmentsExecutor::tick, 1000000 / ticksPerSecond_);
        } else {
            stop();
        }
    }

    void tick() {
        auto const dt = it_->dt;
        if (dt > 0) {
            // Integrate next interval.
            tick0();
        } else if (dt == 0 && ++it_ != end()) {
            // If there is next segment then integrate it's first interval.
            tick0();
        } else if (dt < 0) {
            // It is a homing cycle.
            if (any(neq(it_->velocity, 0))) { // all velocities != 0 -- still homing
                // If any of switches is not hit then integrate next interval.
                tick0();

                // Check end switch for every axis and stop if hit.
                for (int i = 0; i < AxesTraits::size; i++) {
                    if (it_->velocity[i] != 0 && motor_->checkEndSwitchHit(i)) {
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

    int32_t currentTick() const { return currentTick_; }

    void stop() {
        ticker_->detach();
        it_ = end();
        running_ = false;
        currentTick_ = 0;
        if (onStopped_.first) {
            onStopped_.first(onStopped_.second);
        }
    }

    Ai const &position() const { return position_; }

    void setPosition(Ai const &position = axZero<Ai>()) { position_ = position; }

  private:
    FORCE_INLINE Sg *begin() RESTRICT { return trajectory_.data(); }
    FORCE_INLINE Sg *end() RESTRICT { return trajectory_.data() + trajectory_.size(); }

    FORCE_INLINE void tick0() RESTRICT {
        motor_->begin();

        // Update time.
        --it_->dt;
        ++currentTick_;

        shouldMakeAnyStep_ = false;
        shouldChangeAnyDir_ = false;

        updateDir(StepperNumber<0>{});

        if (shouldChangeAnyDir_) {
            writeDir(StepperNumber<0>{});
            wait_us(4);
        }

        updateErr(StepperNumber<0>{});

        if (shouldMakeAnyStep_) {
            writeStep(StepperNumber<0>{});
            wait_us(2);
            clearStep(StepperNumber<0>{});
        }

        // Notify motor about integration end.
        motor_->end();
    }

    // Integrate i-th axis.
    template <int i>
    FORCE_INLINE void updateDir(StepperNumber<i>) RESTRICT {
        auto oldDir = dir_[i];
        dir_[i] = it_->velocity[i] < 0;
        shouldChangeAnyDir_ = shouldChangeAnyDir_ || oldDir != dir_[i];

        updateDir(StepperNumber<i + 1>{});
    }

    // All axes were integrated.
    FORCE_INLINE void updateDir(StepperNumber<size>) RESTRICT {}

    template <int i>
    FORCE_INLINE void updateErr(StepperNumber<i>) RESTRICT {
        // Update difference between rounded and actual position.
        it_->error[i] += it_->velocity[i];

        int sign = it_->velocity[i] >= 0 ? 1 : -1;

        // +1 -- positive or zero slope; -1 -- negative slope.
        //       error >= 0.5                  error <= -0.5
        if (2 * sign * it_->error[i] >= it_->denominator) {
            //   error -= 1                    error += 1
            it_->error[i] -= it_->denominator * sign;
            // Rising edge.
            position_[i] += sign;

            shouldMakeAnyStep_ = true;
            step_[i] = true;
        } else {
            step_[i] = false;
        }

        it_->velocity[i] += it_->acceleration[i];

        updateErr(StepperNumber<i + 1>{});
    }

    FORCE_INLINE void updateErr(StepperNumber<size>) RESTRICT {}

    // Integrate i-th axis.
    template <int i>
    FORCE_INLINE void writeDir(StepperNumber<i>) RESTRICT {
        motor_->writeDirection(StepperNumber<i>{}, dir_[i]);

        writeDir(StepperNumber<i + 1>{});
    }

    // All axes were integrated.
    FORCE_INLINE void writeDir(StepperNumber<size>) RESTRICT {}

    template <int i>
    FORCE_INLINE void writeStep(StepperNumber<i>) RESTRICT {
        motor_->writeStep(StepperNumber<i>{}, step_[i]);

        writeStep(StepperNumber<i + 1>{});
    }

    FORCE_INLINE void writeStep(StepperNumber<size>) RESTRICT {}

    template <int i>
    FORCE_INLINE void clearStep(StepperNumber<i>) RESTRICT {
        motor_->writeStep(StepperNumber<i>{}, false);

        clearStep(StepperNumber<i + 1>{});
    }

    FORCE_INLINE void clearStep(StepperNumber<size>) RESTRICT {}

    int32_t currentTick_{};
    bool running_{};

    bool shouldMakeAnyStep_{};
    bool shouldChangeAnyDir_{};
    bool step_[size]{};
    bool dir_[size]{};

    Sg *RESTRICT it_{};
    Sgs trajectory_;
    TMotor *RESTRICT motor_{};
    TTicker *RESTRICT ticker_{};
    Ai position_{};
    int32_t ticksPerSecond_{};
    std::pair<Callback, void *> onStarted_;
    std::pair<Callback, void *> onStopped_;
};
}
