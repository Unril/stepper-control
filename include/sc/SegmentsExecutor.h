#pragma once

#include "Interfaces.h"

#include <atomic>

namespace StepperControl {
// Starts timer and generates steps using provided linear or parabolic segments.
// Uses modified Bresenham's line drawing algorithm.
template <typename TMotor, typename TTicker, typename AxesTraits = DefaultAxesTraits>
class SegmentsExecutor : public ISegmentsExecutor<AxesTraits> {
  public:
    using Ai = TAi<AxesTraits::size>;
    using Sg = TSg<AxesTraits::size>;
    using Sgs = TSgs<AxesTraits::size>;
    using Callback = void (*)(void *);

    SegmentsExecutor(TMotor *motor, TTicker *ticker)
        : running_(false), homing_(false), motor_(motor), ticker_(ticker), position_(axZero<Ai>()),
          ticksPerSecond_(1), onStarted_(nullptr, nullptr), onStopped_(nullptr, nullptr) {
        scAssert(motor_ && ticker_);
        currSeg_ = 0;
    }

    int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t ticksPerSecond) override {
        scAssert(ticksPerSecond > 0);
        ticksPerSecond_ = ticksPerSecond;
    }

    void setSegments(Sgs const &segments) override {
        segments_ = segments;
        currSeg_ = segments_.size();
    }

    void setSegments(Sgs &&segments) override {
        segments_ = move(segments);
        currSeg_ = segments_.size();
    }

    Sgs const &segments() const { return segments_; }

    void setOnStarted(Callback func, void *payload) { onStarted_ = std::make_pair(func, payload); }

    void setOnStopped(Callback func, void *payload) { onStopped_ = std::make_pair(func, payload); }

    void start() override {
        if (segments_.empty()) {
            return;
        }
        currSeg_ = 0;
        running_ = true;
        if (onStarted_.first) {
            onStarted_.first(onStarted_.second);
        }
        ticker_->attach_us(this, &SegmentsExecutor::tick, 1000000 / ticksPerSecond_);
    }

    void tick() RESTRICT {
        Sg *RESTRICT s = &(segments_[currSeg_]);
        if (s->dt > 0) {
            // Integrate next interval.
            tick0(s);
            return;
        }
        if (s->dt == 0 && ++currSeg_ < segments_.size()) {
            // If there is next segment then integrate it's first interval.
            tick0(&(segments_[currSeg_]));
            return;
        }
        if (s->dt < 0) {
            // It is a homing cycle.
            // TODO: check maximum frequency it can work on. Skip cycles if necessary.
            homing_ = any(neq(s->velocity, 0));
            if (homing_) {
                // If any of switches is not hit then integrate next interval.
                tick0(s);

                // Check end switch for every axis and stop if hit.
                for (unsigned i = 0; i < AxesTraits::size; i++) {
                    if (motor_->checkEndSwitchHit(i)) {
                        s->velocity[i] = 0;
                    }
                }
            } else {
                // Stop and reset position when all switches are hit.
                s->dt = 0;
                position_.fill(0);
            }
            return;
        }
        // No segments left.
        stop();
    }

    bool isRunning() const override { return running_; }

    bool isHoming() const { return homing_; }

    void stop() override {
        running_ = false;
        ticker_->detach();
        currSeg_ = segments_.size();
        if (onStopped_.first) {
            onStopped_.first(onStopped_.second);
        }
    }

    Ai const &position() const override { return position_; }

    void setPosition(Ai const &position = axZero<Ai>()) override { position_ = position; }

  private:
    FORCE_INLINE void tick0(Sg *RESTRICT s) RESTRICT {
        motor_->begin();

        // Update time.
        --(s->dt);

        tickI(UIntConst<0>(), s);

        // Notify motor about integration end.
        motor_->end();
    }

    // Integrate i-th axis.
    template <unsigned i>
    FORCE_INLINE void tickI(UIntConst<i>, Sg *RESTRICT s) RESTRICT {
        // Direction.
        if (s->velocity[i] >= 0) {
            // Positive or zero slope.
            motor_->writeDirection(UIntConst<i>(), UIntConst<0>());

            // Update difference between rounded and actual position.
            s->error[i] += s->velocity[i];
            s->velocity[i] += s->acceleration[i];

            // error >= 0.5
            if (2 * s->error[i] >= s->denominator) {
                // error -= 1
                s->error[i] -= s->denominator;
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

            // Update difference between rounded and actual position.
            s->error[i] += s->velocity[i];
            s->velocity[i] += s->acceleration[i];

            // error <= -0.5
            if (-2 * s->error[i] >= s->denominator) {
                // error += 1
                s->error[i] += s->denominator;
                // Rising edge.
                --position_[i];
                motor_->writeStep(UIntConst<i>(), UIntConst<1>());
            } else {
                // Falling edge.
                motor_->writeStep(UIntConst<i>(), UIntConst<0>());
            }
        }

        // Integrate next axis.
        tickI(UIntConst<i + 1>(), s);
    }

    // All axes were integrated.
    FORCE_INLINE void tickI(UIntConst<AxesTraits::size>, Sg *RESTRICT) RESTRICT {}

    std::atomic<bool> running_;
    std::atomic<bool> homing_;
    std::atomic<size_t> currSeg_;
    Sgs segments_;
    TMotor *motor_;
    TTicker *ticker_;
    Ai position_;
    int32_t ticksPerSecond_;
    std::pair<Callback, void *> onStarted_;
    std::pair<Callback, void *> onStopped_;
};
}
