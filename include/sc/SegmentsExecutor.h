#pragma once

#include "Interfaces.h"

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
        it_ = segments_.end();
    }

    int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t ticksPerSecond) override {
        scAssert(ticksPerSecond > 0);
        ticksPerSecond_ = ticksPerSecond;
    }

    void setSegments(Sgs const &segments) override {
        segments_ = segments;
        it_ = segments_.end();
    }

    void setSegments(Sgs &&segments) override {
        segments_ = move(segments);
        it_ = segments_.end();
    }

    Sgs const &segments() const { return segments_; }

    void setOnStarted(Callback func, void *payload) { onStarted_ = std::make_pair(func, payload); }

    void setOnStopped(Callback func, void *payload) { onStopped_ = std::make_pair(func, payload); }

    void start() override {
        if (segments_.empty()) {
            return;
        }
        it_ = segments_.begin();
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
        } else if (dt == 0 && ++it_ != segments_.end()) {
            // If there is next segment then integrate it's first interval.
            tick0();
        } else if (dt < 0) {
            // It is a homing cycle.
            // TODO: check maximum frequency it can work on. Skip cycles if necessary.
            homing_ = any(neq(it_->velocity, 0));
            if (homing_) {
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
            // No segments left.
            stop();
        }
    }

    bool isRunning() const override { return running_; }

    bool isHoming() const { return homing_; }

    void stop() override {
        running_ = false;
        ticker_->detach();
        it_ = segments_.end();
        if (onStopped_.first) {
            onStopped_.first(onStopped_.second);
        }
    }

    Ai const &position() const override { return position_; }

    void setPosition(Ai const &position) override { position_ = position; }

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
    template <unsigned i>
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
    FORCE_INLINE void tickI(UIntConst<AxesTraits::size>) {}

    bool running_, homing_;
    typename Sgs::iterator it_;
    Sgs segments_;
    TMotor *motor_;
    TTicker *ticker_;
    Ai position_;
    int32_t ticksPerSecond_;
    std::pair<Callback, void *> onStarted_;
    std::pair<Callback, void *> onStopped_;
};
}
