#pragma once

#include "Interfaces.h"

namespace StepperControl {
template <unsigned i>
struct StepperNumber {
    static const unsigned value = i;
};

// Starts timer and generates steps using provided linear or parabolic segments.
// Uses modified Bresenham's line drawing algorithm.
template <typename TMotor, typename TTicker, typename AxesTraits = DefaultAxesTraits>
class SegmentsExecutor : public ISegmentsExecutor<AxesTraits> {
  public:
    static const unsigned size = AxesTraits::size;
    using Ai = TAi<size>;
    using Sg = TSg<size>;
    using Sgs = TSgs<size>;
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

    void tick() throw() {
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

    void setPosition(Ai const &position = axZero<Ai>()) override { position_ = position; }

  private:
    FORCE_INLINE void tick0() throw() {
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
    FORCE_INLINE void writeDir(StepperNumber<i>) throw() {
        motor_->writeDirection(StepperNumber<i>{}, it_->velocity[i] < 0);
        writeDir(StepperNumber<i + 1>{});
    }

    FORCE_INLINE void writeDir(StepperNumber<size>) throw() {}

    // Integrate i-th axis.
    template <unsigned i>
    FORCE_INLINE void updateError(StepperNumber<i>, bool(&stepRising)[size]) throw() {
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
    FORCE_INLINE void updateError(StepperNumber<size>, bool[size]) throw() {}

    template <unsigned i>
    FORCE_INLINE void writeStep(StepperNumber<i>, bool(&stepRising)[size]) throw() {
        motor_->writeStep(StepperNumber<i>{}, stepRising[i]);
        writeStep(StepperNumber<i + 1>{}, stepRising);
    }

    FORCE_INLINE void writeStep(StepperNumber<size>, bool[size]) throw() {}

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
