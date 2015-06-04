#pragma once

#include "Axes.h"

namespace StepperControl {
template <size_t AxesSize>
struct Segment {
    using Ai = Axes<int32_t, AxesSize>;

    Segment(int32_t dt, Ai const &dx) : dt(dt) {
        scAssert(dt > 0);
        scAssert(all(le(axesAbs(dx) * 2, axesConstant<AxesSize>(dt))) && "dx <= dt/2");

        denominator_ = 2 * dt;
        velocity_ = 2 * dx;
        halfAcceleration_.fill(0);
        error_.fill(0);
    }

    Segment(int32_t twiceDt, Ai const &dx1, Ai const &dx2) : dt(twiceDt) {
        scAssert(twiceDt > 0);
        scAssert(all(le(axesAbs(dx1) * 4, axesConstant<AxesSize>(twiceDt))) && "dx1 <= dt/2");
        scAssert(all(le(axesAbs(dx2) * 4, axesConstant<AxesSize>(twiceDt))) && "dx2 <= dt/2");

        denominator_ = twiceDt * twiceDt;
        velocity_ = 2 * dx1 * twiceDt;
        halfAcceleration_ = dx2 - dx1;
        error_.fill(0);
    }

    inline bool isCompleted() const { return dt == 0; }

    inline void tick(Ai &positionDelta, Ai &position) {
        scAssert(!isCompleted());

        for (size_t i = 0; i < AxesSize; i++) {
            // Integrate fist half of interval
            velocity_[i] += halfAcceleration_[i];

            // Update difference between rounded and actual position.
            error_[i] += velocity_[i];

            if (velocity_[i] >= 0) {
                // Positive or zero slope.
                // error >= 0.5
                if (2 * error_[i] >= denominator_) {
                    // error -= 1
                    error_[i] -= denominator_;

                    // Update position and it's delta
                    position[i] += 1;
                    positionDelta[i] = 1;
                } else {
                    positionDelta[i] = 0;
                }
            } else {
                // Negative slope.
                // -error >= 0.5
                if (-2 * error_[i] >= denominator_) {
                    // error += 1
                    error_[i] += denominator_;

                    // Update position and it's delta
                    position[i] -= 1;
                    positionDelta[i] = -1;
                } else {
                    positionDelta[i] = 0;
                }
            }
            // Integrate second half of interval
            velocity_[i] += halfAcceleration_[i];
        }

        // Decrement remaining time.
        --dt;
    }

  private:
    Ai velocity_, halfAcceleration_, error_;
    int32_t dt, denominator_;
};

template <size_t AxesSize>
class SegmentsGenerator {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;
    using Segments = std::vector<Segment<AxesSize>>;

    void setPath(std::vector<Ai> const &path) { path_ = path; }

    void setDurations(std::vector<float> const &durations) { durations_ = durations; }

    void setBlendDurations(std::vector<float> const &blendDurations) {
        blendDurations_ = blendDurations;
    }

    void update() {}

    Segments const &segments() const { return segments_; }

  private:
    std::vector<Ai> path_;
    std::vector<float> durations_;
    std::vector<float> blendDurations_;
    Segments segments_;
};

template <size_t AxesSize, template <size_t> class TMotor, typename TTicker>
class SegmentsExecutor {
  public:
    using Segments = std::vector<Segment<AxesSize>>;
    using Ai = Axes<int32_t, AxesSize>;

    SegmentsExecutor(TMotor<AxesSize> *motor, TTicker *ticker)
        : running_(false), motor_(motor), ticker_(ticker), ticksPerSecond_(1) {
        scAssert(motor_ && ticker_);
        positionDelta_.fill(0);
        position_.fill(0);
    }

    Ai const &position() const { return position_; }

    void setPosition(Ai const &position) { position_ = position; }

    int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t ticksPerSecond) {
        scAssert(ticksPerSecond > 0);
        ticksPerSecond_ = ticksPerSecond;
    }

    void setSegments(Segments const &segments) { segments_ = segments; }

    void start() {
        if (!segments_.empty()) {
            running_ = true;
        }
        positionDelta_.fill(0);
        segment_ = segments_.begin();

        // Start timer.
        ticker_->attach_us(this, &SegmentsExecutor::tick, 1000000 / ticksPerSecond_);
    }

    void tick() {
        // Update stepper position if required.
        motor_->write(position_, positionDelta_);
        if (!segment_->isCompleted()) {
            // Integrate next interval.
            segment_->tick(positionDelta_, position_);
        } else if (++segment_ != segments_.end()) {
            // If there is next segment then integrate it's first interval.
            segment_->tick(positionDelta_, position_);
        } else {
            // No segments left.
            stop();
        }
    }

    bool running() const { return running_; }

    void stop() {
        running_ = false;
        ticker_->detach();
    }

  private:
    bool running_;
    TMotor<AxesSize> *motor_;
    TTicker *ticker_;
    Ai positionDelta_;
    Ai position_;
    Segments segments_;
    typename Segments::iterator segment_;
    int32_t ticksPerSecond_;
};
}
