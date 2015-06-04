#pragma once

#include "Axes.h"

namespace StepperControl {
template <size_t AxesSize>
struct Segment {
    using Ai = Axes<int32_t, AxesSize>;

    Segment(int32_t t0, Ai const &x0, int32_t t2, Ai const &x2) : x0(x0), x2(x2), t0(t0), t2(t2) {
        auto Dx1 = x2 - x0;
        auto Dt = t2 - t0;

        scAssert(all(le(axesAbs(Dx1) * 2, /* <= */ axesConstant<AxesSize>(Dt))));
        scAssert(Dt > 0);

        denominator_ = 2 * Dt;
        velocity_ = 2 * Dx1;
        halfAcceleration_.fill(0);
        error_.fill(0);
    }

    Segment(int32_t t0, Ai const &x0, Ai const &x1, int32_t t2, Ai const &x2)
        : x0(x0), x2(x2), t0(t0), t2(t2) {
        auto Dx1 = x1 - x0;
        auto Dx2 = x2 - x1;
        auto twiceDt = t2 - t0;

        scAssert(all(le(axesAbs(Dx1) * 4, /* <= */ axesConstant<AxesSize>(twiceDt))));
        scAssert(all(le(axesAbs(Dx2) * 4, /* <= */ axesConstant<AxesSize>(twiceDt))));
        scAssert(twiceDt > 0);

        denominator_ = twiceDt * twiceDt;
        velocity_ = 2 * Dx1 * twiceDt;
        halfAcceleration_ = Dx2 - Dx1;
        error_.fill(0);
    }

    inline bool isRunning() const {
        scAssert(t0 != t2 || all(eq(x0, x2)));

        return t0 != t2;
    }

    inline void tick(Ai &stepDirection) {
        scAssert(isRunning());

        for (size_t i = 0; i < AxesSize; i++) {
            velocity_[i] += halfAcceleration_[i];
            error_[i] += velocity_[i];
            if (velocity_[i] >= 0) {
                if (2 * error_[i] >= denominator_) {
                    error_[i] -= denominator_;
                    x0[i] += 1;
                    stepDirection[i] = 1;
                } else {
                    stepDirection[i] = 0;
                }
            } else {
                if (-2 * error_[i] >= denominator_) {
                    error_[i] += denominator_;
                    x0[i] -= 1;
                    stepDirection[i] = -1;
                } else {
                    stepDirection[i] = 0;
                }
            }
            velocity_[i] += halfAcceleration_[i];
        }
        t0 += 1;
    }

    Ai x0, x2;
    int32_t t0, t2;

  private:
    Ai velocity_, halfAcceleration_, error_;
    int32_t denominator_;
};

template <size_t AxesSize>
class SegmentsGenerator {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;

    void setPath(std::vector<Ai> const &path) { path_ = path; }

    void setDurations(std::vector<float> const &durations) { durations_ = durations; }

    void setBlendDurations(std::vector<float> const &blendDurations) {
        blendDurations_ = blendDurations;
    }

    void update() {}

    std::vector<Segment<AxesSize>> const &segments() const { return segments_; }

  private:
    std::vector<Ai> path_;
    std::vector<float> durations_;
    std::vector<float> blendDurations_;
    std::vector<Segment<AxesSize>> segments_;
};

template <size_t AxesSize, template <size_t> class T>
class SegmentsExecutor {
  public:
    using Segments = std::vector<Segment<AxesSize>>;
    using Ai = Axes<int32_t, AxesSize>;

    SegmentsExecutor() {
        running_ = false;
        stepDirection_.fill(0);
    }

    void setSegments(Segments const &segments) { segments_ = segments; }

    void start() {
        if (!segments_.empty()) {
            running_ = true;
        }
        stepDirection_.fill(0);
        segment_ = segments_.begin();
    }

    void tick() {
        t_->write(segment_->x0, segment_->t0, stepDirection_);
        if (running_ = segment_->isRunning()) {
            segment_->tick(stepDirection_);
        } else if (++segment_ != segments_.end()) {
            running_ = true;
            segment_->tick(stepDirection_);
        } else {
            stop();
        }
    }

    bool running() const { return running_; }

    void stop() {}

    T<AxesSize> *t_;

  private:
    bool running_;
    Ai stepDirection_;
    Segments segments_;
    typename Segments::iterator segment_;
};
}
