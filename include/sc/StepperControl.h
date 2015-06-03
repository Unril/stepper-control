#pragma once

#include "Axes.h"

namespace StepperControl {
struct Segment {
    Segment(int x0, int t0, int x2, int t2) : x0(x0), t0(t0), x2(x2), t2(t2) {
        auto dx1 = x2 - x0;
        auto dt = t2 - t0;

        scAssert(abs(dx1) * 2 <= dt);

        denom = 2 * dt;
        vel = 2 * dx1;
        accHalf = 0;
        err = 0;
    }

    Segment(int x0, int t0, int x1, int x2, int t2) : x0(x0), t0(t0), x2(x2), t2(t2) {
        auto dx1 = x1 - x0;
        auto dx2 = x2 - x1;
        auto dt = (t2 - t0) / 2;

        scAssert(abs(dx1) * 2 <= dt);
        scAssert(abs(dx2) * 2 <= dt);

        denom = 4 * dt * dt;
        vel = 4 * dx1 * dt;
        accHalf = dx2 - dx1;
        err = 0;
    }

    inline void tick(bool *running, int8_t *high) {
        if (t0 == t2) {
            *running = false;
            scAssert(x0 == x2);
            return;
        }

        vel += accHalf;
        err += vel;
        auto sx = vel >= 0 ? 1 : -1;
        if (2 * err * sx >= denom) {
            err -= sx * denom;
            x0 += sx;
            *high = sx;
        } else {
            *high = 0;
        }
        vel += accHalf;
        t0 += 1;
    }

    int32_t x0, t0, x2, t2;

  private:
    int32_t vel, accHalf, err, denom;
};

template <size_t AxesSize>
class SegmentsGenerator {
  public:
    using AxesFloat = Axes<float, AxesSize>;
    using AxesInt = Axes<int32_t, AxesSize>;

    void setPath(std::vector<AxesInt> const &path) { path_ = path; }

    void setDurations(std::vector<float> const &durations) { durations_ = durations; }

    void setBlendDurations(std::vector<float> const &blendDurations) {
        blendDurations_ = blendDurations;
    }

    void update() {}

    std::vector<Segment> const &segments() const { return segments_; }

  private:
    std::vector<AxesInt> path_;
    std::vector<float> durations_;
    std::vector<float> blendDurations_;
    std::vector<Segment> segments_;
};

template <size_t AxesSize, typename T>
class SegmentsExecutor {
  public:
    using Segments = std::vector<Segment>;

    void setSegments(Segments const &segments) { segments_ = segments; }

    void start() {
        s_ = segments_.begin();
        if (s_ != segments_.end()) {
            running_ = true;
        }
    }

    void tick() {
        t_->setPixel(s_->x0, s_->t0, high_);
        s_->tick(&running_, &high_);
        if (!running_ && ++s_ != segments_.end()) {
            running_ = true;
            s_->tick(&running_, &high_);
        }
    }

    bool running() const { return running_; }

    T *t_;

  private:
    int8_t high_ = 0;
    bool running_ = false;
    Segments segments_;
    typename Segments::iterator s_;
};
}
