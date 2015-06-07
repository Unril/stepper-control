#pragma once

#include "Axes.h"

namespace StepperControl {

inline int32_t lTruncTowardZero(float v) { return static_cast<int32_t>(v); }

inline int32_t lTruncTowardInf(float v) {
    return static_cast<int32_t>(v < 0.0f ? floor(v) : ceil(v));
}

// Contains data for Bresenham's algorithm.
template <size_t AxesSize>
struct Segment {
    using Ai = Axes<int32_t, AxesSize>;

    /* Linear segment.
       It is set by start and end points p0-p1.
    x  ^
       |
   x1  +-------p1
       |     / |    dt = t1 - t0, ticks
       |   /   |    dx = x1 - x0, steps
       | /     |
   x0  p0------+---> t
       t0        t1
    */
    Segment(int32_t dt, Ai const &dx) : dt(dt) {
        scAssert(dt > 0);
        // dx <= dt/2
        scAssert(all(le(axAbs(dx) * 2, axConst<Ai>(dt))));

        denominator = 2 * dt;
        velocity = 2 * dx;
        halfAcceleration.fill(0);
        error.fill(0);
    }

    /* Parabolic segment.
       It is set by endpoints of two tangent to parabola segments p0-p1 and p1-p2.
    x  ^
   x1  +----p1         twiceDt = t2 - t0, ticks
       |   /| \        dx1 = x1 - x0, steps
       |  / |   \      dx2 = x2 - x1, steps
   x2  +-/--|-----p2
       |/   |     |
   x0  p0---+-----+---> t
       t0   t1    t2
    */
    Segment(int32_t twiceDt, Ai const &dx1, Ai const &dx2) : dt(twiceDt) {
        scAssert(twiceDt > 0);
        // dx1 <= dt1/2 && dx2 <= dt2/2
        scAssert(all(le(axAbs(dx1) * 4, axConst<Ai>(twiceDt))));
        scAssert(all(le(axAbs(dx2) * 4, axConst<Ai>(twiceDt))));

        denominator = twiceDt * twiceDt;
        velocity = 2 * dx1 * twiceDt;
        halfAcceleration = dx2 - dx1;
        error.fill(0);
    }

    friend bool operator==(Segment const &lhs, Segment const &rhs) {
        return lhs.dt == rhs.dt && lhs.denominator == rhs.denominator &&
               lhs.velocity == rhs.velocity && lhs.halfAcceleration == rhs.halfAcceleration &&
               lhs.error == rhs.error;
    }

    friend bool operator!=(Segment const &lhs, Segment const &rhs) { return !(lhs == rhs); }

    friend std::ostream &operator<<(std::ostream &os, Segment const &obj) {
        return os << std::endl
                  << "dt: " << obj.dt << " denominator: " << obj.denominator
                  << " velocity: " << obj.velocity << " halfAcceleration: " << obj.halfAcceleration
                  << " error: " << obj.error;
    }

    int32_t dt, denominator;
    Ai velocity, halfAcceleration, error;
};

// It creates sequence of linear and parabolic segments from given path points,
// durations between points and durations of blend segments.
template <size_t AxesSize>
class SegmentsGenerator {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;
    using Segments = std::vector<Segment<AxesSize>>;

    template <typename T>
    void setAll(T const &generator) {
        setPath(generator.path());
        setDurations(generator.durations());
        setBlendDurations(generator.blendDurations());
    }

    void setPath(std::vector<Ai> const &path) { path_ = path; }

    void setDurations(std::vector<float> const &durations) {
        durations_.resize(durations.size());
        transform(durations.begin(), durations.end(), durations_.begin(), &lTruncTowardInf);
    }

    void setBlendDurations(std::vector<float> const &blendDurations) {
        blendDurations_.resize(blendDurations.size());
        transform(blendDurations.begin(), blendDurations.end(), blendDurations_.begin(),
                  &lTruncTowardInf);
    }

    void update() {
        scAssert(!path_.empty());
        scAssert(path_.size() - 1 == durations_.size());
        scAssert(path_.size() == blendDurations_.size());

        segments_.clear();

        for (size_t i = 0; i < path_.size(); i++) {
            addSegmentsForPoint(i);
        }
    }

    Segments const &segments() const { return segments_; }

  private:
    void addSegmentsForPoint(size_t i) {
        auto firstPoint = i == 0;
        auto lastPoint = (i == path_.size() - 1);

        auto x = path_[i];
        auto tBlend = blendDurations_[i];

        // Add only nonzero blend segment.
        if (tBlend > 0) {
            auto v = axZero<Af>();     // First tangent slope.
            auto vNext = axZero<Af>(); // Second tangent slope.

            // Treat first and last points differently.
            if (firstPoint) {
                // First tangent of first blend has zero slope.
                auto const &xNext = path_[i + 1];
                auto Dt = static_cast<float>(durations_[i]);
                vNext = axCast<float>(xNext - x) / Dt;
            } else if (lastPoint) {
                // Second tangent of last blend has zero slope.
                auto const &xPrev = path_[i - 1];
                auto DtPrev = static_cast<float>(durations_[i - 1]);
                v = axCast<float>(x - xPrev) / DtPrev;
            } else {
                auto const &xPrev = path_[i - 1];
                auto const &xNext = path_[i + 1];
                auto Dt = static_cast<float>(durations_[i]);
                auto DtPrev = static_cast<float>(durations_[i - 1]);
                v = axCast<float>(x - xPrev) / DtPrev;
                vNext = axCast<float>(xNext - x) / Dt;
            }

            scAssert(all(le(axAbs(v), axConst<Af>(0.5f))));
            scAssert(all(le(axAbs(vNext), axConst<Af>(0.5f))));

            auto Dx = axLRound(0.5f * tBlend * v);
            auto DxNext = axLRound(0.5f * tBlend * vNext);

            // Check rounded slope <= 0.5 and correct blend duration if neccesary.
            auto tBlendCorrected = tBlend;
            for (size_t j = 0; j < AxesSize; ++j) {
                auto DxAbsX4 = abs(Dx[j] * 4);
                if (tBlendCorrected < DxAbsX4) {
                    tBlendCorrected = DxAbsX4;
                }
                auto DxNextAbsX4 = abs(DxNext[j] * 4);
                if (tBlendCorrected < DxNextAbsX4) {
                    tBlendCorrected = DxNextAbsX4;
                }
            }

            segments_.emplace_back(tBlendCorrected, Dx, DxNext);
        }

        // Where is no linear segments after last point.
        if (lastPoint)
            return;

        auto tBlendNext = blendDurations_[i + 1];
        auto const &xNext = path_[i + 1];
        auto Dt = static_cast<float>(durations_[i]);
        auto Dx = xNext - x;

        // Segment slope.
        auto v = axCast<float>(Dx) / Dt;

        // Calculate blend x difference the same way as above to be consistent in rounding.
        auto DxBlend = axLRound(0.5f * tBlend * v);
        auto DxBlendNext = axLRound(0.5f * tBlendNext * v);

        auto tBlendPart = (tBlend + tBlendNext) * 0.5f;
        auto tLine = Dt - tBlendPart;
        auto DxLine = Dx - (DxBlend + DxBlendNext);
        auto rtLine = lTruncTowardInf(tLine);

        if (rtLine > 0) {
            segments_.emplace_back(rtLine, DxLine);
        }
    }

    std::vector<Ai> path_;
    std::vector<int32_t> durations_;
    std::vector<int32_t> blendDurations_;
    Segments segments_;
};

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

    void start() {
        if (!segments_.empty()) {
            running_ = true;
        }
        sg_ = segments_.begin();

        // Start timer.
        ticker_->attach_us(this, &SegmentsExecutor::tick, 1000000 / ticksPerSecond_);
    }

    void tick() {
        if (sg_->dt != 0) {
            // Integrate next interval.
            tickI<0>();
        } else if (++sg_ != segments_.end()) {
            // If there is next segment then integrate it's first interval.
            tickI<0>();
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
