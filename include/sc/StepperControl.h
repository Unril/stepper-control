#pragma once

#include "Axes.h"

namespace StepperControl {
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

    void update() {
        scAssert(!path_.empty());
        scAssert(path_.size() - 1 == durations_.size());
        scAssert(path_.size() == blendDurations_.size());

        segments_.clear();

        for (size_t i = 0; i < path_.size(); i++) {
            auto firstPoint = (i == 0);
            auto lastPoint = (i == (path_.size() - 1));

            auto q = path_[i];
            auto tb = blendDurations_[i];

            if (lround(tb) > 0) {
                if (firstPoint) {
                    auto qNext = path_[i + 1];
                    auto Dt = durations_[i];
                    auto vNext = axCast<float>(qNext - q) / Dt;

                    segments_.emplace_back(lround(tb), axZero<Ai>(), axLRound(0.5f * tb * vNext));
                } else if (lastPoint) {
                    auto qPrev = path_[i - 1];
                    auto DtPrev = durations_[i - 1];
                    auto v = axCast<float>(q - qPrev) / DtPrev;

                    segments_.emplace_back(lround(tb), axLRound(0.5f * tb * v), axZero<Ai>());
                } else {
                    auto pPrev = path_[i - 1];
                    auto pNext = path_[i + 1];

                    auto Dt = durations_[i];
                    auto DtPrev = durations_[i - 1];
                    auto v = axCast<float>(q - pPrev) / DtPrev;
                    auto vNext = axCast<float>(pNext - q) / Dt;

                    segments_.emplace_back(lround(tb), axLRound(0.5f * tb * v),
                                           axLRound(0.5f * tb * vNext));
                }
            }

            if (!lastPoint) {
                auto tbNext = blendDurations_[i + 1];
                auto qNext = path_[i + 1];
                auto Dt = durations_[i];
                auto Dq = qNext - q;
                auto v = axCast<float>(Dq) / Dt;

                auto Dqb = axLRound(0.5f * tb * v);
                auto DqbNext = axLRound(0.5f * tbNext * v);

                auto tbPart = (tb + tbNext) * 0.5f;
                auto tl = Dt - tbPart;
                auto Dql = Dq - (Dqb + DqbNext);

                if (lround(tl) > 0) {
                    segments_.emplace_back(lround(tl), Dql);
                }
            }
        }
    }

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
    using Sg = Segment<AxesSize>;
    using Segments = std::vector<Sg>;
    using Ai = Axes<int32_t, AxesSize>;

    SegmentsExecutor(TMotor<AxesSize> *motor, TTicker *ticker)
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
    TMotor<AxesSize> *motor_;
    TTicker *ticker_;
    Segments segments_;
    typename Segments::iterator sg_;
    int32_t ticksPerSecond_;
};
}
