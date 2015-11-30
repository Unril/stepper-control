#pragma once

#include "Axes.h"

namespace StepperControl {
/*
From "Turning Paths Into Trajectories Using Parabolic Blends"
by Tobias Kunz and Mike Stilman
https://github.com/willowgarage/arm_navigation/blob/master/constraint_aware_spline_smoother/include/constraint_aware_spline_smoother/KunzStilman/Trajectory.h

Calculate initial durations.
DT_i = max_j(|q_i+1[j] - q_i[j]|/vmax[j])
v_i = (q_i+1 - q_i)/dT_i

And blend durations.
tb_i = max_j(|v_i[j] - v_i-1[j]|/amax[j])
a_i = (v_i - v_i-1)/tb_i

Apply slow down factor to velocities if blends overlap.
f_i = sqrt(min(dT_i-1, dT_i)/tb_i)
*/
template <size_t AxesSize>
class PathToTrajectoryConverter {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;

    explicit PathToTrajectoryConverter(std::vector<Ai> &path) : path_(path) {
        maxVelocity_.fill(0.5f);
        maxAcceleration_.fill(0.1f);
    }

    // Should be less or equal that 0.4 to proper segment generation.
    // In steps per tick.
    void setMaxVelocity(Af const &maxVel) {
        scAssert(all(gt(maxVel, axZero<Af>())));
        maxVelocity_ = maxVel;
    }

    // In steps per tick^2.
    void setMaxAcceleration(Af const &maxAccel) {
        scAssert(all(gt(maxAccel, axZero<Af>())));
        maxAcceleration_ = maxAccel;
    }

    // Remove way-point if difference between it and next is less than threshold for all axes.
    // Does not remove first or last way-points.
    void removeCloseWaypoints(Ai const &threshold) {
        scAssert(all(ge(threshold, axZero<Ai>())));

        auto remove = true;
        while (remove) {
            remove = false;
            auto it = path_.begin();
            auto next = it;
            if (next != path_.end()) {
                ++next;
            }
            while (next != path_.end()) {
                if (all(le(axAbs(*it - *next), threshold))) {
                    remove = true;
                    auto nextNext = next;
                    ++nextNext;
                    if (it == path_.begin()) {
                        it = path_.erase(next);
                    } else if (nextNext == path_.end()) {
                        it = path_.erase(it);
                    } else {
                        *it = (*it + *next) / 2;
                        ++it;
                        it = path_.erase(it);
                    }
                    next = it;
                    if (next != path_.end()) {
                        ++next;
                    }
                } else {
                    it = next;
                    ++next;
                }
            }
        }
    }

    void update() {
        scAssert(!path_.empty());
        resizeVectorsToFitPath();
        calculateTimeBetweenWaypointsAndInitialVelocitiesOfLinearSegments();
        applySlowDownFactor();
    }

    std::vector<Af> const &velocities() const { return velocities_; }

    std::vector<Af> const &accelerations() const { return accelerations_; }

    std::vector<float> const &durations() const { return Dts_; }

    std::vector<float> &durations() { return Dts_; }

    std::vector<float> const &blendDurations() const { return tbs_; }

    std::vector<float> &blendDurations() { return tbs_; }

    Af const &maxVelocity() const { return maxVelocity_; }

    Af const &maxAcceleration() const { return maxAcceleration_; }

  private:
    void resizeVectorsToFitPath() {
        velocities_.resize(path_.size() - 1);
        Dts_.resize(path_.size() - 1);
        accelerations_.resize(path_.size());
        tbs_.resize(path_.size());
    }

    void calculateTimeBetweenWaypointsAndInitialVelocitiesOfLinearSegments() {
        for (size_t i = 0; i < path_.size() - 1; i++) {
            Dts_[i] = 0.0f;
            for (size_t j = 0; j < path_[i].size(); j++) {
                Dts_[i] =
                    std::max(Dts_[i], (std::abs(path_[i + 1][j] - path_[i][j]) / maxVelocity_[j]));
            }
            velocities_[i] = axCast<float>(path_[i + 1] - path_[i]) / Dts_[i];
        }
    }

    void applySlowDownFactor() {
        const auto eps = 1.e-6f;

        int numBlendsSlowedDown = std::numeric_limits<int>::max();
        std::vector<float> slowDownFactors(path_.size());
        while (numBlendsSlowedDown > 1) {
            numBlendsSlowedDown = 0;
            fill(slowDownFactors.begin(), slowDownFactors.end(), 1.0f);

            for (size_t i = 0; i < path_.size(); i++) {
                // Calculate blend duration and acceleration.
                Af previousVelocity = (i == 0) ? axConst<AxesSize>(0.f) : velocities_[i - 1];
                Af nextVelocity = (i == path_.size() - 1) ? axConst<AxesSize>(0.f) : velocities_[i];
                tbs_[i] = 0.0f;
                for (size_t j = 0; j < path_[i].size(); j++) {
                    tbs_[i] = std::max(
                        tbs_[i],
                        (abs(nextVelocity[j] - previousVelocity[j]) / maxAcceleration_[j]));
                    accelerations_[i] = (nextVelocity - previousVelocity) / tbs_[i];
                }

                // Calculate slow down factor such that the blend phase replaces at most half of the
                // neighboring linear trajectory.
                if ((i > 0 && tbs_[i] > Dts_[i - 1] + eps &&
                     tbs_[i - 1] + tbs_[i] > 2.0 * Dts_[i - 1] + eps) ||
                    (i < path_.size() - 1 && tbs_[i] > Dts_[i] + eps &&
                     tbs_[i] + tbs_[i + 1] > 2.0 * Dts_[i] + eps)) {
                    numBlendsSlowedDown++;
                    auto maxDuration = std::min(
                        i == 0 ? std::numeric_limits<float>::max() : Dts_[i - 1],
                        i == path_.size() - 1 ? std::numeric_limits<float>::max() : Dts_[i]);
                    slowDownFactors[i] = std::sqrt(maxDuration / tbs_[i]);
                }
            }

            // Apply slow down factors to linear trajectory.
            for (size_t i = 0; i < path_.size() - 1; i++) {
                velocities_[i] *= std::min(slowDownFactors[i], slowDownFactors[i + 1]);
                Dts_[i] = Dts_[i] / std::min(slowDownFactors[i], slowDownFactors[i + 1]);
            }
        }
    }

    std::vector<Ai> &path_; // In steps.
    std::vector<Af> velocities_;
    std::vector<Af> accelerations_;
    std::vector<float> Dts_;
    std::vector<float> tbs_;
    Af maxVelocity_;
    Af maxAcceleration_;
};
}
