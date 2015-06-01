#pragma once

#include "Axes.h"

#include <cstdint>

namespace StepperControl {

/* From "Turning Paths Into Trajectories Using Parabolic Blends"
by Tobias Kunz and Mike Stilman
https://github.com/willowgarage/arm_navigation/blob/master/constraint_aware_spline_smoother/include/constraint_aware_spline_smoother/KunzStilman/Trajectory.h

dT_i = max_j(|q_i+1[j] - q_i[j]|/vmax[j])
tb_i = max_j(|v_i[j] - v_i-1[j]|/amax[j])

v_i = (q_i+1 - q_i)/dT_i
a_i = (v_i - v_i-1)/tb_i

f_i = sqrt(min(dT_i-1, dT_i)/tb_i)

*/
template <size_t AxesSize>
class TrajectoryGenerator {
  public:
    using AxesFloat = Axes<float, AxesSize>;
    using AxesInt = Axes<int32_t, AxesSize>;

    TrajectoryGenerator() {
        maxVelocity_.fill(0);
        maxAcceleration_.fill(0);
    }

    // steps
    void setPath(std::vector<AxesInt> const &path) {
        if (path.size() <= 1) {
            return;
        }
        path_ = path;
        velocities_.resize(path_.size() - 1);
        durations_.resize(path_.size() - 1);
        accelerations_.resize(path_.size());
        blendDurations_.resize(path_.size());
    }

    // steps per tick
    void setMaxVelocity(AxesFloat const &axesFloat) { maxVelocity_ = axesFloat; }

    // steps per tick^2
    void setMaxAcceleration(AxesFloat const &axesFloat) { maxAcceleration_ = axesFloat; }

    void updateTrajectory() {
        calculateTimeBetweenWaypointsAndInitialVelocitiesOfLinearSegments();
        applySlowDownFactor();
    }

    std::vector<AxesInt> const &path() const { return path_; }

    std::vector<AxesFloat> const &velocities() const { return velocities_; }

    std::vector<AxesFloat> const &accelerations() const { return accelerations_; }

    std::vector<float> const &durations() const { return durations_; }

    std::vector<float> const &blendDurations() const { return blendDurations_; }

    AxesFloat const &maxVelocity() const { return maxVelocity_; }

    AxesFloat const &maxAcceleration() const { return maxAcceleration_; }

  private:
    void calculateTimeBetweenWaypointsAndInitialVelocitiesOfLinearSegments() {
        for (size_t i = 0; i < path_.size() - 1; i++) {
            durations_[i] = 0.0;
            for (size_t j = 0; j < path_[i].size(); j++) {
                durations_[i] =
                    std::max(durations_[i], (abs(path_[i + 1][j] - path_[i][j]) / maxVelocity_[j]));
            }
            velocities_[i] = cast<float>(path_[i + 1] - path_[i]) / durations_[i];
        }
    }

    void applySlowDownFactor() {
        const auto eps = 0.000001f;

        int numBlendsSlowedDown = std::numeric_limits<int>::max();
        std::vector<float> slowDownFactors(path_.size());
        while (numBlendsSlowedDown > 1) {
            numBlendsSlowedDown = 0;
            fill(slowDownFactors.begin(), slowDownFactors.end(), 1.0f);

            for (size_t i = 0; i < path_.size(); i++) {
                // calculate blend duration and acceleration
                AxesFloat previousVelocity =
                    (i == 0) ? axesConstant<AxesSize>(0.f) : velocities_[i - 1];
                AxesFloat nextVelocity =
                    (i == path_.size() - 1) ? axesConstant<AxesSize>(0.f) : velocities_[i];
                blendDurations_[i] = 0.0;
                for (size_t j = 0; j < path_[i].size(); j++) {
                    blendDurations_[i] = std::max(
                        blendDurations_[i],
                        (abs(nextVelocity[j] - previousVelocity[j]) / maxAcceleration_[j]));
                    accelerations_[i] = (nextVelocity - previousVelocity) / blendDurations_[i];
                }

                // calculate slow down factor such that the blend phase replaces at most half of the
                // neighboring linear segments

                if ((i > 0 && blendDurations_[i] > durations_[i - 1] + eps &&
                     blendDurations_[i - 1] + blendDurations_[i] > 2.0 * durations_[i - 1] + eps) ||
                    (i < path_.size() - 1 && blendDurations_[i] > durations_[i] + eps &&
                     blendDurations_[i] + blendDurations_[i + 1] > 2.0 * durations_[i] + eps)) {
                    numBlendsSlowedDown++;
                    auto maxDuration = std::min(
                        i == 0 ? std::numeric_limits<float>::max() : durations_[i - 1],
                        i == path_.size() - 1 ? std::numeric_limits<float>::max() : durations_[i]);
                    slowDownFactors[i] = sqrt(maxDuration / blendDurations_[i]);
                }
            }

            // apply slow down factors to linear segments
            for (size_t i = 0; i < path_.size() - 1; i++) {
                velocities_[i] *= std::min(slowDownFactors[i], slowDownFactors[i + 1]);
                durations_[i] =
                    durations_[i] / std::min(slowDownFactors[i], slowDownFactors[i + 1]);
            }
        }
    }

    std::vector<AxesInt> path_;
    std::vector<AxesFloat> velocities_;
    std::vector<AxesFloat> accelerations_;
    std::vector<float> durations_;
    std::vector<float> blendDurations_;
    AxesFloat maxVelocity_;
    AxesFloat maxAcceleration_;
};
}
