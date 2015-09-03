#pragma once

#include "Interfaces.h"
#include "PathToTrajectoryConverter.h"
#include "TrajectoryToSegmentsConverter.h"

namespace StepperControl {

// Interpreter reacts to callbacks from parser and creates commands from them.
template <size_t AxesSize>
class GCodeInterpreter : public IGCodeInterpreter<AxesSize> {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;
    using Sg = Segment<AxesSize>;

    explicit GCodeInterpreter(ISegmentsExecutor<AxesSize> *exec,
                              Printer *printer = Printer::instance())
        : executor_(exec), mode_(DistanceMode::Absolute), homingVel_(axConst<Af>(0.01f)),
          maxVel_(axConst<Af>(0.5f)), maxAcc_(axConst<Af>(0.1f)), stepPerUnit_(axConst<Af>(1.f)),
          ticksPerSec_(1), printer_(printer) {
        scAssert(exec != nullptr);
        scAssert(printer != nullptr);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Callbacks
    ///////////////////////////////////////////////////////////////////////////

    void feedrateOverride(float) override {}

    // Max velocity and acceleration should be set before this call.
    void linearMove(Af const &positionInUnits, float feed) override {
        if (path_.empty()) {
            // Start path from current position.
            path_.push_back(executor_->position());
        }

        // Update only finite values. Infinite values corresponds to unset axes.
        auto position = path_.back();
        for (size_t i = 0; i < AxesSize; ++i) {
            if (!std::isfinite(positionInUnits[i])) {
                continue;
            }
            auto steps = lroundf(positionInUnits[i] * stepPerUnit_[i]);
            if (mode_ == DistanceMode::Relative) {
                position[i] += steps;
            } else {
                position[i] = steps;
            }
        }

        // Add only position different from previous.
        if (position != path_.back()) {
            path_.push_back(position);
        }
    }

    // Same as linear move.
    void g0RapidMove(Af const &pos) override { linearMove(pos, inf()); }

    // Same as linear move.
    void g1LinearMove(Af const &pos, float feed) override { linearMove(pos, feed); }

    // Ticks per second should be set before this call.
    void g4Wait(float sec) override {
        appendPathToSegments();
        auto ticks = lround(sec * ticksPerSec_);
        segments_.push_back(Sg(ticks));
    }

    // Homing velocity should be set before this call.
    void g28RunHomingCycle() override {
        appendPathToSegments();
        segments_.push_back(Sg(homingVel_));
    }

    void g90g91DistanceMode(DistanceMode mode) override { mode_ = mode; }

    // Ticks per second and steps per unit length should be set before this call.
    void m100MaxVelocityOverride(Af const &unitsPerSec) override {
        auto stepsPerTick = unitsPerSec * stepPerUnit_ / static_cast<float>(ticksPerSec_);
        copyOnlyFinite(stepsPerTick, maxVel_);
        scAssert(all(gt(maxVel_, axZero<Af>())));
        applyInplace(maxVel_, Clamp<float>(0, 0.5f));
    }

    // Ticks per second and steps per unit length should be set before this call.
    void m101MaxAccelerationOverride(Af const &unitsPerSecSqr) override {
        auto stepsPerTickSqr =
            unitsPerSecSqr * stepPerUnit_ / (static_cast<float>(ticksPerSec_) * ticksPerSec_);
        copyOnlyFinite(stepsPerTickSqr, maxAcc_);
        scAssert(all(gt(maxAcc_, axZero<Af>())));
    }

    // Steps per unit length should be set before this call.
    void m102StepsPerUnitLengthOverride(Af const &stepsPerUnit) override {
        copyOnlyFinite(stepsPerUnit, stepPerUnit_);
        scAssert(all(gt(stepPerUnit_, axZero<Af>())));
    }

    // Ticks per second and steps per unit length should be set before this call.
    void m103HomingVelocityOverride(Af const &unitsPerSec) override {
        auto stepsPerTick = unitsPerSec * stepPerUnit_ / static_cast<float>(ticksPerSec_);
        copyOnlyFinite(stepsPerTick, homingVel_);
        scAssert(all(gt(homingVel_, axZero<Af>())));
        applyInplace(homingVel_, Clamp<float>(0.0f, 0.5f));
    }

    void m104PrintInfo() const override {
        *printer_ << "Max velocity: " << maxVel_ << "\nMax acceleration: " << maxAcc_
                  << "\nHoming velocity: " << homingVel_
                  << "\nSteps per unit length: " << stepPerUnit_
                  << "\nMode: " << (mode_ == DistanceMode::Absolute ? "Absolute" : "Relative")
                  << "\nTicks per second: " << ticksPerSec_ << "\nPath (" << path_.size() << "): ";
        for (auto &p : path_) {
            *printer_ << "\n    " << p;
        }
        *printer_ << "\n";
    }

    void error(size_t pos, const char *line, const char *reason) override {
        *printer_ << "Error: " << reason << " at " << pos << " in " << line << "\n";
    }

    void start() override {
        loadSegmentsToExecutor();
        executor_->start();
    }

    void stop() override { executor_->stop(); }

    void printCurrentPosition() const override {
        *printer_ << "Position: " << toUnits(executor_->position()) << "\n";
    }

    void clearCommandsBuffer() override {
        path_.clear();
        segments_.clear();
    }

    ///////////////////////////////////////////////////////////////////////////
    // State
    ///////////////////////////////////////////////////////////////////////////

    void setTicksPerSecond(int32_t tps) {
        scAssert(tps > 0);
        ticksPerSec_ = tps;
        executor_->setTicksPerSecond(tps);
    }

    Af const &maxVelocity() const { return maxVel_; }

    Af const &maxAcceleration() const { return maxAcc_; }

    Af const &stepsPerUnitLength() const { return stepPerUnit_; }

    Af const &homingVelocity() const { return homingVel_; }

    int32_t ticksPerSecond() const { return ticksPerSec_; }

    std::vector<Sg> const &segments() const { return segments_; }

    std::vector<Ai> const &path() const { return path_; }

    Af toUnits(Ai const &pos) const { return axCast<float>(pos) / stepPerUnit_; }

    void clearAll() {
        executor_->stop();
        segments_ = move(std::vector<Sg>());
        path_ = move(std::vector<Ai>());
        executor_->setSegments(move(std::vector<Sg>()));
    }

  private:
    void loadSegmentsToExecutor() {
        appendPathToSegments();
        path_.clear();
        executor_->setSegments(move(segments_));
        segments_.clear();
    }

    void appendPathToSegments() {
        if (path_.empty()) {
            return;
        }
        auto lastPosition = path_.back();

        PathToTrajectoryConverter<AxesSize> trajGen;
        trajGen.setMaxVelocity(maxVelocity());
        trajGen.setMaxAcceleration(maxAcceleration());
        trajGen.setPath(move(path_));
        trajGen.update();

        path_.clear();
        path_.push_back(lastPosition);

        TrajectoryToSegmentsConverter<AxesSize> segGen;
        segGen.setPath(move(trajGen.path()));
        segGen.setBlendDurations(move(trajGen.blendDurations()));
        segGen.setDurations(move(trajGen.durations()));
        segGen.update(segments_);
    }

    ISegmentsExecutor<AxesSize> *executor_;
    std::vector<Sg> segments_;
    std::vector<Ai> path_;
    DistanceMode mode_;
    Af homingVel_;
    Af maxVel_;
    Af maxAcc_;
    Af stepPerUnit_;
    int32_t ticksPerSec_;
    Printer *printer_;
};
}
