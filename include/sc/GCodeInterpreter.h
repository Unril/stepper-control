#pragma once

#include "Interfaces.h"
#include "PathToTrajectoryConverter.h"
#include "TrajectoryToSegmentsConverter.h"

namespace StepperControl {

template <typename T>
struct Clamp {
    Clamp(T minV, T maxV) : minVal(minV), maxVal(maxV) {}
    T operator()(T val) { return std::min(maxVal, std::max(minVal, val)); }
    T minVal, maxVal;
};

// Interpreter reacts to callbacks from parser and creates commands from them.
template <typename AxesTraits = DefaultAxesTraits>
class GCodeInterpreter : public IGCodeInterpreter<AxesTraits> {
  public:
    using Af = TAf<AxesTraits::size>;
    using Ai = TAi<AxesTraits::size>;
    using Sg = TSg<AxesTraits::size>;
    using Sgs = TSgs<AxesTraits::size>;

    explicit GCodeInterpreter(ISegmentsExecutor<AxesTraits> *exec,
                              Printer *printer = Printer::instance())
        : executor_(exec), mode_(DistanceMode::Absolute), homingVel_(axConst<Af>(0.01f)),
          maxVel_(axConst<Af>(0.5f)), maxAcc_(axConst<Af>(0.1f)), stepPerUnit_(axConst<Af>(1.f)),
          maxDistance_(axInf<Af>()), ticksPerSec_(1), printer_(printer) {
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
        for (size_t i = 0; i < AxesTraits::size; ++i) {
            auto pos = positionInUnits[i];
            if (!std::isfinite(pos)) {
                continue;
            }
            auto maxDist = maxDistance_[i];
            if (pos > maxDist) {
                pos = maxDist;
            } else if (std::isfinite(maxDist)) {
                pos = 0;
            }

            auto steps = lroundf(pos * stepPerUnit_[i]);
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
        *printer_ << "Max velocity: " << maxVel_ << eol << "Max acceleration: " << maxAcc_ << eol
                  << "Homing velocity: " << homingVel_ << eol
                  << "Steps per unit length: " << stepPerUnit_ << eol
                  << "Max distance: " << maxDistance_ << eol
                  << "Mode: " << (mode_ == DistanceMode::Absolute ? "Absolute" : "Relative") << eol
                  << "Ticks per second: " << ticksPerSec_ << eol << "Path (" << path_.size()
                  << "): ";
        for (auto &p : path_) {
            *printer_ << eol << "    " << p;
        }
        *printer_ << eol;
    }

    // Set max traveling distance in units. If it's not inf then all moves will be trimmed from zero
    // to that value.
    void m105MaxDistanceOverride(Af const &units) override {
        copyOnlyFinite(units, maxDistance_);
        scAssert(all(gt(maxDistance_, axZero<Af>())));
    }

    void m106PrintAxesConfiguration() override {
        *printer_ << "Axes: " << AxesTraits::names() << eol;
    }

    void error(const char *reason) override { *printer_ << "Error: " << reason << eol; }

    void start() override {
        if (executor_->isRunning()) {
            error("already running");
            return;
        }
        executor_->setSegments(move(Sgs()));
        loadSegmentsToExecutor();
        executor_->start();
    }

    void stop() override { executor_->stop(); }

    bool isRunning() const override { return executor_->isRunning(); }

    void printCurrentPosition() const override {
        *printer_ << "Position: " << toUnits(executor_->position()) << eol;
    }

    void printCompleted() {
        printCurrentPosition();
        *printer_ << "Completed" << eol;
    }

    void clearCommandsBuffer() override {
        path_ = move(std::vector<Ai>());
        segments_ = move(Sgs());
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

    Af const &maxDistance() const { return maxDistance_; }

    int32_t ticksPerSecond() const { return ticksPerSec_; }

    Sgs const &segments() const { return segments_; }

    std::vector<Ai> const &path() const { return path_; }

    Af toUnits(Ai const &pos) const { return axCast<float>(pos) / stepPerUnit_; }

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

        PathToTrajectoryConverter<AxesTraits::size> trajGen;
        trajGen.setMaxVelocity(maxVelocity());
        trajGen.setMaxAcceleration(maxAcceleration());
        trajGen.setPath(move(path_));
        trajGen.update();

        path_.clear();
        path_.push_back(lastPosition);

        TrajectoryToSegmentsConverter<AxesTraits::size> segGen;
        segGen.setPath(move(trajGen.path()));
        segGen.setBlendDurations(move(trajGen.blendDurations()));
        segGen.setDurations(move(trajGen.durations()));
        segGen.update(segments_);
    }

    ISegmentsExecutor<AxesTraits> *executor_;
    Sgs segments_;
    std::vector<Ai> path_;
    DistanceMode mode_;
    Af homingVel_;
    Af maxVel_;
    Af maxAcc_;
    Af stepPerUnit_;
    Af maxDistance_;
    int32_t ticksPerSec_;
    Printer *printer_;
};
}
