#pragma once

#include "Interfaces.h"
#include "PathToTrajectoryConverter.h"
#include "TrajectoryToSegmentsConverter.h"

namespace StepperControl {
template <typename T = float>
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
        : executor_(exec), mode_(DistanceMode::Absolute), homingVelUnitsPerSec_(axConst<Af>(0.01f)),
          maxVelUnitsPerSec_(axConst<Af>(0.5f)), maxAccUnitsPerSec2_(axConst<Af>(0.1f)),
          stepPerUnit_(axConst<Af>(1.f)), maxDistanceUnits_(axInf<Af>()), ticksPerSec_(1),
          printer_(printer) {
        scAssert(exec != nullptr);
        scAssert(printer != nullptr);
    }

    ///////////////////////////////////////////////////////////////////////////
    // G Commands
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
            auto maxDist = maxDistanceUnits_[i];
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
        auto ticks = lround(sec * ticksPerSecond());
        segments_.push_back(Sg(ticks));
    }

    void g28RunHomingCycle() override {
        appendPathToSegments();
        segments_.push_back(Sg(homingVelocity()));
    }

    void g90g91DistanceMode(DistanceMode mode) override { mode_ = mode; }

    ///////////////////////////////////////////////////////////////////////////
    // M commands
    ///////////////////////////////////////////////////////////////////////////

    void m100MaxVelocityOverride(Af const &unitsPerSec) override {
        copyOnlyFinite(unitsPerSec, maxVelUnitsPerSec_);
        scAssert(all(gt(maxVelUnitsPerSec_, axZero<Af>())));
    }

    void m101MaxAccelerationOverride(Af const &unitsPerSecSqr) override {
        copyOnlyFinite(unitsPerSecSqr, maxAccUnitsPerSec2_);
        scAssert(all(gt(maxAccUnitsPerSec2_, axZero<Af>())));
    }

    void m102StepsPerUnitLengthOverride(Af const &stepsPerUnit) override {
        copyOnlyFinite(stepsPerUnit, stepPerUnit_);
        scAssert(all(gt(stepPerUnit_, axZero<Af>())));
    }

    void m103HomingVelocityOverride(Af const &unitsPerSec) override {
        copyOnlyFinite(unitsPerSec, homingVelUnitsPerSec_);
        scAssert(all(gt(homingVelUnitsPerSec_, axZero<Af>())));
    }

    void m104PrintInfo() const override {
        *printer_ << "Max velocity: " << maxVelUnitsPerSec_ << eol
                  << "Max acceleration: " << maxAccUnitsPerSec2_ << eol
                  << "Homing velocity: " << homingVelUnitsPerSec_ << eol
                  << "Steps per unit length: " << stepPerUnit_ << eol
                  << "Max distance: " << maxDistanceUnits_ << eol
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
        copyOnlyFinite(units, maxDistanceUnits_);
        scAssert(all(gt(maxDistanceUnits_, axZero<Af>())));
    }

    void m106PrintAxesConfiguration() override {
        *printer_ << "Axes: " << AxesTraits::names() << eol;
    }

    ///////////////////////////////////////////////////////////////////////////
    // Others
    ///////////////////////////////////////////////////////////////////////////

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

    void printCompleted() const {
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

    // Steps per tick
    Af maxVelocity() const {
        return apply(maxVelUnitsPerSec_ * stepPerUnit_ / static_cast<float>(ticksPerSec_),
                     Clamp<>(0.0f, 0.5f));
    }

    // Steps per tick per tick
    Af maxAcceleration() const {
        return maxAccUnitsPerSec2_ * stepPerUnit_ /
               (static_cast<float>(ticksPerSec_) * ticksPerSec_);
    }

    Af const &stepsPerUnitLength() const { return stepPerUnit_; }

    // Steps per tick
    Af homingVelocity() const {
        return apply(homingVelUnitsPerSec_ * stepPerUnit_ / static_cast<float>(ticksPerSec_),
                     Clamp<>(0.0f, 0.5f));
    }

    // Steps
    Af maxDistance() const { return maxDistanceUnits_ * stepPerUnit_; }

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
    Af homingVelUnitsPerSec_;
    Af maxVelUnitsPerSec_;
    Af maxAccUnitsPerSec2_;
    Af stepPerUnit_;
    Af maxDistanceUnits_;
    int32_t ticksPerSec_;
    Printer *printer_;
};
}
