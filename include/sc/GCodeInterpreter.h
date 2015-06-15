#pragma once

#include "GCodeParser.h"
#include "Command.h"
#include "PathToTrajectoryConverter.h"
#include "TrajectoryToSegmentsConverter.h"
#include "SegmentsExecutor.h"

namespace StepperControl {
template <size_t AxesSize>
struct ISegmentsExecutor {
    using Ai = Axes<int32_t, AxesSize>;
    using Sg = Segment<AxesSize>;
    using Segments = std::vector<Sg>;

    virtual ~ISegmentsExecutor() {}

    virtual void start() = 0;
    virtual void stop() = 0;
    virtual bool isRunning() = 0;
    virtual Ai const &position() = 0;
    virtual void setSegments(Segments const &segments) = 0;
    virtual void setSegments(Segments &&segments) = 0;
};

// Interpreter reacts to callbacks from parser and creates commands from them.
template <size_t AxesSize>
class GCodeInterpreter : public IGCodeInterpreter<AxesSize> {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;
    using Cmd = Command<AxesSize>;
    using Seg = Segment<AxesSize>;

    explicit GCodeInterpreter(ISegmentsExecutor<AxesSize> *exec)
        : executor_(exec), notifyInterval_(1), nextNotifyTick_(1), wasRunning_(false),
          mode_(Absolute), currentPos_(axConst<Ai>(0)), homingVel_(axConst<Af>(0.1f)),
          maxVel_(axConst<Af>(0.5f)), maxAcc_(axConst<Af>(0.1f)), stepPerUnit_(axConst<Af>(1.f)), ticksPerSec_(1) {
        scAssert(exec != nullptr);
    }

    ///////////////////////////////////////////////////////////////////////////
    // Callbacks
    ///////////////////////////////////////////////////////////////////////////

    void feedrateOverride(float feed) override {}

    // Max velocity and acceleration should be set before this call.
    void linearMove(Af const &positionInUnits, float feed) override {
        // If it is first move command in commands list.
        if (cmds_.empty() || cmds_.back().type() != Cmd::Move) {
            // Start path from current position.
            cmds_.push_back(Cmd({currentPos_}, maxVelocity(), maxAcceleration()));
        }

        auto position = positionInUnits * stepPerUnit_;
        if (mode_ == Relative) {
            position += axCast<float>(currentPos_);
        }

        auto previousPosition = currentPos_;
        tansformOnlyFinite(position, currentPos_, &lroundf);
        if (previousPosition != currentPos_) {
            cmds_.back().push_back(currentPos_);
        }
    }

    // Same as linear move.
    void g0RapidMove(Af const &pos) override { linearMove(pos, inf()); }

    // Same as linear move.
    void g1LinearMove(Af const &pos, float feed) override { linearMove(pos, feed); }

    // Ticks per second should be set before this call.
    void g4Wait(float sec) override {
        auto ticks = lround(sec * ticksPerSec_);
        cmds_.push_back(Cmd(ticks));
    }

    // Homing velocity should be set before this call.
    void g28RunHomingCycle() override { cmds_.push_back(Cmd(homingVel_)); }

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

    void error(size_t pos, const char *line, const char *reason) override {
        printf("Error: %s at %d in %s\n", reason, static_cast<int>(pos), line);
    }

    void start() override {}

    void stop() override {}

    void printInfo() const override {
        printf("Current position: ");
        axPrintf(currentPos_);
        printf("\nTicks per second: %ld", static_cast<long>(ticksPerSec_));
        printf("\nCommands (%ld): ", static_cast<long>(cmds_.size()));
        for (auto &cmd : cmds_) {
            cmd.printInfo();
        }
        printf("\n");
    }

    void clearCommandsBuffer() override { cmds_.clear(); }

    ///////////////////////////////////////////////////////////////////////////
    // Control
    ///////////////////////////////////////////////////////////////////////////

    void update() {
        if (executor_->getStoppedAndReset()) {
            syncPosition();
            axPrintf(currentPositionInUnits());
            printf("\nCompleted\n");
        }
    }

    void printCurrentPosition() {
        axPrintf(currentPositionInUnits());
        printf("\n");
    }

    ///////////////////////////////////////////////////////////////////////////
    // State
    ///////////////////////////////////////////////////////////////////////////

    void setTicksPerSecond(int32_t tps) {
        scAssert(tps > 0);
        ticksPerSec_ = tps;
    }

    void setCurrentPosition(Ai const &pos) { currentPos_ = pos; }

    std::vector<Cmd> const &commands() const { return cmds_; }

    std::vector<Cmd> &commands() { return cmds_; }

    Ai const &currentPosition() const { return currentPos_; }

    Af const &maxVelocity() const { return maxVel_; }

    Af const &maxAcceleration() const { return maxAcc_; }

    Af const &stepsPerUnitLength() const { return stepPerUnit_; }

    Af const &homingVelocity() const { return homingVel_; }

    int32_t ticksPerSecond() const { return ticksPerSec_; }

    void clearCommands() { cmds_.clear(); }

    bool hasCommands() const { return !cmds_.empty(); }

    void setNotifyDelay(float sec) {}

  private:
    std::vector<Seg> moveCommandToSegments(Cmd &cmd) const {
        PathToTrajectoryConverter<AxesSize> trajGen;
        trajGen.setMaxVelocity(cmd->maxVelocity());
        trajGen.setMaxAcceleration(cmd->maxAcceleration());
        trajGen.setPath(move(cmd->path()));
        trajGen.update();

        TrajectoryToSegmentsConverter<AxesSize> segGen;
        segGen.setPath(move(trajGen.path()));
        segGen.setBlendDurations(move(trajGen.blendDurations()));
        segGen.setDurations(move(trajGen.durations()));
        segGen.update();

        return move(segGen.segments());
    }

    Af currentPositionInUnits() const {
        return axCast<float>(executor_->position()) / stepPerUnit_;
    }

    void syncPosition() { setCurrentPosition(executor_->position()); }

    ISegmentsExecutor<AxesSize> *executor_;
    int32_t notifyInterval_;
    int32_t nextNotifyTick_;
    bool wasRunning_;

    std::vector<Cmd> cmds_;
    std::vector<Seg> segments_;
    DistanceMode mode_;
    Ai currentPos_;
    Af homingVel_;
    Af maxVel_;
    Af maxAcc_;
    Af stepPerUnit_;
    int32_t ticksPerSec_;
};
}
