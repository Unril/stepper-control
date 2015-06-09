#pragma once

#include "GCodeParser.h"
#include "Command.h"

namespace StepperControl {

// Intepreter reacts to callbacks from parser and creates commands from them.
template <size_t AxesSize>
class GCodeInterpreter : public GCodeParserCallbacks<AxesSize> {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;
    using Cmd = Command<AxesSize>;

    GCodeInterpreter()
        : mode_(Absolute), currentPos_(axConst<Ai>(0)), homingVel_(axConst<Af>(0.1f)),
          maxVel_(axConst<Af>(0.5f)), maxAcc_(axConst<Af>(0.1f)), stepPerUnit_(axConst<Af>(1.f)),
          ticksPerSec_(1) {}

    void feedrateOverride(float feed) override {}

    // Max velocity and acceleration should be set befor this call.
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

    // Homing velocity should be set befor this call.
    void g28RunHomingCycle() override { cmds_.push_back(Cmd(homingVel_)); }

    void g90g91DistanceMode(DistanceMode mode) override { mode_ = mode; }

    // Ticks per second and steps per unit lenght should be set before this call.
    void m100MaxVelocityOverride(Af const &unitsPerSec) override {
        auto stepsPerTick = unitsPerSec * stepPerUnit_ / static_cast<float>(ticksPerSec_);
        copyOnlyFinite(stepsPerTick, maxVel_);
        scAssert(all(gt(maxVel_, axZero<Af>())));
        applyInplace(maxVel_, Clamp<float>(0, 0.5f));
    }

    // Ticks per second and steps per unit lenght should be set before this call.
    void m101MaxAccelerationOverride(Af const &unitsPerSecSqr) override {
        auto stepsPerTickSqr =
            unitsPerSecSqr * stepPerUnit_ / static_cast<float>(ticksPerSec_ * ticksPerSec_);
        copyOnlyFinite(stepsPerTickSqr, maxAcc_);
        scAssert(all(gt(maxAcc_, axZero<Af>())));
    }

    // Steps per unit lenght should be set before this call.
    void m102StepsPerUnitLengthOverride(Af const &stepsPerUnit) override {
        copyOnlyFinite(stepsPerUnit, stepPerUnit_);
        scAssert(all(gt(stepPerUnit_, axZero<Af>())));
    }

    // Ticks per second and steps per unit lenght should be set before this call.
    void m103HomingVelocityOverride(Af const &unitsPerSec) override {
        auto stepsPerTick = unitsPerSec * stepPerUnit_ / static_cast<float>(ticksPerSec_);
        copyOnlyFinite(stepsPerTick, homingVel_);
        scAssert(all(gt(homingVel_, axZero<Af>())));
        applyInplace(homingVel_, Clamp<float>(0.0f, 0.5f));
    }

    void error(size_t pos, const char *line, const char *reason) override {
        std::cout << "Error: " << reason << " at " << pos << " in " << line;
    }

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

    bool hasCommands() { return !cmds_.empty(); }

  private:
    std::vector<Cmd> cmds_;
    DistanceMode mode_;
    Ai currentPos_;
    Af homingVel_;
    Af maxVel_;
    Af maxAcc_;
    Af stepPerUnit_;
    int32_t ticksPerSec_;
};
}
