#pragma once

#include "GCodeParser.h"

namespace StepperControl {
template <size_t AxesSize>
struct Command {
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;

    enum Type {
        Move,
        Wait,
        Homing,
    };

    // Move
    Command(std::vector<Ai> const &path, Af const &maxVel, Af const &maxAccel)
        : type_(Move), path_(path), maxVelocity_(maxVel), maxAcceleration_(maxAccel) {}

    // Wait
    explicit Command(int32_t waitDuration) : type_(Wait), waitDuration_(waitDuration) {}

    // Homing
    explicit Command(Af const &homingVelocity) : type_(Homing), homingVelocity_(homingVelocity) {}

    inline void push_back(Ai const &waypoint) { path_.push_back(waypoint); }

    inline bool empty() const { return path_.empty(); }

    inline Type const &type() const { return type_; }

    inline std::vector<Ai> const &path() const {
        scAssert(type() == Move);
        return path_;
    }

    inline Af const &maxVelocity() const {
        scAssert(type() == Move);
        return maxVelocity_;
    }

    inline Af const &homingVelocity() const {
        scAssert(type() == Homing);
        return homingVelocity_;
    }

    inline Af const &maxAcceleration() const {
        scAssert(type() == Move);
        return maxAcceleration_;
    }

    inline int32_t waitDuration() const {
        scAssert(type() == Wait);
        return waitDuration_;
    }

  private:
    Type type_;

    // In steps.
    std::vector<Ai> path_;

    // In steps per tick.
    Af maxVelocity_;

    union {
        // In steps per tick.
        Af homingVelocity_;

        // In steps per tick^2.
        Af maxAcceleration_;

        // In ticks.
        int32_t waitDuration_;
    };
};

// Intepreter reacts to callbacks from parser and creates path from them.
template <size_t AxesSize>
class GCodeInterpreter : public GCodeParserCallbacks<AxesSize> {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;
    using Cmd = Command<AxesSize>;

    GCodeInterpreter()
        : mode_(Absolute), currentPos_(axConst<Ai>(0)), homingVel_(axConst<Af>(0.1f)),
          maxVel_(axConst<Af>(0.5f)), maxAcc_(axConst<Af>(0.1f)), stPerUnit_(axConst<Af>(1.f)),
          ticksPerSec_(1) {}

    void feedrateOverride(float feed) override {}

    // Max velocity and acceleration should be set befor this call.
    void linearMove(Af const &positionInUnits, float feed) override {
        if (cmds_.empty() || cmds_.back().type() != Cmd::Move) {
            // Start path from current position.
            cmds_.push_back(Cmd({currentPos_}, maxVelocity(), maxAcceleration()));
        }

        auto newPosition = positionInUnits * stPerUnit_;
        if (mode_ == Relative) {
            newPosition += axCast<float>(currentPos_);
        }
        auto previousPosition = currentPos_;
        tansformOnlyFinite(newPosition, currentPos_, &lroundf);
        if (previousPosition != currentPos_) {
            cmds_.back().push_back(currentPos_);
        }
    }

    // Same as linear move.
    void g0RapidMove(Af const &pos) override { linearMove(pos, inf()); }

    // Same as linear move.
    void g1LinearMove(Af const &pos, float feed) override { linearMove(pos, feed); }

    // Ticks per second should be set before this call.
    void g4Wait(float sec) override { cmds_.push_back(Cmd(lround(sec * ticksPerSec_))); }

    // Homing velocity should be set befor this call.
    void g28RunHomingCycle() override { cmds_.push_back(Cmd(homingVel_)); }

    void g90g91DistanceMode(DistanceMode mode) override { mode_ = mode; }

    // Ticks per second and steps per unit lenght should be set before this call.
    void m100MaxVelocityOverride(Af const &vel) override {
        auto c = stPerUnit_ / static_cast<float>(ticksPerSec_);
        copyOnlyFinite(vel * c, maxVel_);
        scAssert(all(gt(maxVel_, axZero<Af>())));
        applyInplace(maxVel_, Clamp<float>(0, 0.5f));
    }

    // Ticks per second and steps per unit lenght should be set before this call.
    void m101MaxAccelerationOverride(Af const &acc) override {
        auto c = stPerUnit_ / static_cast<float>(ticksPerSec_ * ticksPerSec_);
        copyOnlyFinite(acc * c, maxAcc_);
        scAssert(all(gt(maxAcc_, axZero<Af>())));
    }

    // Steps per unit lenght should be set before this call.
    void m102StepsPerUnitLengthOverride(Af const &spl) override {
        copyOnlyFinite(spl, stPerUnit_);
        scAssert(all(gt(stPerUnit_, axZero<Af>())));
    }

    // Ticks per second and steps per unit lenght should be set before this call.
    void m103HomingVelocityOverride(Af const &vel) override {
        auto c = stPerUnit_ / static_cast<float>(ticksPerSec_);
        copyOnlyFinite(vel * c, homingVel_);
        scAssert(all(gt(homingVel_, axZero<Af>())));
        applyInplace(homingVel_, Clamp<float>(0, 0.5f));
    }

    void error(size_t pos, const char *line, const char *reason) override {
        std::cout << "Error: " << reason << " at " << pos << " in " << line;
    }

    void setTicksPerSecond(int32_t tps) {
        scAssert(tps > 0);
        ticksPerSec_ = tps;
    }

    std::vector<Cmd> const &commands() const { return cmds_; }

    std::vector<Cmd> &commands() { return cmds_; }

    Ai const &currentPosition() const { return currentPos_; }

    Af const &maxVelocity() const { return maxVel_; }

    Af const &maxAcceleration() const { return maxAcc_; }

    Af const &stepsPerUnitLength() const { return stPerUnit_; }

    Af const &homingVelocity() const { return homingVel_; }

    int32_t ticksPerSecond() const { return ticksPerSec_; }

  private:
    std::vector<Cmd> cmds_;
    DistanceMode mode_;
    Ai currentPos_;
    Af homingVel_;
    Af maxVel_;
    Af maxAcc_;
    Af stPerUnit_;
    int32_t ticksPerSec_;
};
}
