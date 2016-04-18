#pragma once

#include "PathToTrajectoryConverter.h"
#include "Segment.h"
#include "TrajectoryToSegmentsConverter.h"

namespace StepperControl {

template <typename T = float>
struct Clamp {
    Clamp(T minV, T maxV) : minVal(minV), maxVal(maxV) {}
    T operator()(T val) { return std::min(maxVal, std::max(minVal, val)); }
    T minVal, maxVal;
};

template <typename T>
Clamp<T> clamp(T minV, T maxV) {
    return Clamp<T>(minV, maxV);
}

template <size_t size>
struct Command {
    using Af = TAf<size>;

    struct MoveCmd {
        Af pos;
        Af vel;
        Af acc;
        DistanceMode mode;
    };

    struct WaitCmd {
        float sec;
    };

    struct HomingCmd {
        Af vel;
    };

    enum Type {
        Move,
        Wait,
        Homing,
    } type;

    union {
        MoveCmd move;
        WaitCmd wait;
        HomingCmd homing;
    };

    Command(Af const &pos, Af const &vel, Af const &acc, DistanceMode mode = DistanceMode::Absolute)
        : type(Move), move({pos, vel, acc, mode}) {}

    Command(float sec) : type(Wait), wait({sec}) {}

    Command(Af const &vel) : type(Homing), homing({vel}) {}

    friend bool operator==(Command const &lhs, Command const &rhs) {
        if (lhs.type != rhs.type) {
            return false;
        }
        switch (lhs.type) {
        case Move:
            return memcmp(&lhs.move, &rhs.move, sizeof(MoveCmd)) == 0;
        case Wait:
            return memcmp(&lhs.wait, &rhs.wait, sizeof(WaitCmd)) == 0;
        case Homing:
            return memcmp(&lhs.homing, &rhs.homing, sizeof(HomingCmd)) == 0;
        default:
            scAssert(!"Wrong type");
            return false;
        }
    }

    friend bool operator!=(Command const &lhs, Command const &rhs) { return !(lhs == rhs); }

    friend Printer &operator<<(Printer &p, Command const &cmd) {
        switch (cmd.type) {
        case Move:
            p << "Move: " << cmd.move.pos;
            break;
        case Wait:
            p << "Wait: " << cmd.wait.sec << "s";
            break;
        case Homing:
            p << "Homing";
            break;
        default:
            scAssert(!"Wrong type");
            break;
        }
        return p;
    }
};

/*
 Interpreter reacts to callbacks from parser and creates commands from them.


struct ISegmentsExecutor {
    virtual void start() {}
    virtual void stop() {}
    virtual bool isRunning() const {}
    virtual Ai const &position() const {}
    virtual void setPosition(Ai const &) {}
    virtual void setSegments(Sgs const &) {}
    virtual void setSegments(Sgs &&) {}
    virtual void setTicksPerSecond(int32_t) {}
};
 */
template <typename ISegmentsExecutor, typename AxesTraits = DefaultAxesTraits>
class GCodeInterpreter {
  public:
    using Af = TAf<AxesTraits::size>;
    using Ai = TAi<AxesTraits::size>;
    using Sg = TSg<AxesTraits::size>;
    using Cmd = Command<AxesTraits::size>;

    explicit GCodeInterpreter(ISegmentsExecutor *exec, Printer *printer = Printer::instance())
        : executor_(exec), mode_(DistanceMode::Absolute), homingVelUnitsPerSec_(axConst<Af>(1.f)),
          maxVelUnitsPerSec_(axConst<Af>(1.f)), maxAccUnitsPerSec2_(axConst<Af>(1.f)),
          stepPerUnit_(axConst<Af>(1.f)), minPosUnits_(axInf<Af>()), maxPosUnits_(axInf<Af>()),
          ticksPerSec_(1), printer_(printer) {
        scAssert(exec != nullptr);
        scAssert(printer != nullptr);
    }

    ///////////////////////////////////////////////////////////////////////////
    // G Commands
    ///////////////////////////////////////////////////////////////////////////

    void feedrateOverride(float) {}

    // Max velocity and acceleration should be set before this call.
    void linearMove(Af const &positionInUnits, float feed = inf()) {
        commands_.emplace_back(positionInUnits, maxVelocity(), maxAcceleration(), mode_);
    }

    // Same as linear move.
    void g0RapidMove(Af const &pos) { linearMove(pos, inf()); }

    // Same as linear move.
    void g1LinearMove(Af const &pos, float feed = inf()) { linearMove(pos, feed); }

    // Ticks per second should be set before this call.
    void g4Wait(float sec) { commands_.emplace_back(sec); }

    void g28RunHomingCycle() { commands_.emplace_back(homingVelocity()); }

    void g90g91DistanceMode(DistanceMode mode) { mode_ = mode; }

    ///////////////////////////////////////////////////////////////////////////
    // M commands
    ///////////////////////////////////////////////////////////////////////////

    // Overrides only finite axes
    void m100MaxVelocityOverride(Af const &unitsPerSec) {
        copyOnlyFinite(unitsPerSec, maxVelUnitsPerSec_);
        scAssert(all(gt(maxVelUnitsPerSec_, axZero<Af>())));
    }

    // Overrides only finite axes
    void m101MaxAccelerationOverride(Af const &unitsPerSecSqr) {
        copyOnlyFinite(unitsPerSecSqr, maxAccUnitsPerSec2_);
        scAssert(all(gt(maxAccUnitsPerSec2_, axZero<Af>())));
    }

    // Can be negative.
    // Overrides only finite axes
    void m102StepsPerUnitLengthOverride(Af const &stepsPerUnit) {
        copyOnlyFinite(stepsPerUnit, stepPerUnit_);
        scAssert(all(neq(stepPerUnit_, axZero<Af>())));
    }

    // Overrides only finite axes
    void m103HomingVelocityOverride(Af const &unitsPerSec) {
        copyOnlyFinite(unitsPerSec, homingVelUnitsPerSec_);
        scAssert(all(gt(homingVelUnitsPerSec_, axZero<Af>())));
    }

    void m104PrintInfo() const {
        *printer_ << "Max velocity: " << maxVelUnitsPerSec_ << " (" << maxVelocity() << ")" << eol
                  << "Max acceleration: " << maxAccUnitsPerSec2_ << " (" << maxAcceleration() << ")"
                  << eol << "Homing velocity: " << homingVelUnitsPerSec_ << " (" << homingVelocity()
                  << ")" << eol << "Steps per unit length: " << stepPerUnit_ << eol
                  << "Min position: " << minPosUnits_ << " (" << minPosition() << ")" << eol
                  << "Max position: " << maxPosUnits_ << " (" << maxPosition() << ")" << eol
                  << "Mode: " << (mode_ == DistanceMode::Absolute ? "Absolute" : "Relative") << eol
                  << "Ticks per second: " << ticksPerSec_ << eol << "Commands (" << commands_.size()
                  << "): ";
        for (auto const &cmd : commands_) {
            *printer_ << eol << "    " << cmd;
        }
        *printer_ << eol;
    }

    // Overrides all axes
    void m105MinPositionOverride(Af const &units) { minPosUnits_ = units; }

    // Overrides all axes
    void m106MaxPositionOverride(Af const &units) { maxPosUnits_ = units; }

    void m110PrintAxesConfiguration() { *printer_ << "Axes: " << AxesTraits::names() << eol; }

    ///////////////////////////////////////////////////////////////////////////
    // Others
    ///////////////////////////////////////////////////////////////////////////

    void error(const char *reason, ptrdiff_t pos, const char *str) const {
        *printer_ << "Error: " << reason << " at " << static_cast<int>(pos) << " in " << str << eol;
    }

    void start() {
        if (executor_->isRunning()) {
            *printer_ << "Error: already running" << eol;
            return;
        }
        loadSegmentsToExecutor();
        executor_->start();
    }

    void stop() { executor_->stop(); }

    bool isRunning() const { return executor_->isRunning(); }

    void printCurrentPosition() const {
        *printer_ << "Position: " << toUnits(executor_->position()) << eol;
    }

    void printCompleted() const {
        printCurrentPosition();
        *printer_ << "Completed\r\n";
    }

    void clearCommandsBuffer() {
        // Also frees memory.
        commands_ = move(std::vector<Cmd>());
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
                     clamp(-1.f, 1.f));
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
                     clamp(-1.f, 1.f));
    }

    // Steps
    Af minPosition() const { return minPosUnits_ * stepPerUnit_; }

    // Steps
    Af maxPosition() const { return maxPosUnits_ * stepPerUnit_; }

    int32_t ticksPerSecond() const { return ticksPerSec_; }

    std::vector<Cmd> const &commands() const { return commands_; }

    std::vector<Ai> path() const {
        std::vector<Ai> points;
        for (auto const &cmd : commands_) {
            if (cmd.type != Cmd::Move) {
                break;
            }
            addMoveCmdPoint(points, cmd, executor_->position(), mode());
        }
        return points;
    }

    Af toUnits(Ai const &pos) const { return axCast<float>(pos) / stepPerUnit_; }

    DistanceMode mode() const { return mode_; }

  private:
    void addMoveCmdPoint(std::vector<Ai> &points, Cmd const &cmd, Ai const &currPos,
                         DistanceMode mode) const {
        scAssert(cmd.type == Cmd::Move);

        if (points.empty()) {
            // Start path from current position.
            points.push_back(currPos);
        }

        // Update only finite values. Infinite values corresponds to unset axes.
        auto targetPosition = points.back();
        for (int i = 0; i < AxesTraits::size; ++i) {
            auto pos = cmd.move.pos[i];
            if (!std::isfinite(pos)) {
                continue;
            }

            auto maxPos = maxPosUnits_[i];
            if (std::isfinite(maxPos) && pos > maxPos) {
                pos = maxPos;
            }
            auto minPos = minPosUnits_[i];
            if (std::isfinite(minPos) && pos < minPos) {
                pos = minPos;
            }

            auto steps = lroundf(pos * stepPerUnit_[i]);
            if (mode == DistanceMode::Relative) {
                targetPosition[i] += steps;
            } else {
                targetPosition[i] = steps;
            }
        }

        // Add only position different from previous.
        if (targetPosition != points.back()) {
            points.push_back(targetPosition);
        }
    }

    void loadSegmentsToExecutor() {
        std::vector<Ai> points;
        std::vector<Sg> trajectory;

        auto currPos = executor_->position();
        auto acc = axZero<Af>();
        auto vel = axZero<Af>();

        auto appendPointsToTrajectory = [&] {
            if (points.size() < 2) {
                return;
            }
            auto lastPoint = points.back();

            PathToTrajectoryConverter<AxesTraits::size> trajGen(points);
            trajGen.setMaxVelocity(vel);
            trajGen.setMaxAcceleration(acc);
            trajGen.update();

            TrajectoryToSegmentsConverter<AxesTraits::size> segGen(points);
            segGen.setBlendDurations(move(trajGen.blendDurations()));
            segGen.setDurations(move(trajGen.durations()));
            segGen.appendTo(trajectory);

            points.clear();
            points.push_back(lastPoint);
        };

        for (auto const &cmd : commands_) {
            switch (cmd.type) {
            case Cmd::Move: {
                if (any(neq(vel, cmd.move.vel)) || any(neq(acc, cmd.move.acc))) {
                    appendPointsToTrajectory();

                    vel = cmd.move.vel;
                    acc = cmd.move.acc;
                }

                addMoveCmdPoint(points, cmd, currPos, cmd.move.mode);
            } break;
            case Cmd::Wait: {
                auto sec = cmd.wait.sec;
                if (sec < 0) {
                    break;
                }
                appendPointsToTrajectory();
                if (sec > 0) {
                    auto ticks = lround(sec * ticksPerSecond());
                    trajectory.push_back(Sg(ticks));
                }
            } break;
            case Cmd::Homing: {
                appendPointsToTrajectory();
                points.clear();
                trajectory.emplace_back(cmd.homing.vel);
                currPos.fill(0);
            } break;
            default:
                scAssert(!"Unexpected command");
            }
        }
        commands_.clear();

        appendPointsToTrajectory();
        executor_->setTrajectory(move(trajectory));
    }

    ISegmentsExecutor *executor_;
    std::vector<Cmd> commands_;
    DistanceMode mode_;
    Af homingVelUnitsPerSec_;
    Af maxVelUnitsPerSec_;
    Af maxAccUnitsPerSec2_;
    Af stepPerUnit_;
    Af minPosUnits_; // Can be inf
    Af maxPosUnits_; // Can be inf
    int32_t ticksPerSec_;
    Printer *printer_;
};
}
