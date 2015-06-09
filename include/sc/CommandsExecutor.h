#pragma once

#include "GCodeParser.h"
#include "GCodeInterpreter.h"
#include "PathToTrajectoryConverter.h"
#include "TrajectoryToSegmentsConverter.h"
#include "SegmentsExecutor.h"

namespace StepperControl {
using std::move;

template <size_t AxesSize, typename TMotor, typename TTicker>
class CommandsExecutor {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;
    using Cmd = Command<AxesSize>;
    using Seg = Segment<AxesSize>;

    CommandsExecutor(int32_t ticksPerSecond, TMotor *motor, TTicker *ticker)
        : parser_(&interpreter_), executor_(motor, ticker) {
        wasRunning_ = false;
        nextCmd_ = cmds_.end();
        interpreter_.setTicksPerSecond(ticksPerSecond);
        executor_.setTicksPerSecond(ticksPerSecond);
    }

    void executeLine(char const *line) {
        scAssert(line);

        if (*line == '~') {
            start();
        } else if (*line == '!') {
            stop();
        } else if (*line == '^') {
            reset();
        } else {
            parse(line);
        }
    }

    Af currentPositionInUnits() const {
        return axCast<float>(executor_.position()) / interpreter_.stepsPerUnitLength();
    }

    void update() {
        auto interval = executor_.ticksPerSecond() / 5;
        if (executor_.ticks() % interval == 0) {
            std::cout << currentPositionInUnits() << std::endl;
        }

        if (wasRunning_ && !executor_.running()) {
            // Just stopped.
            // Try load next portion of segments to executor.
            if (nextCmd_ != cmds_.end()) {
                runNextCommand();
            } else {
                // No more commands.
                // Sync end.
                interpreter_.setCurrentPosition(executor_.position());

                std::cout << currentPositionInUnits() << std::endl << "Completed" << std::endl;
            }
        }
        wasRunning_ = executor_.running();
    }

    void runNextCommand() {
        switch (nextCmd_->type()) {
        case Cmd::Move: {
            pathToTrajectory_.setMaxVelocity(nextCmd_->maxVelocity());
            pathToTrajectory_.setMaxAcceleration(nextCmd_->maxAcceleration());
            pathToTrajectory_.setPath(move(nextCmd_->path()));
            pathToTrajectory_.update();

            trajectoryToSegments_.setPath(move(pathToTrajectory_.path()));
            trajectoryToSegments_.setBlendDurations(move(pathToTrajectory_.blendDurations()));
            trajectoryToSegments_.setDurations(move(pathToTrajectory_.durations()));
            trajectoryToSegments_.update();

            executor_.setSegments(move(trajectoryToSegments_.segments()));
            executor_.start();
            break;
        }
        case Cmd::Homing: {
            auto dt = std::numeric_limits<int32_t>::max();
            auto vInv = 1.f / nextCmd_->homingVelocity();
            auto dx = -dt / axCast<int32_t>(vInv);

            executor_.setSegments({Seg(dt, dx)});
            executor_.startHoming();
            break;
        }
        case Cmd::Wait: {
            executor_.setSegments({Seg(nextCmd_->waitDuration(), axZero<Ai>())});
            executor_.start();
            break;
        }
        default:
            scAssert(!"Unexpected command type.");
            break;
        }

        ++nextCmd_;
    }

    void start() {
        // Move commands from interpreter to local commands list.
        if (interpreter_.hasCommands()) {
            cmds_ = move(interpreter_.commands());
            nextCmd_ = cmds_.begin();
            interpreter_.clearCommands();

            runNextCommand();
        }
    }

    void stop() {
        executor_.stop();
        cmds_.clear();
        nextCmd_ = cmds_.end();
    }

    void reset() {
        stop();
        interpreter_.clear();
    }

    void parse(char const *line) { parser_.parseLine(line); }

  private:
    bool wasRunning_;
    typename std::vector<Cmd>::iterator nextCmd_;
    std::vector<Cmd> cmds_;
    GCodeInterpreter<AxesSize> interpreter_;
    GCodeParser<AxesSize> parser_;
    PathToTrajectoryConverter<AxesSize> pathToTrajectory_;
    TrajectoryToSegmentsConverter<AxesSize> trajectoryToSegments_;
    SegmentsExecutor<AxesSize, TMotor, TTicker> executor_;
};
}
