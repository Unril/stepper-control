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
        positionNotifyInterval_ = ticksPerSecond / 5;
        positionNotifyTick_ = positionNotifyInterval_;
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
        } else if (*line == '?') {
            printInfo();
        } else {
            parse(line);
        }
    }

    // Should be called in a loop.
    void update() {
        if (executor_.ticks() > positionNotifyTick_) {
            positionNotifyTick_ += positionNotifyInterval_;
            axPrintln(currentPositionInUnits());
        }

        if (wasRunning_ && !executor_.running()) {
            // Just stopped.
            // Try load next portion of segments to executor.
            if (nextCmd_ != cmds_.end()) {
                runNextCommand();
            } else {
                positionNotifyTick_ = positionNotifyInterval_;

                // No more commands.
                cmds_.clear();
                nextCmd_ = cmds_.end();
                syncPosition();

                // Report completed.
                axPrintln(currentPositionInUnits());
                printf("Completed\n");
            }
        }
        wasRunning_ = executor_.running();
    }

  private:
    Af currentPositionInUnits() const {
        return axCast<float>(executor_.position()) / interpreter_.stepsPerUnitLength();
    }

    void syncPosition() { interpreter_.setCurrentPosition(executor_.position()); }

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
        if (interpreter_.hasCommands()) {
            // Move commands from interpreter to local commands list.
            cmds_ = move(interpreter_.commands());
            interpreter_.clearCommands();
            nextCmd_ = cmds_.begin();

            runNextCommand();
        }
    }

    void stop() {
        executor_.stop();
        cmds_.clear();
        nextCmd_ = cmds_.end();
        syncPosition();
    }

    void reset() {
        stop();
        interpreter_.clearCommands();
    }

    void printInfo() {}

    void parse(char const *line) { parser_.parseLine(line); }

    bool wasRunning_;
    int32_t positionNotifyInterval_;
    int32_t positionNotifyTick_;
    typename std::vector<Cmd>::iterator nextCmd_;
    std::vector<Cmd> cmds_;
    GCodeInterpreter<AxesSize> interpreter_;
    GCodeParser<AxesSize> parser_;
    PathToTrajectoryConverter<AxesSize> pathToTrajectory_;
    TrajectoryToSegmentsConverter<AxesSize> trajectoryToSegments_;
    SegmentsExecutor<AxesSize, TMotor, TTicker> executor_;
};
}
