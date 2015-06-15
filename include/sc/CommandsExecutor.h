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
        notifyInterval_ = ticksPerSecond / 5;
        nextNotifyTick_ = notifyInterval_;
        nextCmd_ = cmds_.end();
        interpreter_.setTicksPerSecond(ticksPerSecond);
        executor_.setTicksPerSecond(ticksPerSecond);
    }

    GCodeInterpreter<AxesSize> &interpreter() { return interpreter_; }

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
        auto ticks = executor_.ticks();
        if (ticks > nextNotifyTick_) {
            nextNotifyTick_ = ticks + notifyInterval_;
            axPrintf(currentPositionInUnits());
            printf("\n");
        }

        if (wasRunning_ && !executor_.running()) {
            // Just stopped.
            // Try load next portion of segments to executor.
            if (nextCmd_ != cmds_.end()) {
                runNextCommand();
            } else {
                nextNotifyTick_ = notifyInterval_;

                // No more commands.
                cmds_.clear();
                nextCmd_ = cmds_.end();
                syncPosition();

                // Report completed.
                axPrintf(currentPositionInUnits());
                printf("\nCompleted\n");
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
            PathToTrajectoryConverter<AxesSize> pathToTrajectory;
            TrajectoryToSegmentsConverter<AxesSize> trajectoryToSegments;

            pathToTrajectory.setMaxVelocity(nextCmd_->maxVelocity());
            pathToTrajectory.setMaxAcceleration(nextCmd_->maxAcceleration());
            pathToTrajectory.setPath(move(nextCmd_->path()));
            pathToTrajectory.update();

            trajectoryToSegments.setPath(move(pathToTrajectory.path()));
            trajectoryToSegments.setBlendDurations(move(pathToTrajectory.blendDurations()));
            trajectoryToSegments.setDurations(move(pathToTrajectory.durations()));
            trajectoryToSegments.update();

            executor_.setSegments(move(trajectoryToSegments.segments()));
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

    void printInfo() const {
        executor_.printInfo();
        interpreter_.printInfo();
    }

    void parse(char const *line) { parser_.parseLine(line); }

    bool wasRunning_;
    int32_t notifyInterval_;
    int32_t nextNotifyTick_;
    typename std::vector<Cmd>::iterator nextCmd_;
    std::vector<Cmd> cmds_;

    GCodeInterpreter<AxesSize> interpreter_;
    GCodeParser<AxesSize> parser_;
    SegmentsExecutor<AxesSize, TMotor, TTicker> executor_;
};
}
