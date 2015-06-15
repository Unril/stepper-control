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

        if (wasRunning_ && !executor_.isRunning()) {
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
        wasRunning_ = executor_.isRunning();
    }

  private:
    Af currentPositionInUnits() const {
        return axCast<float>(executor_.position()) / interpreter_.stepsPerUnitLength();
    }

    void syncPosition() { interpreter_.setCurrentPosition(executor_.position()); }

    void moveCommandToSegments() {
        PathToTrajectoryConverter<AxesSize> trajGen;
        trajGen.setMaxVelocity(nextCmd_->maxVelocity());
        trajGen.setMaxAcceleration(nextCmd_->maxAcceleration());
        trajGen.setPath(move(nextCmd_->path()));
        trajGen.update();

        TrajectoryToSegmentsConverter<AxesSize> segGen;
        segGen.setPath(move(trajGen.path()));
        segGen.setBlendDurations(move(trajGen.blendDurations()));
        segGen.setDurations(move(trajGen.durations()));
        segGen.update();

        executor_.setSegments( move(segGen.segments()));
    }

    void runNextCommand() {
        switch (nextCmd_->type()) {
        case Cmd::Move: {
            moveCommandToSegments();
            break;
        }
        case Cmd::Homing: {
            executor_.setSegments({Seg(nextCmd_->homingVelocity())});
            break;
        }
        case Cmd::Wait: {
            executor_.setSegments({Seg(nextCmd_->waitDuration())});
            break;
        }
        default:
            scAssert(!"Unexpected command type.");
            return;
        }

        executor_.start();
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
