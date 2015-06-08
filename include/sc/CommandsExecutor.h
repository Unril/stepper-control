#pragma once

#include "GCodeParser.h"
#include "GCodeInterpreter.h"
#include "PathToTrajectoryConverter.h"
#include "TrajectoryToSegmentsConverter.h"
#include "SegmentsExecutor.h"

namespace StepperControl {

template <size_t AxeSize, typename TMotor, typename TTicker>
class CommandsExecutor {
  public:
    CommandsExecutor(int32_t ticksPerSecond, TMotor *motor, TTicker *ticker)
        : parser_(&interpreter_), executor_(motor, ticker) {

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

    void start() {
        executor_.start();
    }

    void stop() {
        executor_.stop();
    }

    void reset() {
        executor_.stop();
    }

    void parse(char const *line) {
        parser_.parseLine(line);
    }

  private:
    GCodeInterpreter<AxeSize> interpreter_;
    GCodeParser<AxeSize> parser_;
    PathToTrajectoryConverter<AxeSize> pathToTrajectoryConverter_;
    TrajectoryToSegmentsConverter<AxeSize> trajectoryToSegmentsConverter_;
    SegmentsExecutor<AxeSize, TMotor, TTicker> executor_;
};
}