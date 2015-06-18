#pragma once

#include "Segment.h"

namespace StepperControl {

enum class DistanceMode { Absolute, Relative };

template <size_t AxesSize>
class IGCodeInterpreter {
  public:
    using Af = Axes<float, AxesSize>;

    virtual ~IGCodeInterpreter() {}

    virtual void feedrateOverride(float feed) = 0;

    virtual void linearMove(Af const &pos, float feed) = 0;

    virtual void g0RapidMove(Af const &pos) = 0;

    virtual void g1LinearMove(Af const &pos, float feed) = 0;

    virtual void g4Wait(float sec) = 0;

    virtual void g28RunHomingCycle() = 0;

    virtual void g90g91DistanceMode(DistanceMode) = 0;

    virtual void m100MaxVelocityOverride(Af const &vel) = 0;

    virtual void m101MaxAccelerationOverride(Af const &acc) = 0;

    virtual void m102StepsPerUnitLengthOverride(Af const &spl) = 0;

    virtual void m103HomingVelocityOverride(Af const &vel) = 0;

    virtual void m104PrintInfo() const = 0;

    virtual void error(size_t pos, const char *line, const char *reason) = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual void printCurrentPosition() const = 0;

    virtual void clearCommandsBuffer() = 0;
};

template <size_t AxesSize>
struct ISegmentsExecutor {
    using Ai = Axes<int32_t, AxesSize>;
    using Sg = Segment<AxesSize>;
    using Segments = std::vector<Sg>;

    virtual ~ISegmentsExecutor() {}

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual bool isRunning() const = 0;

    virtual Ai const &position() const = 0;

    virtual void setPosition(Ai const &) = 0;

    virtual void setSegments(Segments const &) = 0;

    virtual void setSegments(Segments &&) = 0;

    virtual void setTicksPerSecond(int32_t) = 0;
};
}
