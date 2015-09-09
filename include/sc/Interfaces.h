#pragma once

#include "Segment.h"

#include <vector>

namespace StepperControl {

struct DefaultAxesTraits {
    static const unsigned size = 9;
    static const char *names() { return "ABCUVWXYZ"; }
};

template <size_t size>
using TAf = Axes<float, size>;

template <size_t size>
using TAi = Axes<int32_t, size>;

template <size_t size>
using TSg = Segment<size>;

template <size_t size>
using TSgs = std::vector<TSg<size>>;

enum class DistanceMode { Absolute, Relative };

template <typename AxesTraits>
class IGCodeInterpreter {
  public:
    using Af = TAf<AxesTraits::size>;

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

    virtual void m105MaxDistanceOverride(Af const &vel) = 0;

    virtual void m106PrintAxesConfiguration() = 0;

    virtual void error(const char *reason) = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual bool isRunning() const = 0;

    virtual void printCurrentPosition() const = 0;

    virtual void clearCommandsBuffer() = 0;
};

template <typename AxesTraits>
struct ISegmentsExecutor {
    using Ai = TAi<AxesTraits::size>;
    using Sg = TSg<AxesTraits::size>;
    using Sgs = TSgs<AxesTraits::size>;

    virtual ~ISegmentsExecutor() {}

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual bool isRunning() const = 0;

    virtual Ai const &position() const = 0;

    virtual void setPosition(Ai const &) = 0;

    virtual void setSegments(Sgs const &) = 0;

    virtual void setSegments(Sgs &&) = 0;

    virtual void setTicksPerSecond(int32_t) = 0;
};
}
