#pragma once

#include "GCodeParser.h"

namespace StepperControl {

/*
Intepreter reacts to callbacks from parser and creates path from them.
*/
template <size_t AxesSize>
class GCodeInterpreter : public GCodeParserCallbacks<AxesSize> {
  public:
    using AxesFloat = Axes<float, AxesSize>;
    using AxesInt = Axes<int32_t, AxesSize>;

    GCodeInterpreter() {
        maxAcceleration_.fill(1);
        maxVelocity_.fill(1);
        stepsPerUnitLength_.fill(1);
        ticksPerSecond_ = 1;
        currentPosition_.fill(0);
        mode_ = Absolute;
    }

    void feedrateOverride(float feed) override {}

    void linearMove(AxesFloat const &positionInUnits, float feed) override {
        ensureStartPosition();
        auto newPosition = positionInUnits * stepsPerUnitLength_;
        if (mode_ == Relative) {
            newPosition += axCast<float>(currentPosition_);
        }
        auto previousPosition = currentPosition_;
        tansformOnlyFinite(newPosition, &currentPosition_, &lroundf);
        if (previousPosition != currentPosition_) {
            path_.push_back(currentPosition_);
        }
    }

    void g0RapidMove(AxesFloat const &pos) override { linearMove(pos, inf()); }

    void g1LinearMove(AxesFloat const &pos, float feed) override { linearMove(pos, feed); }

    void ensureStartPosition() {
        if (path_.empty()) {
            path_.push_back(currentPosition_);
        }
    }

    void g4Wait(float sec) override {}

    void g28RunHomingCycle() override {}

    void g90g91DistanceMode(DistanceMode mode) override { mode_ = mode; }

    void m100MaxVelocityOverride(AxesFloat const &vel) override {
        float tpsInv = 1.f / ticksPerSecond_;
        copyOnlyFinite(vel * stepsPerUnitLength_ * tpsInv, &maxVelocity_);
    }

    void m101MaxAccelerationOverride(AxesFloat const &acc) override {
        float tpsInvSqr = 1.f / (ticksPerSecond_ * ticksPerSecond_);
        copyOnlyFinite(acc * stepsPerUnitLength_ * tpsInvSqr, &maxAcceleration_);
    }

    void m102StepsPerUnitLengthOverride(AxesFloat const &spl) override {
        copyOnlyFinite(spl, &stepsPerUnitLength_);
    }

    std::vector<AxesInt> const &path() const { return path_; }

    AxesInt const &currentPosition() const { return currentPosition_; }

    AxesFloat const &maxVelocity() const { return maxVelocity_; }

    AxesFloat const &maxAcceleration() const { return maxAcceleration_; }

    AxesFloat const &stepsPerUnitLength() const { return stepsPerUnitLength_; }

    int32_t ticksPerSecond() const { return ticksPerSecond_; }

    void setTicksPerSecond(int32_t tps) {
        assert(tps > 0);
        ticksPerSecond_ = tps;
    }

  private:
    DistanceMode mode_;
    std::vector<AxesInt> path_;
    AxesInt currentPosition_;
    AxesFloat maxVelocity_;
    AxesFloat maxAcceleration_;
    AxesFloat stepsPerUnitLength_;
    int32_t ticksPerSecond_;
};
}