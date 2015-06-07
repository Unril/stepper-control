#pragma once

#include "GCodeParser.h"

namespace StepperControl {

// Intepreter reacts to callbacks from parser and creates path from them.
template <size_t AxesSize>
class GCodeInterpreter : public GCodeParserCallbacks<AxesSize> {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;

    GCodeInterpreter() {
        maxVelocity_.fill(0.5f);
        maxAcceleration_.fill(0.1f);
        stepsPerUnitLength_.fill(1);
        ticksPerSecond_ = 1;
        currentPosition_.fill(0);
        mode_ = Absolute;
    }

    void feedrateOverride(float feed) override {}

    void linearMove(Af const &positionInUnits, float feed) override {
        ensureStartPosition();
        auto newPosition = positionInUnits * stepsPerUnitLength_;
        if (mode_ == Relative) {
            newPosition += axCast<float>(currentPosition_);
        }
        auto previousPosition = currentPosition_;
        tansformOnlyFinite(newPosition, currentPosition_, &lroundf);
        if (previousPosition != currentPosition_) {
            path_.push_back(currentPosition_);
        }
    }

    void g0RapidMove(Af const &pos) override { linearMove(pos, inf()); }

    void g1LinearMove(Af const &pos, float feed) override { linearMove(pos, feed); }

    void ensureStartPosition() {
        if (path_.empty()) {
            path_.push_back(currentPosition_);
        }
    }

    void g4Wait(float sec) override {}

    void g28RunHomingCycle() override {}

    void g90g91DistanceMode(DistanceMode mode) override { mode_ = mode; }

    void m100MaxVelocityOverride(Af const &vel) override {
        float tpsInv = 1.f / ticksPerSecond_;
        copyOnlyFinite(vel * stepsPerUnitLength_ * tpsInv, maxVelocity_);
        scAssert(all(gt(maxVelocity_, axZero<Af>())));
        applyInplace(maxVelocity_, [](float v) { return v > 0.5f ? 0.5f : v; });
    }

    void m101MaxAccelerationOverride(Af const &acc) override {
        float tpsInvSqr = 1.f / (ticksPerSecond_ * ticksPerSecond_);
        copyOnlyFinite(acc * stepsPerUnitLength_ * tpsInvSqr, maxAcceleration_);
        scAssert(all(gt(maxAcceleration_, axZero<Af>())));
    }

    void m102StepsPerUnitLengthOverride(Af const &spl) override {
        copyOnlyFinite(spl, stepsPerUnitLength_);
        scAssert(all(gt(stepsPerUnitLength_, axZero<Af>())));
    }

    void setTicksPerSecond(int32_t tps) {
        assert(tps > 0);
        ticksPerSecond_ = tps;
    }

    std::vector<Ai> const &path() const { return path_; }

    Ai const &currentPosition() const { return currentPosition_; }

    Af const &maxVelocity() const { return maxVelocity_; }

    Af const &maxAcceleration() const { return maxAcceleration_; }

    Af const &stepsPerUnitLength() const { return stepsPerUnitLength_; }

    int32_t ticksPerSecond() const { return ticksPerSecond_; }

  private:
    DistanceMode mode_;
    std::vector<Ai> path_;
    Ai currentPosition_;
    Af maxVelocity_;
    Af maxAcceleration_;
    Af stepsPerUnitLength_;
    int32_t ticksPerSecond_;
};
}