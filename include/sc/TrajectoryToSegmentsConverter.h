#pragma once

#include "Segment.h"

#include <algorithm>

namespace StepperControl {
// It creates sequence of linear and parabolic segments from given path points,
// durations between points and durations of blend segments.
template <size_t AxesSize>
class TrajectoryToSegmentsConverter {
  public:
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;
    using Segments = std::vector<Segment<AxesSize>>;

    void setPath(std::vector<Ai> const &path) { path_ = path; }

    void setPath(std::vector<Ai> &&path) { path_ = move(path); }

    void setDurations(std::vector<float> const &durations) { Dts_ = durations; }

    void setDurations(std::vector<float> &&durations) { Dts_ = move(durations); }

    void setBlendDurations(std::vector<float> const &blendDurations) { tbs_ = blendDurations; }

    void setBlendDurations(std::vector<float> &&blendDurations) { tbs_ = move(blendDurations); }

    void update() {
        scAssert(!path_.empty());
        scAssert(path_.size() - 1 == Dts_.size());
        scAssert(path_.size() == tbs_.size());

        std::transform(Dts_.begin(), Dts_.end(), Dts_.begin(), &ceilf);
        std::transform(tbs_.begin(), tbs_.end(), tbs_.begin(), &ceilf);

        segments_.clear();
        for (size_t i = 0; i < path_.size(); i++) {
            addSegmentsForPoint(i);
        }
    }

    Segments const &segments() const { return segments_; }

    Segments &segments() { return segments_; }

  private:
    void addSegmentsForPoint(size_t i) {
        auto firstPoint = i == 0;
        auto lastPoint = (i == path_.size() - 1);

        auto x = path_[i];
        auto tBlend = tbs_[i];

        // Add only nonzero blend segment.
        if (tBlend > 0) {
            auto v = axZero<Af>();     // First tangent slope.
            auto vNext = axZero<Af>(); // Second tangent slope.

            // Treat first and last points differently.
            if (firstPoint) {
                // First tangent of first blend has zero slope.
                vNext = axCast<float>(path_[i + 1] - x) / Dts_[i];
            } else if (lastPoint) {
                // Second tangent of last blend has zero slope.
                v = axCast<float>(x - path_[i - 1]) / Dts_[i - 1];
            } else {
                v = axCast<float>(x - path_[i - 1]) / Dts_[i - 1];
                vNext = axCast<float>(path_[i + 1] - x) / Dts_[i];
            }

            scAssert(all(le(axAbs(v), axConst<Af>(0.5f))));
            scAssert(all(le(axAbs(vNext), axConst<Af>(0.5f))));

            auto Dx = axLRound(0.5f * tBlend * v);
            auto DxNext = axLRound(0.5f * tBlend * vNext);

            // Check rounded slope <= 0.5 and correct blend duration if neccesary.
            auto tBlendCorrected = tBlend;
            for (size_t j = 0; j < AxesSize; ++j) {
                auto DxAbsX4 = abs(Dx[j] * 4.f);
                if (tBlendCorrected < DxAbsX4) {
                    tBlendCorrected = DxAbsX4;
                }
                auto DxNextAbsX4 = abs(DxNext[j] * 4.f);
                if (tBlendCorrected < DxNextAbsX4) {
                    tBlendCorrected = DxNextAbsX4;
                }
            }

            segments_.emplace_back(static_cast<int32_t>(tBlendCorrected), Dx, DxNext);
        }

        // Where is no linear segments after last point.
        if (lastPoint)
            return;

        auto tBlendNext = tbs_[i + 1];
        auto const &xNext = path_[i + 1];
        auto Dt = Dts_[i];
        auto Dx = xNext - x;

        // Segment slope.
        auto v = axCast<float>(Dx) / Dt;

        // Calculate blend x difference the same way as above to be consistent in rounding.
        auto DxBlend = axLRound(0.5f * tBlend * v);
        auto DxBlendNext = axLRound(0.5f * tBlendNext * v);

        auto tBlendPart = (tBlend + tBlendNext) * 0.5f;
        auto tLine = Dt - tBlendPart;
        auto DxLine = Dx - (DxBlend + DxBlendNext);

        auto tLineTrunc = lTruncTowardInf(tLine);
        if (tLineTrunc > 0) {
            // Check rounded slope <= 0.5 and correct line duration if neccesary.
            for (size_t j = 0; j < AxesSize; ++j) {
                auto DxAbsX2 = abs(DxLine[j] * 2);
                if (tLineTrunc < DxAbsX2) {
                    tLineTrunc = DxAbsX2;
                }
            }
            segments_.emplace_back(tLineTrunc, DxLine);
        }
    }

    std::vector<Ai> path_;
    std::vector<float> Dts_;
    std::vector<float> tbs_;
    Segments segments_;
};
}
