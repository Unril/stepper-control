#pragma once

#include "Axes.h"

#include <cstdint>

namespace StepperControl {
enum DistanceMode { Absolute, Relative };

template <size_t AxesSize>
class GCodeParserCallbacks {
  public:
    virtual ~GCodeParserCallbacks() {}

    virtual void feedrateOverride(float feed) {}

    virtual void linearMove(Axes<float, AxesSize> const &pos, float feed) {}

    virtual void g0RapidMove(Axes<float, AxesSize> const &pos) {}

    virtual void g1LinearMove(Axes<float, AxesSize> const &pos, float feed) {}

    virtual void g4Wait(float sec) {}

    virtual void g28RunHomingCycle() {}

    virtual void g90g91DistanceMode(DistanceMode) {}

    virtual void m100MaxVelocityOverride(Axes<float, AxesSize> const &vel) {}

    virtual void m101MaxAccelerationOverride(Axes<float, AxesSize> const &acc) {}

    virtual void m102StepsPerUnitLengthOverride(Axes<float, AxesSize> const &spl) {}
};

/*
EBNF grammar

digit = "0" .. "9"
integer = digit { digit }
floating =  [ "-" | "+" ] integer ["." integer]
axisName = "A" | "B" | "C" | "U" | "V" | "W" | "X" | "Y" | "Z"
axisFloat = axisName floating
axesFloat = axisFloat {axisFloat}
feedrate = "F" floating
axesWithFeedrate = axesFloat [feedrate]

g0RapidMove = [axesFloat] "\n"
g1LinearMove = [axesWithFeedrate] "\n"
g4Wait = "P" floatind "\n"
g28RunHomingCycle = "\n"
g90AbsoluteDistanceMode = "\n"
g91RelativeDistanceMode = "\n"
m100MaxVelocityOverride = [axesFloat] "\n"
m101MaxAccelerationOverride = [axesFloat] "\n"
m102StepsPerUnitLengthOverride = [axesFloat] "\n"

linearMove = axesWithFeedrate "\n"
feedrateOverride = feedrate "\n"
gCommand = "G" integer ( g0RapidMove | g1LinearMove | g4Wait | g28RunHomingCycle |
                         g90AbsoluteDistanceMode | g91RelativeDistanceMode )
mCommand = "M" integer ( m100MaxVelocityOverride | m101MaxAccelerationOverride)

line = ( linearMove | feedrateOverride | gCommand | mCommand | "\n" )
*/
template <size_t AxesSize>
class GCodeParser {
    using AxesFloat = Axes<float, AxesSize>;

  public:
    explicit GCodeParser(GCodeParserCallbacks<AxesSize> *cb)
        : errorPos_(std::numeric_limits<size_t>::max()), pos_(nullptr), str_(nullptr), cb_(cb) {
        assert(cb);
    }

    bool parseLine(const char *str) {
        assert(str);
        str_ = str;
        pos_ = str;
        errorPos_ = std::numeric_limits<size_t>::max();
        skipSpaces();
        line();
        return errorPos_ == std::numeric_limits<size_t>::max();
    }

    size_t errorPosition() const { return errorPos_; }

  private:
    inline static const char *axesNames() {
        static_assert(AxesSize <= 9, "Where are at most 9 axes names.");
        return "ABCUVWXYZ";
    }

    static void updateAxisValue(AxesFloat *axes, char name, float value) {
        name = toupper(name);
        auto axisName = axesNames();
        for (auto axis = axes->begin(); axis != axes->end(); ++axis, ++axisName) {
            if (*axisName == name) {
                *axis = value;
            }
        }
    }

    bool integer(int32_t *value) {
        if (!isDigit()) {
            return false;
        }
        *value = strtoul(pos_, const_cast<char **>(&pos_), 10);
        skipSpaces();
        return true;
    }

    bool signedInteger(int32_t *value) {
        char signChar = 0;
        if (isSign()) {
            signChar = sym();
            nextsym();
        }
        if (!integer(value)) {
            return false;
        }
        if (signChar == '-') {
            *value = -*value;
        }
        return true;
    }

    bool floating(float *value) {
        if (!isDigit() && !isSign() && sym() != '.') {
            return false;
        }
        *value = strtof(pos_, const_cast<char **>(&pos_));
        skipSpaces();
        return true;
    }

    bool axisName(char *name) {
        if (!isAxis()) {
            return false;
        }
        *name = sym();
        nextsym();
        return true;
    }

    bool axisFloat(char *name, float *value) {
        if (!axisName(name)) {
            return false;
        }
        if (!floating(value)) {
            error();
            return false;
        }
        return true;
    }

    bool axesFloat(AxesFloat *axes) {
        char name = 0;
        float value = inf();
        if (!axisFloat(&name, &value)) {
            return false;
        }
        do {
            updateAxisValue(axes, name, value);
        } while (axisFloat(&name, &value));
        return true;
    }

    bool feedrate(float *feed) {
        if (!isFeedrate()) {
            return false;
        }
        nextsym();
        if (!floating(feed)) {
            error();
            return false;
        }
        return true;
    }

    bool axesWithFeedrate(AxesFloat *steps, float *feed) {
        if (!axesFloat(steps)) {
            return false;
        }
        feedrate(feed);
        return true;
    }

    bool feedrateOverride() {
        float feed = inf();
        if (!feedrate(&feed)) {
            return false;
        }
        if (!expectNewLine()) {
            return false;
        }
        cb_->feedrateOverride(feed);
        return true;
    }

    bool linearMove() {
        AxesFloat steps;
        steps.fill(inf());
        float feed = inf();
        if (!axesWithFeedrate(&steps, &feed)) {
            return false;
        }
        if (!expectNewLine()) {
            return false;
        }
        cb_->linearMove(steps, feed);
        return true;
    }

    bool g0RapidMove() {
        AxesFloat steps;
        steps.fill(inf());
        axesFloat(&steps);
        if (!expectNewLine()) {
            return false;
        }
        cb_->g0RapidMove(steps);
        return true;
    }

    bool g1LinearMove() {
        AxesFloat steps;
        steps.fill(inf());
        float feed = inf();
        axesWithFeedrate(&steps, &feed);
        if (!expectNewLine()) {
            return false;
        }
        cb_->g1LinearMove(steps, feed);
        return true;
    }

    bool g4Wait() {
        if (!isPause()) {
            error();
            return false;
        }
        nextsym();
        float sec = 0;
        if (!floating(&sec)) {
            error();
            return false;
        }
        if (!expectNewLine()) {
            return false;
        }
        cb_->g4Wait(sec);
        return true;
    }

    bool g28RunHomingCycle() {
        if (!expectNewLine()) {
            return false;
        }
        cb_->g28RunHomingCycle();
        return true;
    }

    bool g90AbsoluteDistanceMode() {
        if (!expectNewLine()) {
            return false;
        }
        cb_->g90g91DistanceMode(Absolute);
        return true;
    }

    bool g91RelativeDistanceMode() {
        if (!expectNewLine()) {
            return false;
        }
        cb_->g90g91DistanceMode(Relative);
        return true;
    }

    bool gCommand() {
        if (!isGCommand()) {
            return false;
        }
        nextsym();
        int32_t command = 0;
        if (!integer(&command)) {
            error();
            return false;
        }
        switch (command) {
        case 0:
            return g0RapidMove();
        case 1:
            return g1LinearMove();
        case 4:
            return g4Wait();
        case 28:
            return g28RunHomingCycle();
        case 90:
            return g90AbsoluteDistanceMode();
        case 91:
            return g91RelativeDistanceMode();
        default:
            error();
            return false;
        }
    }

    bool m100MaxVelocityOverride() {
        AxesFloat vel;
        vel.fill(inf());
        axesFloat(&vel);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m100MaxVelocityOverride(vel);
        return true;
    }

    bool m101MaxAccelerationOverride() {
        AxesFloat acc;
        acc.fill(inf());
        axesFloat(&acc);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m101MaxAccelerationOverride(acc);
        return true;
    }

    bool m102StepsPerUnitLengthOverride() {
        AxesFloat spu;
        spu.fill(inf());
        axesFloat(&spu);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m102StepsPerUnitLengthOverride(spu);
        return true;
    }

    bool mCommand() {
        if (!isMCommand()) {
            return false;
        }
        nextsym();
        int32_t command = 0;
        if (!integer(&command)) {
            error();
            return false;
        }
        switch (command) {
        case 100:
            return m100MaxVelocityOverride();
        case 101:
            return m101MaxAccelerationOverride();
        case 102:
            return m102StepsPerUnitLengthOverride();
        default:
            error();
            return false;
        }
    }

    bool expectNewLine() {
        if (!isNewLine()) {
            error();
            return false;
        }
        nextsym();
        return true;
    }

    bool line() {
        if (linearMove()) {
            return true;
        }
        if (gCommand()) {
            return true;
        }
        if (mCommand()) {
            return true;
        }
        if (feedrateOverride()) {
            return true;
        }
        return expectNewLine();
    }

    inline bool isEnd() const { return sym() == '\0'; }
    inline bool isGCommand() const { return toupper(sym()) == 'G'; }
    inline bool isMCommand() const { return toupper(sym()) == 'M'; }
    inline bool isFeedrate() const { return toupper(sym()) == 'F'; }
    inline bool isPause() const { return toupper(sym()) == 'P'; }
    inline bool isAxis() const { return sym() && strchr(axesNames(), toupper(sym())); }
    inline bool isDigit() const { return isdigit(sym()) != 0; }
    inline bool isSign() const { return sym() == '-' || sym() == '+'; }
    inline bool isNewLine() const { return sym() == '\n'; }
    inline bool isSpace() const { return sym() == ' ' || sym() == '\t' || sym() == '\r'; }

    inline void skipSpaces() {
        while (isSpace()) {
            ++pos_;
        }
    }

    inline void nextsym() {
        if (sym() == 0) {
            error();
        }
        ++pos_;
        skipSpaces();
    }

    inline char sym() const { return *pos_; }

    inline void error() { errorPos_ = pos_ - str_; }

    size_t errorPos_;
    const char *pos_;
    const char *str_;
    GCodeParserCallbacks<AxesSize> *cb_;
};
}
