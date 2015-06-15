#pragma once

#include "Axes.h"

#include <cstdint>

namespace StepperControl {
enum DistanceMode { Absolute, Relative };

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

    virtual void error(size_t pos, const char *line, const char *reason) = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

    virtual void printInfo() const = 0;

    virtual void clearCommandsBuffer() = 0;
};

/*
Parser accepts input line and calls corresponding callbacks.

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
m103HomingVelocityOverride = [axesFloat] "\n"

linearMove = axesWithFeedrate "\n"
feedrateOverride = feedrate "\n"
gCommand = "G" integer ( g0RapidMove | g1LinearMove | g4Wait | g28RunHomingCycle |
    g90AbsoluteDistanceMode | g91RelativeDistanceMode )
mCommand = "M" integer ( m100MaxVelocityOverride | m101MaxAccelerationOverride |
    m102StepsPerUnitLengthOverride | m103HomingVelocityOverride)
start = "~" "\n"
stop = "!" "\n"
clearCommandsBuffer = "^" "\n"
printInfo = "?" "\n"
line = ( start | stop | | clearCommandsBuffer | printInfo |
    linearMove | feedrateOverride | gCommand | mCommand | "\n" )
*/
template <size_t AxesSize>
class GCodeParser {
    using Af = Axes<float, AxesSize>;

  public:
    explicit GCodeParser(IGCodeInterpreter<AxesSize> *cb)
        : errorPos_(std::numeric_limits<size_t>::max()), pos_(nullptr), str_(nullptr), cb_(cb) {
        scAssert(cb);
    }

    bool parseLine(const char *str) {
        scAssert(str);
        str_ = str;
        pos_ = str;
        errorPos_ = std::numeric_limits<size_t>::max();
        skipSpaces();
        return line() && errorPos_ == std::numeric_limits<size_t>::max();
    }

    size_t errorPosition() const { return errorPos_; }

  private:
    inline static const char *axesNames() {
        static_assert(AxesSize <= 9, "Where are at most 9 axes names.");
        return "ABCUVWXYZ";
    }

    static void updateAxisValue(Af *axes, char name, float value) {
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
            error("expect floating value");
            return false;
        }
        return true;
    }

    bool axesFloat(Af *axes) {
        char name = 0;
        auto value = inf();
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
            error("expect floating feed");
            return false;
        }
        return true;
    }

    bool axesWithFeedrate(Af *steps, float *feed) {
        if (!axesFloat(steps)) {
            return false;
        }
        feedrate(feed);
        return true;
    }

    bool feedrateOverride() {
        auto feed = inf();
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
        auto steps = axInf<Af>();
        auto feed = inf();
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
        auto steps = axInf<Af>();
        axesFloat(&steps);
        if (!expectNewLine()) {
            return false;
        }
        cb_->g0RapidMove(steps);
        return true;
    }

    bool g1LinearMove() {
        auto steps = axInf<Af>();
        auto feed = inf();
        axesWithFeedrate(&steps, &feed);
        if (!expectNewLine()) {
            return false;
        }
        cb_->g1LinearMove(steps, feed);
        return true;
    }

    bool g4Wait() {
        if (!isPause()) {
            error("expect P");
            return false;
        }
        nextsym();
        float sec = 0;
        if (!floating(&sec)) {
            error("expect floating seconds");
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
            error("expect integer command");
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
            error("unknown G command");
            return false;
        }
    }

    bool m100MaxVelocityOverride() {
        auto vel = axInf<Af>();
        axesFloat(&vel);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m100MaxVelocityOverride(vel);
        return true;
    }

    bool m101MaxAccelerationOverride() {
        auto acc = axInf<Af>();
        axesFloat(&acc);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m101MaxAccelerationOverride(acc);
        return true;
    }

    bool m102StepsPerUnitLengthOverride() {
        auto spu = axInf<Af>();
        axesFloat(&spu);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m102StepsPerUnitLengthOverride(spu);
        return true;
    }

    bool m103HomingVelocityOverride() {
        auto vel = axInf<Af>();
        axesFloat(&vel);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m103HomingVelocityOverride(vel);
        return true;
    }

    bool mCommand() {
        if (!isMCommand()) {
            return false;
        }
        nextsym();
        int32_t command = 0;
        if (!integer(&command)) {
            error("expect integer command");
            return false;
        }
        switch (command) {
        case 100:
            return m100MaxVelocityOverride();
        case 101:
            return m101MaxAccelerationOverride();
        case 102:
            return m102StepsPerUnitLengthOverride();
        case 103:
            return m103HomingVelocityOverride();
        default:
            error("unknown M command");
            return false;
        }
    }

    bool start() {
        if (!isStart()) {
            return false;
        }
        nextsym();
        if (!expectNewLine()) {
            return false;
        }
        cb_->start();
        return true;
    }

    bool stop() {
        if (!isStop()) {
            return false;
        }
        nextsym();
        if (!expectNewLine()) {
            return false;
        }
        cb_->stop();
        return true;
    }

    bool clearCommandsBuffer() {
        if (!isClear()) {
            return false;
        }
        nextsym();
        if (!expectNewLine()) {
            return false;
        }
        cb_->clearCommandsBuffer();
        return true;
    }

    bool printInfo() {
        if (!isInfo()) {
            return false;
        }
        nextsym();
        if (!expectNewLine()) {
            return false;
        }
        cb_->printInfo();
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
        if (start()) {
            return true;
        }
        if (stop()) {
            return true;
        }
        if (clearCommandsBuffer()) {
            return true;
        }
        if (printInfo()) {
            return true;
        }
        return expectNewLine();
    }

    inline bool expectNewLine() {
        if (!isNewLine()) {
            error("expect new line");
            return false;
        }
        nextsym();
        return true;
    }

    inline bool isGCommand() const { return toupper(sym()) == 'G'; }

    inline bool isMCommand() const { return toupper(sym()) == 'M'; }

    inline bool isFeedrate() const { return toupper(sym()) == 'F'; }

    inline bool isPause() const { return toupper(sym()) == 'P'; }

    inline bool isAxis() const { return sym() && strchr(axesNames(), toupper(sym())); }

    inline bool isDigit() const { return isdigit(sym()) != 0; }

    inline bool isSign() const { return sym() == '-' || sym() == '+'; }

    inline bool isNewLine() const { return sym() == '\n'; }

    inline bool isSpace() const { return sym() == ' ' || sym() == '\t' || sym() == '\r'; }

    inline bool isStart() const { return sym() == '~'; }

    inline bool isStop() const { return sym() == '!'; }

    inline bool isClear() const { return sym() == '^'; }

    inline bool isInfo() const { return sym() == '?'; }

    inline void skipSpaces() {
        while (isSpace()) {
            ++pos_;
        }
    }

    inline void nextsym() {
        if (sym() == 0) {
            error("expect symbol");
        }
        ++pos_;
        skipSpaces();
    }

    inline char sym() const { return *pos_; }

    inline void error(const char *reason) {
        errorPos_ = pos_ - str_;
        cb_->error(errorPos_, str_, reason);
    }

    size_t errorPos_;
    const char *pos_;
    const char *str_;
    IGCodeInterpreter<AxesSize> *cb_;
};
}
