#pragma once

#include "Axes.h"

namespace StepperControl {

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
g4Wait = "P" floating "\n"
g28RunHomingCycle = "\n"
g90AbsoluteDistanceMode = "\n"
g91RelativeDistanceMode = "\n"
m100MaxVelocityOverride = [axesFloat] "\n"
m101MaxAccelerationOverride = [axesFloat] "\n"
m102StepsPerUnitLengthOverride = [axesFloat] "\n"
m103HomingVelocityOverride = [axesFloat] "\n"
m104PrintInfo = "\n"
m105MinPositionOverride = [axesFloat] "\n"
m106MaxPositionOverride = [axesFloat] "\n"
m110PrintAxesConfiguration = "\n"

linearMove = axesWithFeedrate "\n"
feedrateOverride = feedrate "\n"
gCommand = "G" integer ( g0RapidMove | g1LinearMove | g4Wait | g28RunHomingCycle |
    g90AbsoluteDistanceMode | g91RelativeDistanceMode )
mCommand = "M" integer ( m100MaxVelocityOverride | m101MaxAccelerationOverride |
    m102StepsPerUnitLengthOverride | m103HomingVelocityOverride | m104PrintInfo |
    m105MinPositionOverride | m106MaxPositionOverride | m110PrintAxesConfiguration)
start = "~" "\n"
stop = "!" "\n"
clearCommandsBuffer = "^" "\n"
printCurrentPosition = "?" "\n"
line = ( start | stop | | clearCommandsBuffer | printCurrentPosition |
    linearMove | feedrateOverride | gCommand | mCommand | "\n" )


struct IGCodeInterpreter {]
  void feedrateOverride(float feed) {}
  void linearMove(Af const &pos, float feed) {}
  void g0RapidMove(Af const &pos) {}
  void g1LinearMove(Af const &pos, float feed) {}
  void g4Wait(float sec) {}
  void g28RunHomingCycle() {}
  void g90g91DistanceMode(DistanceMode) {}
  void m100MaxVelocityOverride(Af const &vel) {}
  void m101MaxAccelerationOverride(Af const &acc) {}
  void m102StepsPerUnitLengthOverride(Af const &spl) {}
  void m103HomingVelocityOverride(Af const &vel) {}
  void m104PrintInfo() const = 0;
  void m105MinPositionOverride(Af const &vel) {}
  void m106MaxPositionOverride(Af const &vel) {}
  void m110PrintAxesConfiguration() {}
  void error(const char *reason, const char *pos, const char *str) {}
  void start() {}
  void stop() {}
  bool isRunning() const = 0;
  void printCurrentPosition() const = 0;
  void clearCommandsBuffer() {}
};
*/

inline float strtofWithoutHex(const char *str, const char **str_end) {
    while (isspace(*str)) {
        ++str;
    }
    auto s = str;
    if (*s == '+' || *s == '-') {
        ++s;
    }
    if (*s == '0') {
        ++s;
        *str_end = s;
        if (*s == 'x' || *s == 'X') {
            return 0.f;
        }
    }
    return strtof(str, const_cast<char **>(str_end));
}

template <typename IGCodeInterpreter, typename AxesTraits = DefaultAxesTraits>
class GCodeParser {
    using Af = TAf<AxesTraits::size>;

  public:
    explicit GCodeParser(IGCodeInterpreter *cb) : cb_(cb) { scAssert(cb); }

    void parseLine(const char *str) {
        scAssert(str);
        str_ = str;
        curr_ = str;
        err_ = nullptr;
        skipSpaces();

        line();

        if (err_) {
            cb_->clearCommandsBuffer();
            cb_->error(err_, curr_ - str_, str_);
        }
    }

  private:
    static void updateAxisValue(Af *axes, char name, float value) {
        auto nameUp = toupper(name);
        auto nameIt = AxesTraits::names();
        for (auto axisIt = axes->begin(); axisIt != axes->end(); ++axisIt, ++nameIt) {
            if (*nameIt == nameUp) {
                *axisIt = value;
                return;
            }
        }
    }

    bool integer(int32_t *value) {
        if (!isDigit()) {
            return false;
        }
        *value = strtoul(curr_, const_cast<char **>(&curr_), 10);
        skipSpaces();
        return true;
    }

    bool floating(float *value) {
        if (!isDigit() && !isSign() && sym() != '.') {
            return false;
        }
        *value = strtofWithoutHex(curr_, &curr_);
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
            return error("expect floating value");
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
            return error("expect floating feed");
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
            return error("expect P");
        }
        nextsym();
        float sec = 0;
        if (!floating(&sec)) {
            return error("expect floating seconds");
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
        cb_->g90g91DistanceMode(DistanceMode::Absolute);
        return true;
    }

    bool g91RelativeDistanceMode() {
        if (!expectNewLine()) {
            return false;
        }
        cb_->g90g91DistanceMode(DistanceMode::Relative);
        return true;
    }

    bool gCommand() {
        if (!isGCommand()) {
            return false;
        }
        nextsym();
        int32_t command = 0;
        if (!integer(&command)) {
            return error("expect integer command");
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
            return error("unknown G command");
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

    bool m104PrintInfo() {
        if (!expectNewLine()) {
            return false;
        }
        cb_->m104PrintInfo();
        return true;
    }

    bool m105MinPositionOverride() {
        auto dist = axInf<Af>();
        axesFloat(&dist);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m105MinPositionOverride(dist);
        return true;
    }

    bool m106MaxPositionOverride() {
        auto dist = axInf<Af>();
        axesFloat(&dist);
        if (!expectNewLine()) {
            return false;
        }
        cb_->m106MaxPositionOverride(dist);
        return true;
    }

    bool m110PrintAxesConfiguration() {
        if (!expectNewLine()) {
            return false;
        }
        cb_->m110PrintAxesConfiguration();
        return true;
    }

    bool mCommand() {
        if (!isMCommand()) {
            return false;
        }
        nextsym();
        int32_t command = 0;
        if (!integer(&command)) {
            return error("expect integer command");
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
        case 104:
            return m104PrintInfo();
        case 105:
            return m105MinPositionOverride();
        case 106:
            return m106MaxPositionOverride();
        case 110:
            return m110PrintAxesConfiguration();
        default:
            return error("unknown M command");
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

    bool printCurrentPosition() {
        if (!isInfo()) {
            return false;
        }
        nextsym();
        if (!expectNewLine()) {
            return false;
        }
        cb_->printCompleted();
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
        if (printCurrentPosition()) {
            return true;
        }
        return expectNewLine();
    }

    bool expectNewLine() {
        if (!isNewLine()) {
            return error("expect new line");
        }
        nextsym();
        return true;
    }

    bool isGCommand() const { return toupper(sym()) == 'G'; }

    bool isMCommand() const { return toupper(sym()) == 'M'; }

    bool isFeedrate() const { return toupper(sym()) == 'F'; }

    bool isPause() const { return toupper(sym()) == 'P'; }

    bool isAxis() const {
        auto s = toupper(sym());
        auto names = AxesTraits::names();
        for (int i = 0; i < AxesTraits::size; i++) {
            if (names[i] == s) {
                return true;
            }
        }
        return false;
    }

    bool isDigit() const { return isdigit(sym()) != 0; }

    bool isSign() const { return sym() == '-' || sym() == '+'; }

    bool isNewLine() const { return sym() == '\n'; }

    bool isSpace() const { return !isNewLine() && isspace(sym()) != 0; }

    bool isStart() const { return sym() == '~'; }

    bool isStop() const { return sym() == '!'; }

    bool isClear() const { return sym() == '^'; }

    bool isInfo() const { return sym() == '?'; }

    void skipSpaces() {
        while (isSpace()) {
            ++curr_;
        }
    }

    bool nextsym() {
        if (sym() == 0) {
            return error("expect symbol");
        }
        ++curr_;
        skipSpaces();
        return true;
    }

    char sym() const { return *curr_; }

    bool error(const char *reason) {
        err_ = reason;
        return false;
    }

    const char *curr_{};
    const char *str_{};
    const char *err_{};
    IGCodeInterpreter *cb_{};
};
}
