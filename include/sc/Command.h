#pragma once

#include "Axes.h"

#include <vector>

namespace StepperControl {
// Represents gcode program instruction.
template <size_t AxesSize>
struct Command {
    using Af = Axes<float, AxesSize>;
    using Ai = Axes<int32_t, AxesSize>;

    enum Type {
        Move,
        Wait,
        Homing,
    };

    Command(Command const &other) = default;

    Command(Command &&other)
        : type_(other.type_), path_(std::move(other.path_)),
          maxAcceleration_(std::move(other.maxAcceleration_)), u(other.u) {}

    Command &operator=(Command const &other) = default;

    Command &operator=(Command &&other) {
        if (this == &other)
            return *this;
        type_ = other.type_;
        path_ = std::move(other.path_);
        maxAcceleration_ = other.maxAcceleration_;
        u = other.u;
        return *this;
    }

    // Move
    Command(std::vector<Ai> const &path, Af const &maxVel, Af const &maxAccel)
        : type_(Move), path_(path), maxAcceleration_(maxAccel) {
        u.maxVelocity_ = maxVel;
    }

    // Wait
    explicit Command(int32_t waitDuration) : type_(Wait) { u.waitDuration_ = waitDuration; }

    // Homing
    explicit Command(Af const &homingVelocity) : type_(Homing) {
        u.homingVelocity_ = homingVelocity;
    }

    inline void push_back(Ai const &waypoint) {
        scAssert(type() == Move);
        path_.push_back(waypoint);
    }

    inline bool empty() const {
        scAssert(type() == Move);
        return path_.empty();
    }

    inline Type const &type() const { return type_; }

    inline std::vector<Ai> const &path() const {
        scAssert(type() == Move);
        return path_;
    }

    inline std::vector<Ai> &path() {
        scAssert(type() == Move);
        return path_;
    }

    inline Af const &maxVelocity() const {
        scAssert(type() == Move);
        return u.maxVelocity_;
    }

    inline Af const &homingVelocity() const {
        scAssert(type() == Homing);
        return u.homingVelocity_;
    }

    inline Af const &maxAcceleration() const {
        scAssert(type() == Move);
        return maxAcceleration_;
    }

    inline int32_t waitDuration() const {
        scAssert(type() == Wait);
        return u.waitDuration_;
    }

  private:
    Type type_;

    // In steps.
    std::vector<Ai> path_;

    // In steps per tick^2.
    Af maxAcceleration_;

    union {
        // In steps per tick.
        Af homingVelocity_;

        // In steps per tick.
        Af maxVelocity_;
        // In ticks.
        int32_t waitDuration_;
    } u;
};
}
