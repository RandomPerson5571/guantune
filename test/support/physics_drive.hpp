#pragma once

#include "autotune/angle.hpp"
#include "autotune/idle_pid_guard.hpp"
#include "autotune/output.hpp"
#include "autotune/self_tuning_auton.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
using namespace autotune;

// First-order motor response with static friction, saturation, braking and noise.
// This is an independent model; there are no scripted success poses.
struct PhysicsDrive : TuningDrive {
    double position = 0, velocity = 0, target = 0, previousError = 0;
    double response = 0.08, friction = 2, noise = 0.002, voltage = 12000;
    double command = 0, maximum = 60;
    bool active = false, manual = false, swapped = false, ready = true;
    Mode mode = Mode::LATERAL;
    Gains gains{10, 0, 5};
    std::uint32_t time = 0;
    unsigned moves = 0;
    int throwAt = 0;
    DrivePose pose() override {
        if (throwAt == 2 && active)
            throw std::runtime_error("pose");
        const double measured = position + noise * std::sin(time * 0.017);
        return mode == Mode::LATERAL ? DrivePose{0, measured, 0} : DrivePose{0, 0, measured};
    }
    void setTrialGains(const Gains& g, double) override {
        assert(!active);
        gains = g;
        swapped = true;
    }
    void restoreGains() override {
        assert(!active);
        gains = {10, 0, 5};
        swapped = false;
    }
    void moveTo(const DrivePose& p, bool, std::uint32_t, double output) override {
        begin(p.y, output);
    }
    void turnTo(double heading, std::uint32_t, double output) override {
        // Match the real adapter: a clockwise turn to a compass heading, while
        // this model's physical position remains continuous across revolutions.
        const double clockwiseDistance =
            detail::normalizeHeading(heading - detail::normalizeHeading(position));
        begin(position + clockwiseDistance, output);
    }
    void begin(double t, double output) {
        target = t;
        maximum = output;
        previousError = target - position;
        active = true;
        manual = false;
        ++moves;
        if (throwAt == 1)
            throw std::runtime_error("move");
    }
    bool supportsAngular() const override { return true; }
    void selectMode(Mode m) override { mode = m; }
    void turnBack(double power) override { driveBackward(power); }
    void driveBackward(double power) override {
        command = -power;
        active = true;
        manual = true;
    }
    bool isMoving() override { return active; }
    void stop() override {
        active = false;
        command = 0;
    }
    std::uint32_t nowMs() override { return time; }
    void delayMs(std::uint32_t ms) override {
        if (throwAt == 3 && active)
            throw std::runtime_error("delay");
        for (std::uint32_t i = 0; i < ms; ++i) {
            if (active && !manual && (time + i) % 10 == 0) {
                const double error = target - position;
                command = std::clamp(gains.kP * error + gains.kD * (error - previousError),
                                     -maximum, maximum);
                previousError = error;
            }
            const double effective =
                std::copysign(std::max(0.0, std::abs(command) - friction), command);
            const double desired = effective * 0.7 * voltage / 12000;
            const double tau = active ? response : 0.02;
            velocity += (desired - velocity) * 0.001 / tau;
            position += velocity * 0.001;
        }
        time += ms;
    }
    bool available() override { return true; }
    bool calibrated() override { return ready; }
};

struct ConstPid {
    const double p;
    explicit ConstPid(double value) : p(value) {}
};

struct FrozenReturnDrive : PhysicsDrive {
    double lastForward = 0;
    DrivePose pose() override {
        if (!manual)
            lastForward = position;
        return {0, lastForward, 0};
    }
};

struct AngularReturnJumpDrive : PhysicsDrive {
    bool jumped = false;
    DrivePose pose() override {
        auto p = PhysicsDrive::pose();
        if (manual)
            jumped = true;
        if (jumped)
            p.x = 3;
        return p;
    }
};

struct WrappedPhysicsDrive : PhysicsDrive {
    DrivePose pose() override {
        auto p = PhysicsDrive::pose();
        p.headingDegrees = std::fmod(p.headingDegrees + 360, 360);
        return p;
    }
};

struct InitialReturnJumpDrive : PhysicsDrive {
    unsigned returnPoses = 0;
    bool returning = false;
    void restoreGains() override {
        if (swapped)
            returning = true;
        PhysicsDrive::restoreGains();
    }
    DrivePose pose() override {
        auto p = PhysicsDrive::pose();
        if (returning && ++returnPoses >= 1)
            p.x = 3;
        return p;
    }
};

// Simulate a heading update after the last forward sample but before the first
// return sample. It must be checked, not silently folded into a new baseline.
struct AngularBoundaryJumpDrive : PhysicsDrive {
    void restoreGains() override {
        if (swapped)
            position += 5;
        PhysicsDrive::restoreGains();
    }
};
