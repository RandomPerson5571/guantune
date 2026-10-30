#pragma once

#include "autotune/self_tuning_auton.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace autotune;

struct FakeDrive : TuningDrive {
    enum Profile {
        GOOD,
        SHORT,
        OVERSHOOT,
        OSCILLATE,
        NOISY,
        COAST,
        BAD_SENSOR,
        TOO_FAR
    } profile = GOOD;
    struct Motion {
        DrivePose from, target;
        bool forwards;
        Gains gains;
    };
    std::vector<Motion> motions;
    std::uint32_t time = 0, start = 0, stoppedAt = 0;
    DrivePose current{10, 20, 0}, from = current, target = current, home = current;
    bool homeSaved = false;
    double shortDistance = 22, returnOffset = 0;
    bool enabled = true, active = false, forwards = true, coasting = false, trialGains = false;
    bool abortOnReturn = false, disableOnReturn = false;
    unsigned writes = 0;
    Gains gains{10, 0, 5};
    SelfTuningAuton* cancel = nullptr;
    DrivePose pose() override {
        const double ms = static_cast<std::uint32_t>(time - start);
        if (active) {
            if (forwards) {
                double distance = std::min(24.0, ms * 0.024);
                if (profile == SHORT)
                    distance = std::min(shortDistance, ms * 0.024);
                if (profile == OVERSHOOT)
                    distance = ms < 1100 ? std::min(26.0, ms * 0.026)
                                         : std::max(24.0, 26 - (ms - 1100) * 0.01);
                if (profile == OSCILLATE)
                    distance = ms < 1000 ? ms * 0.024 : 24 + 2 * std::sin((ms - 1000) / 100);
                if (profile == NOISY && ms >= 1000)
                    distance = 24 + 0.02 * std::sin((ms - 1000) / 100);
                if (profile == BAD_SENSOR)
                    distance = std::numeric_limits<double>::quiet_NaN();
                if (profile == TOO_FAR)
                    distance = ms * 0.05;
                const double radians = home.headingDegrees * 3.14159265358979323846 / 180;
                const double initial =
                    (from.x - home.x) * std::sin(radians) + (from.y - home.y) * std::cos(radians);
                if (ms == 0)
                    distance = initial;
                current = {home.x + distance * std::sin(radians),
                           home.y + distance * std::cos(radians), home.headingDegrees};
            } else {
                const double fraction = std::min(1.0, ms / 1000);
                current = {from.x + (target.x - from.x) * fraction,
                           from.y + (target.y + returnOffset - from.y) * fraction,
                           target.headingDegrees};
            }
        } else if (coasting) {
            current.y = 44 + std::min(2.0, static_cast<std::uint32_t>(time - stoppedAt) * 0.01);
        }
        return current;
    }
    void setTrialGains(const Gains& g, double) override {
        assert(!active);
        trialGains = true;
        gains = g;
        ++writes;
    }
    void restoreGains() override {
        assert(!active);
        trialGains = false;
        gains = {10, 0, 5};
    }
    void moveTo(const DrivePose& t, bool forward, std::uint32_t, double maxSpeed) override {
        assert(!active && maxSpeed == 60);
        assert(forward); // Return must never call moveToPoint.
        from = pose();
        if (!homeSaved) {
            home = from;
            homeSaved = true;
        }
        target = t;
        forwards = forward;
        start = time;
        coasting = false;
        active = true;
        assert(forward == trialGains); // Return uses the original controller.
        motions.push_back({from, target, forward, gains});
        ++writes;
    }
    void driveBackward(double power) override {
        assert(!active && !trialGains && power >= 1 && power <= 40);
        from = pose();
        target = home;
        forwards = false;
        start = time;
        coasting = false;
        active = true;
        motions.push_back({from, target, false, gains});
        ++writes;
    }
    bool isMoving() override {
        const unsigned duration = !forwards              ? 1000
                                  : profile == OVERSHOOT ? 1400
                                  : profile == OSCILLATE || profile == NOISY || profile == TOO_FAR
                                      ? 5000
                                      : 1000;
        return active && static_cast<std::uint32_t>(time - start) < duration;
    }
    void stop() override {
        if (!active)
            return;
        pose();
        active = false;
        coasting = forwards && profile == COAST;
        stoppedAt = time;
    }
    std::uint32_t nowMs() override { return time; }
    void delayMs(std::uint32_t ms) override {
        time += ms;
        if (cancel && (!abortOnReturn || !forwards) &&
            static_cast<std::uint32_t>(time - start) >= 100)
            cancel->abort();
        if (disableOnReturn && !forwards)
            enabled = false;
    }
    bool available() override { return enabled; }
    void reset() {} // Callback no longer repositions: the runner returns automatically.
    void stopped() { assert(!active && !trialGains); }
};

// A queued asynchronous motion need not report running before its task starts.
struct DelayedStartDrive : FakeDrive {
    bool pending = false;
    std::uint32_t queuedAt = 0;
    unsigned startupDelayMs = 40;
    void moveTo(const DrivePose& target, bool forwards, std::uint32_t timeout,
                double speed) override {
        FakeDrive::moveTo(target, forwards, timeout, speed);
        active = false;
        pending = true;
        queuedAt = time;
    }
    void delayMs(std::uint32_t ms) override {
        FakeDrive::delayMs(ms);
        if (pending && static_cast<std::uint32_t>(time - queuedAt) >= startupDelayMs) {
            pending = false;
            active = true;
            start = time;
        }
    }
    void stop() override {
        pending = false;
        FakeDrive::stop();
    }
};

// Count actual asynchronous starts separately from requested trials.
struct CountingDelayedDrive : DelayedStartDrive {
    unsigned startedMotions = 0;
    unsigned cancelledBeforeStartup = 0;
    void delayMs(std::uint32_t ms) override {
        const bool wasPending = pending;
        DelayedStartDrive::delayMs(ms);
        if (wasPending && !pending)
            ++startedMotions;
    }
    void stop() override {
        if (pending)
            ++cancelledBeforeStartup;
        DelayedStartDrive::stop();
    }
};
