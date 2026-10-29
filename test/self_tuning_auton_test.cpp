#include "autotune/self_tuning_auton.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace autotune;

struct FakeDrive : TuningDrive {
    enum Profile { GOOD, SHORT, OVERSHOOT, OSCILLATE, NOISY, COAST, BAD_SENSOR, TOO_FAR } profile = GOOD;
    struct Motion { DrivePose from, target; bool forwards; Gains gains; };
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
                if (profile == SHORT) distance = std::min(shortDistance, ms * 0.024);
                if (profile == OVERSHOOT) distance = ms < 1100 ? std::min(26.0, ms * 0.026) : std::max(24.0, 26 - (ms - 1100) * 0.01);
                if (profile == OSCILLATE) distance = ms < 1000 ? ms * 0.024 : 24 + 2 * std::sin((ms - 1000) / 100);
                if (profile == NOISY && ms >= 1000) distance = 24 + 0.02 * std::sin((ms - 1000) / 100);
                if (profile == BAD_SENSOR) distance = std::numeric_limits<double>::quiet_NaN();
                if (profile == TOO_FAR) distance = ms * 0.05;
                const double radians = home.headingDegrees * 3.14159265358979323846 / 180;
                const double initial = (from.x - home.x) * std::sin(radians) + (from.y - home.y) * std::cos(radians);
                if (ms == 0) distance = initial;
                current = {home.x + distance * std::sin(radians), home.y + distance * std::cos(radians), home.headingDegrees};
            } else {
                const double fraction = std::min(1.0, ms / 1000);
                current = {from.x + (target.x - from.x) * fraction,
                           from.y + (target.y + returnOffset - from.y) * fraction, target.headingDegrees};
            }
        } else if (coasting) {
            current.y = 44 + std::min(2.0, static_cast<std::uint32_t>(time - stoppedAt) * 0.01);
        }
        return current;
    }
    void setTrialGains(const Gains& g, double) override {
        assert(!active); trialGains = true; gains = g; ++writes;
    }
    void restoreGains() override { assert(!active); trialGains = false; gains = {10, 0, 5}; }
    void moveTo(const DrivePose& t, bool forward, std::uint32_t, double maxSpeed) override {
        assert(!active && maxSpeed == 60);
        assert(forward); // Return must never call moveToPoint.
        from = pose();
        if (!homeSaved) { home = from; homeSaved = true; }
        target = t; forwards = forward; start = time;
        coasting = false; active = true;
        assert(forward == trialGains); // Return uses the original controller.
        motions.push_back({from, target, forward, gains}); ++writes;
    }
    void driveBackward(double power) override {
        assert(!active && !trialGains && power >= 1 && power <= 40);
        from = pose(); target = home; forwards = false; start = time;
        coasting = false; active = true;
        motions.push_back({from, target, false, gains}); ++writes;
    }
    bool isMoving() override {
        const unsigned duration = !forwards ? 1000 : profile == OVERSHOOT ? 1400 :
            profile == OSCILLATE || profile == NOISY || profile == TOO_FAR ? 5000 : 1000;
        return active && static_cast<std::uint32_t>(time - start) < duration;
    }
    void stop() override {
        if (!active) return;
        pose(); active = false;
        coasting = forwards && profile == COAST; stoppedAt = time;
    }
    std::uint32_t nowMs() override { return time; }
    void delayMs(std::uint32_t ms) override {
        time += ms;
        if (cancel && (!abortOnReturn || !forwards) && static_cast<std::uint32_t>(time - start) >= 100)
            cancel->abort();
        if (disableOnReturn && !forwards) enabled = false;
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
    void moveTo(const DrivePose& target, bool forwards, std::uint32_t timeout, double speed) override {
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

int main() {
    for (auto initialTime : {std::uint32_t{0}, std::uint32_t{0xfffffff0u}}) {
        DelayedStartDrive drive;
        drive.time = initialTime;
        auto result = SelfTuningAuton(drive).run();
        assert(result.status == AutonTuneStatus::COMPLETE);
        assert(result.trials.size() == 2);
        drive.stopped();
    }
    {
        DelayedStartDrive drive;
        drive.startupDelayMs = 1000;
        auto result = SelfTuningAuton(drive).run();
        assert(result.status == AutonTuneStatus::DRIVE_UNAVAILABLE);
        assert(drive.time == 250 && result.trials.size() == 1);
        assert(!drive.pending && !drive.active);
        drive.stopped();
    }
    {
        DelayedStartDrive drive;
        SelfTuningAuton tuner(drive);
        drive.cancel = &tuner;
        drive.startupDelayMs = 200;
        auto result = tuner.run();
        assert(result.status == AutonTuneStatus::ABORTED);
        assert(drive.time == 100 && !drive.pending);
        drive.stopped();
    }
    assert(SelfTuningAuton::gainStep(1.1) == 3);
    assert(SelfTuningAuton::gainStep(6) == 1);
    assert(SelfTuningAuton::gainStep(12) == 0.1);
    assert(SelfTuningAuton::gainStep(2, true) == 0.1);
    AutonTuneConfig c;
    {
        FakeDrive d; d.profile = FakeDrive::SHORT; d.shortDistance = 23.75;
        auto result = AutonTuner(d).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(result.status == AutonTuneStatus::COMPLETE); // Within default 0.5 inch.
        AutonTuneConfig strict;
        strict.range = 0.1;
        strict.maxTrials = 15;
        unsigned count = 0;
        auto limited = AutonTuner(d, strict).run([&](unsigned, const Gains&) {
            ++count; d.reset(); return true;
        });
        assert(limited.status == AutonTuneStatus::TRIAL_LIMIT);
        assert(count == 15 && limited.trials.size() == 15); d.stopped();
    }
    for (double range : {0.0, -0.1, 24.0, std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
        FakeDrive d; AutonTuneConfig invalid; invalid.range = range;
        auto r = AutonTuner(d, invalid).run([](unsigned, const Gains&) { return true; });
        assert(r.status == AutonTuneStatus::INVALID_CONFIG && d.writes == 0);
    }
    for (unsigned trials : {0u, 1u}) {
        FakeDrive d; AutonTuneConfig invalid; invalid.maxTrials = trials;
        auto r = AutonTuner(d, invalid).run([](unsigned, const Gains&) { return true; });
        assert(r.status == AutonTuneStatus::INVALID_CONFIG && d.writes == 0);
    }
    {
        FakeDrive d; AutonTuneConfig invalid; invalid.maxOutput = 40;
        auto r = AutonTuner(d, invalid).run();
        assert(r.status == AutonTuneStatus::INVALID_CONFIG && d.writes == 0);
    }
#if __cplusplus >= 202002L
    {
        FakeDrive d;
        AutonTuner tuner(d, {.range = 0.1, .maxTrials = 15});
        auto r = tuner.run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::COMPLETE); d.stopped();
    }
#endif
    {
        FakeDrive d; SelfTuningAuton tuner(d);
        auto r = tuner.run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::COMPLETE && r.trials.size() == 2);
        assert(r.gains.kP == 1.1 && r.gains.kD == 1.1 && r.gains.kI == 0);
        assert(r.trials[0].distanceInches == 24); d.stopped();
    }
    {
        FakeDrive d; c.maxTrials = 4; SelfTuningAuton tuner(d, c);
        auto r = tuner.run([&](unsigned i, const Gains&) {
            d.reset(); d.profile = i == 0 ? FakeDrive::SHORT : i == 1 ? FakeDrive::OVERSHOOT : FakeDrive::GOOD;
            return true;
        });
        assert(r.status == AutonTuneStatus::COMPLETE);
        assert(r.trials[1].gains.kP == 4.1);
        assert(std::abs(r.trials[2].gains.kD - 1.2) < 1e-9);
        assert(r.gains.kI == 0); d.stopped();
    }
    {
        FakeDrive d; d.profile = FakeDrive::OSCILLATE; c.maxTrials = 2;
        SelfTuningAuton tuner(d, c);
        auto r = tuner.run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::TRIAL_LIMIT);
        assert(r.trials[0].crossings >= 3 && r.trials[1].gains.kD == 4.1); d.stopped();
    }
    for (bool allow : {false, true}) {
        FakeDrive d; d.profile = FakeDrive::SHORT; c = {}; c.allowIntegral = allow;
        auto r = SelfTuningAuton(d, c).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::TRIAL_LIMIT);
        bool usedIntegral = false;
        for (const auto& trial : r.trials) usedIntegral |= trial.gains.kI > 0;
        assert(usedIntegral == allow);
        assert(r.trials[7].gains.kI == 0); d.stopped();
    }
    {
        FakeDrive d; d.profile = FakeDrive::NOISY;
        auto r = SelfTuningAuton(d).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::COMPLETE);
        for (const auto& trial : r.trials) assert(trial.crossings == 0);
        d.stopped();
    }
    {
        FakeDrive d; d.profile = FakeDrive::COAST; c = {}; c.maxTrials = c.confirmationTrials = 1;
        auto r = SelfTuningAuton(d, c).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::TRIAL_LIMIT);
        assert(r.trials[0].distanceInches == 26 && r.trials[0].overshootInches == 2);
        assert(!r.trials[0].settled); d.stopped();
    }
    {
        FakeDrive d; d.profile = FakeDrive::SHORT; c = {}; c.maxKp = 1.1;
        auto r = SelfTuningAuton(d, c).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::GAIN_LIMIT && r.gains.kP == 1.1); d.stopped();
    }
    for (auto profile : {FakeDrive::BAD_SENSOR, FakeDrive::TOO_FAR}) {
        FakeDrive d; d.profile = profile;
        auto r = SelfTuningAuton(d).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == (profile == FakeDrive::BAD_SENSOR ? AutonTuneStatus::SENSOR_FAULT : AutonTuneStatus::TRAVEL_LIMIT));
        d.stopped();
    }
    {
        FakeDrive d; SelfTuningAuton tuner(d); d.cancel = &tuner;
        auto r = tuner.run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::ABORTED); d.stopped();
    }
    {
        FakeDrive d;
        auto r = SelfTuningAuton(d).run([](unsigned, const Gains&) { return false; });
        assert(r.status == AutonTuneStatus::ABORTED); d.stopped();
        d.enabled = false;
        assert(SelfTuningAuton(d).run([](unsigned, const Gains&) { return true; }).status == AutonTuneStatus::DRIVE_UNAVAILABLE);
    }
    {
        FakeDrive d; c = {}; c.maxOutput = std::numeric_limits<double>::quiet_NaN();
        assert(SelfTuningAuton(d, c).run([](unsigned, const Gains&) { return true; }).status == AutonTuneStatus::INVALID_CONFIG);
        assert(d.writes == 0);
    }
    {
        FakeDrive d; d.time = 0xfffffff0u;
        auto r = SelfTuningAuton(d).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(r.status == AutonTuneStatus::COMPLETE); d.stopped();
    }
    {
        FakeDrive d; d.profile = FakeDrive::SHORT; d.shortDistance = 20;
        c = {}; c.maxTrials = c.confirmationTrials = 2;
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::TRIAL_LIMIT && d.motions.size() == 4);
        for (unsigned i = 0; i < d.motions.size(); i += 2) {
            assert(d.motions[i].forwards && !d.motions[i + 1].forwards);
            assert(std::abs(d.motions[i].from.y - 20) <= 0.5 && d.motions[i].target.y == 44);
            assert(d.motions[i + 1].from.y == 40 && d.motions[i + 1].target.y == 20);
        }
        for (const auto& trial : r.trials) assert(trial.distanceInches == 20 && trial.returnedToStart);
        assert(std::abs(d.pose().y - 20) <= 0.5); d.stopped();
    }
    {
        FakeDrive d; d.returnOffset = 3;
        auto r = SelfTuningAuton(d).run();
        assert(r.status == AutonTuneStatus::RETURN_FAILED && r.trials.size() == 1);
        assert(!r.trials[0].returnedToStart && d.motions.size() == 2); d.stopped();
    }
    {
        FakeDrive d; d.returnOffset = -2;
        c = {}; c.returnToleranceInches = 0.01;
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::RETURN_FAILED && r.trials.size() == 1);
        assert(d.pose().y < 20); // Missed the narrow range: stop after crossing it.
        d.stopped();
    }
    for (double power : {0.0, -20.0, 41.0, std::numeric_limits<double>::quiet_NaN()}) {
        FakeDrive d; c = {}; c.returnPower = power;
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::INVALID_CONFIG);
        assert(d.writes == 0);
    }
    for (bool disabled : {false, true}) {
        FakeDrive d; SelfTuningAuton tuner(d); d.cancel = disabled ? nullptr : &tuner;
        d.abortOnReturn = true; d.disableOnReturn = disabled;
        auto r = tuner.run();
        assert(r.status == (disabled ? AutonTuneStatus::DRIVE_UNAVAILABLE : AutonTuneStatus::ABORTED));
        assert(d.motions.size() == 2 && !r.trials[0].returnedToStart); d.stopped();
    }
    {
        FakeDrive d; d.current = {5, 8, 90};
        auto r = SelfTuningAuton(d).run();
        assert(r.status == AutonTuneStatus::COMPLETE);
        assert(std::abs(d.motions[0].target.x - 29) < 1e-9);
        assert(std::abs(d.motions[1].target.x - 5) < 1e-9); d.stopped();
    }
    {
        FakeDrive d; AutonTuneConfig config; config.maxTrials = 3;
        config.bisectOnReversal = true;
        auto r = SelfTuningAuton(d, config).run([&](unsigned index, const Gains&) {
            d.profile = index == 1 ? FakeDrive::OVERSHOOT : FakeDrive::SHORT;
            return true;
        });
        assert(r.status == AutonTuneStatus::TRIAL_LIMIT && r.trials.size() == 3);
        assert(std::abs(r.trials[2].gains.kP - 2.6) < 1e-9);
        assert(r.trials[2].gains.kD == 1.1); d.stopped();
    }
    std::cout << "Self-tuning autonomous tests passed\n";
}
