#include "autotune/self_tuning_auton.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

using namespace autotune;

struct FakeDrive : TuningDrive {
    enum Profile { GOOD, SHORT, OVERSHOOT, OSCILLATE, NOISY, COAST, BAD_SENSOR, TOO_FAR, SIMULATED } profile = GOOD;
    std::uint32_t time = 0, start = 0;
    double left = 0, right = 0, position = 0, velocity = 0;
    bool enabled = true;
    unsigned writes = 0;
    SelfTuningAuton* cancel = nullptr;
    DrivePose pose() override {
        const double ms = time - start;
        double distance = std::min(24.0, ms * 0.024);
        if (profile == SHORT) distance = std::min(22.0, ms * 0.024);
        if (profile == OVERSHOOT) distance = ms < 1100 ? std::min(26.0, ms * 0.026) : std::max(24.0, 26 - (ms - 1100) * 0.01);
        if (profile == OSCILLATE) distance = ms < 1000 ? ms * 0.024 : 24 + 2 * std::sin((ms - 1000) / 100);
        if (profile == NOISY && ms >= 1000) distance = 24 + 0.02 * std::sin((ms - 1000) / 100);
        if (profile == COAST && ms > 1500) distance = 24 + std::min(2.0, (ms - 1500) * 0.01);
        if (profile == BAD_SENSOR) distance = std::numeric_limits<double>::quiet_NaN();
        if (profile == TOO_FAR) distance = ms * 0.05;
        if (profile == SIMULATED) distance = position;
        return {10, 20 + distance, 0}; // Nonzero origin checks relative distance.
    }
    void tank(double l, double r) override {
        assert(std::abs(l) <= 60.000001 && std::abs(r) <= 60.000001);
        left = l; right = r; ++writes;
    }
    std::uint32_t nowMs() override { return time; }
    void delayMs(std::uint32_t ms) override {
        if (profile == SIMULATED) {
            velocity += ((left + right) * 0.5 - velocity * 2) * ms / 1000;
            position += velocity * ms / 1000;
        }
        time += ms;
        if (cancel && time - start >= 100) cancel->abort();
    }
    bool available() override { return enabled; }
    void reset() { start = time; position = velocity = 0; }
    void stopped() { assert(left == 0 && right == 0); }
};

int main() {
    assert(SelfTuningAuton::gainStep(1.1) == 3);
    assert(SelfTuningAuton::gainStep(6) == 1);
    assert(SelfTuningAuton::gainStep(12) == 0.1);
    assert(SelfTuningAuton::gainStep(2, true) == 0.1);
    AutonTuneConfig c;
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
        FakeDrive d; d.profile = FakeDrive::SIMULATED;
        auto r = SelfTuningAuton(d).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(!r.trials.empty());
        assert(std::abs(r.trials[0].distanceInches) > 1);
        d.stopped();
    }
    std::cout << "Self-tuning autonomous tests passed\n";
}
