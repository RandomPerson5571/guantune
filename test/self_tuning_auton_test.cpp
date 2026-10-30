#include "support/scripted_drive.hpp"

void testRepeatedDelayedStartup() {
    CountingDelayedDrive drive;
    AutonTuneConfig config;
    config.range = 0.1;
    config.crossingDeadband = 0.1;
    config.maxTrials = 4;
    const auto result = SelfTuningAuton(drive, config).run();
    assert(result.status == AutonTuneStatus::COMPLETE);
    assert(result.trials.size() == 2 && drive.startedMotions == 2);
    assert(drive.cancelledBeforeStartup == 0);
    assert(result.trials.back().settled && result.trials.back().returnedToStart);
    drive.stopped();
}

void testImpossibleReturnTiming() {
    for (const auto postStopMs : {5000u, 6000u}) {
        FakeDrive drive;
        AutonTuneConfig config;
        config.postStopMs = postStopMs;
        const auto result = SelfTuningAuton(drive, config).run();
        assert(result.status == AutonTuneStatus::INVALID_CONFIG);
        assert(result.trials.empty() && drive.writes == 0);
    }
    // A longer coast period is allowed when the return deadline can fit it.
    FakeDrive drive;
    AutonTuneConfig config;
    config.postStopMs = 600;
    const auto result = SelfTuningAuton(drive, config).run();
    assert(result.status == AutonTuneStatus::COMPLETE);
    drive.stopped();
}

void testNoAdjustmentAfterTrialLimit() {
    FakeDrive drive;
    AutonTuneConfig config;
    config.maxTrials = 3;
    unsigned policyCalls = 0;
    config.adjustmentPolicy = [&](const Gains&, double, double, unsigned index) {
        ++policyCalls;
        if (index == 2) throw std::logic_error("No budget to test another candidate");
        return Gains{4.1, 0, 1.1};
    };
    const auto result = SelfTuningAuton(drive, config).run([&](unsigned index, const Gains&) {
        drive.profile = index == 0 ? FakeDrive::GOOD : FakeDrive::SHORT;
        return true;
    });
    assert(result.status == AutonTuneStatus::TRIAL_LIMIT && result.trials.size() == 3);
    assert(policyCalls == 1 && result.bestTrial == 0);
    assert(result.gains.kP == result.trials.front().gains.kP);
    assert(result.trials.back().returnedToStart);
    drive.stopped();
}

int main() {
    testRepeatedDelayedStartup();
    testImpossibleReturnTiming();
    testNoAdjustmentAfterTrialLimit();
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
    assert(std::abs(SelfTuningAuton::gainStep(6) - 1.2) < 1e-9);
    assert(std::abs(SelfTuningAuton::gainStep(12) - 2.4) < 1e-9);
    assert(SelfTuningAuton::gainStep(100, true) == 2);
    assert(SelfTuningAuton::gainStep(2, true) == 0.1);
    AutonTuneConfig c;
    {
        FakeDrive d; d.profile = FakeDrive::SHORT; d.shortDistance = 23.75;
        auto result = AutonTuner(d).run([&](unsigned, const Gains&) { d.reset(); return true; });
        assert(result.status == AutonTuneStatus::COMPLETE); // Within default 0.5 inch.
        AutonTuneConfig strict;
        strict.range = 0.1;
        strict.maxTrials = 15;
        strict.maxKp = 100;
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
        assert(r.status == (allow ? AutonTuneStatus::TRIAL_LIMIT : AutonTuneStatus::GAIN_LIMIT));
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
        assert(r.status == AutonTuneStatus::TRIAL_LIMIT && r.trials.size() == 2);
        unsigned forwards = 0;
        unsigned returns = 0;
        for (const auto& motion : d.motions) {
            if (motion.forwards) {
                ++forwards;
                assert(std::abs(motion.from.y - 20) <= 0.5 && motion.target.y == 44);
            } else {
                ++returns;
                assert(motion.target.y == 20); // Creeps still use the fixed origin.
            }
        }
        assert(forwards == 2 && returns > 2);
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
        assert(r.status == AutonTuneStatus::COMPLETE);
        assert(std::abs(d.pose().y - 20) <= c.returnToleranceInches); // Predictive stop/creep reaches the narrow window.
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
