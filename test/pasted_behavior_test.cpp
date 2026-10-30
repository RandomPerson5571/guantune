#include "support/scripted_drive.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>

// These regressions exercise the supplied implementation's new public behavior
// through TuningDrive, without replacing PROS or LemLib APIs with test stubs.
void testLatchedAbort() {
    FakeDrive drive;
    SelfTuningAuton tuner(drive);
    tuner.abort();
    assert(tuner.run().status == AutonTuneStatus::ABORTED);
    assert(drive.writes == 0);
    assert(tuner.run().status == AutonTuneStatus::COMPLETE);
    drive.stopped();
}

void testConcurrentRunDoesNotConsumeAbort() {
    FakeDrive drive;
    SelfTuningAuton tuner(drive);
    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false;
    bool release = false;
    AutonTuneResult first;
    std::thread worker([&] {
        first = tuner.run([&](unsigned, const Gains&) {
            std::unique_lock<std::mutex> lock(mutex);
            entered = true;
            condition.notify_one();
            condition.wait(lock, [&] { return release; });
            return true;
        });
    });
    {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [&] { return entered; });
    }
    tuner.abort();
    const auto second = tuner.run();
    assert(second.status == AutonTuneStatus::ALREADY_RUNNING);
    {
        std::lock_guard<std::mutex> lock(mutex);
        release = true;
    }
    condition.notify_one();
    worker.join();
    assert(first.status == AutonTuneStatus::ABORTED && drive.writes == 0);
    assert(tuner.run().status == AutonTuneStatus::COMPLETE);
    drive.stopped();
}

struct SagDrive : FakeDrive {
    double lowVoltage = 6500;
    unsigned durationMs = 50;
    double batteryMillivolts() override {
        if (trialGains && static_cast<std::uint32_t>(time - start) >= 100 &&
            static_cast<std::uint32_t>(time - start) < 100 + durationMs)
            return lowVoltage;
        return 12000;
    }
};

void testBatterySag() {
    SagDrive transient;
    auto result = SelfTuningAuton(transient).run();
    assert(result.status == AutonTuneStatus::COMPLETE);
    assert(result.trials.front().batteryMinimumMillivolts == 6500);

    SagDrive sustained;
    sustained.durationMs = 500;
    result = SelfTuningAuton(sustained).run();
    assert(result.status == AutonTuneStatus::LOW_BATTERY && sustained.time < 500);
    sustained.stopped();

    SagDrive severe;
    severe.lowVoltage = 5000;
    result = SelfTuningAuton(severe).run();
    assert(result.status == AutonTuneStatus::LOW_BATTERY && severe.time == 100);
    severe.stopped();

    struct RestingLowDrive : FakeDrive {
        double batteryMillivolts() override { return 6500; }
    } low;
    assert(SelfTuningAuton(low).run().status == AutonTuneStatus::LOW_BATTERY);
    assert(low.writes == 0);
}

struct FrozenClockDrive : FakeDrive {
    unsigned polls = 0;
    void delayMs(std::uint32_t) override { ++polls; }
};

void testFrozenClock() {
    FrozenClockDrive drive;
    assert(SelfTuningAuton(drive).run().status == AutonTuneStatus::SENSOR_FAULT);
    assert(drive.polls < 1001);
    drive.stopped();
}

void testBestCandidateOnAbort() {
    FakeDrive drive;
    AutonTuneConfig config;
    config.maxTrials = 4;
    const auto result = SelfTuningAuton(drive, config).run([&](unsigned index, const Gains&) {
        drive.profile = index == 0 ? FakeDrive::GOOD : FakeDrive::SHORT;
        return index < 2;
    });
    assert(result.status == AutonTuneStatus::ABORTED && result.bestTrial == 0);
    assert(result.gains.kP == result.trials.front().gains.kP);
    assert(result.trials.size() == 2 && result.trials.back().returnedToStart);
    drive.stopped();
}

void testTargetHeadroom() {
    for (const double target : {33.0, 35.0}) {
        FakeDrive drive;
        AutonTuneConfig config;
        config.distanceInches = target;
        assert(SelfTuningAuton(drive, config).run().status == AutonTuneStatus::INVALID_CONFIG);
        assert(drive.writes == 0);
    }
    FakeDrive angular;
    AutonTuneConfig config;
    config.mode = Mode::ANGULAR;
    config.testDistances = {180};
    config.maxTravelInches = 220;
    assert(SelfTuningAuton(angular, config).run().status == AutonTuneStatus::INVALID_CONFIG);
    assert(angular.writes == 0);
}

struct IdleLagDrive : FakeDrive {
    std::uint32_t idleAtMs = 0;
    std::uint32_t lagMs = 40;
    std::uint32_t secondTrialRequestedAtMs = 0;
    SelfTuningAuton* cancelWhileWaiting = nullptr;
    void restoreGains() override {
        FakeDrive::restoreGains();
        if (!forwards)
            idleAtMs = time + lagMs;
    }
    bool isMoving() override { return (!active && time < idleAtMs) || FakeDrive::isMoving(); }
    void moveTo(const DrivePose& pose, bool forward, std::uint32_t timeout, double speed) override {
        if (homeSaved)
            secondTrialRequestedAtMs = time;
        FakeDrive::moveTo(pose, forward, timeout, speed);
    }
    void delayMs(std::uint32_t ms) override {
        FakeDrive::delayMs(ms);
        if (!active && time < idleAtMs && cancelWhileWaiting)
            cancelWhileWaiting->abort();
    }
};

void testIdleWaitIsBoundedAndCancellable() {
    IdleLagDrive drive;
    std::uint32_t beforeSecondTrialMs = 0;
    const auto result = SelfTuningAuton(drive).run([&](unsigned index, const Gains&) {
        if (index == 1)
            beforeSecondTrialMs = drive.time;
        return true;
    });
    assert(result.status == AutonTuneStatus::COMPLETE);
    assert(drive.secondTrialRequestedAtMs >= beforeSecondTrialMs + drive.lagMs);

    IdleLagDrive slow;
    slow.lagMs = 1000;
    const auto refused = SelfTuningAuton(slow).run();
    assert(refused.status == AutonTuneStatus::DRIVE_UNAVAILABLE && refused.trials.size() == 1);
    assert(slow.secondTrialRequestedAtMs == 0);
    slow.stopped();

    IdleLagDrive cancel;
    SelfTuningAuton tuner(cancel);
    cancel.cancelWhileWaiting = &tuner;
    assert(tuner.run().status == AutonTuneStatus::ABORTED);
    cancel.stopped();
}

void testFrozenReturnClock() {
    struct Drive : FakeDrive {
        unsigned stalledPolls = 0;
        void delayMs(std::uint32_t ms) override {
            if (forwards)
                FakeDrive::delayMs(ms);
            else
                ++stalledPolls;
        }
    } drive;
    const auto result = SelfTuningAuton(drive).run();
    assert(result.status == AutonTuneStatus::SENSOR_FAULT);
    assert(result.trials.size() == 1 && !result.trials.front().returnedToStart);
    assert(drive.stalledPolls < 1001);
    drive.stopped();
}

void testCleanupExceptionsDoNotMaskCallbackFailure() {
    struct Drive : FakeDrive {
        unsigned restoreAttempts = 0;
        void stop() override { throw std::runtime_error("stop failed"); }
        void restoreGains() override {
            ++restoreAttempts;
            throw std::runtime_error("restore failed");
        }
    } drive;
    try {
        SelfTuningAuton(drive).run(
            [](unsigned, const Gains&) -> bool { throw std::logic_error("callback failed"); });
        assert(false);
    } catch (const std::logic_error&) {
    }
    assert(drive.restoreAttempts == 1);
}

int main() {
    testLatchedAbort();
    testConcurrentRunDoesNotConsumeAbort();
    testBatterySag();
    testFrozenClock();
    testBestCandidateOnAbort();
    testTargetHeadroom();
    testIdleWaitIsBoundedAndCancellable();
    testFrozenReturnClock();
    testCleanupExceptionsDoNotMaskCallbackFailure();
    std::cout << "Pasted implementation lifecycle, battery and validation tests passed\n";
}
