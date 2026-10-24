#pragma once

#include "autotune/types.hpp"
#include <atomic>
#include <functional>
#include <vector>

namespace autotune {

struct DrivePose { double x, y, headingDegrees; };

// The caller grants this runner exclusive drivetrain access for run().
class TuningDrive {
public:
    virtual ~TuningDrive() = default;
    virtual DrivePose pose() = 0;
    virtual void tank(double left, double right) = 0;
    virtual std::uint32_t nowMs() = 0;
    virtual void delayMs(std::uint32_t ms) = 0;
    virtual bool available() = 0; // False on disable, another motion, or sensor failure.
};

struct AutonTuneConfig {
    // Acceptable distance error in inches. When set, overrides toleranceInches.
    std::optional<double> range = std::nullopt;
    double distanceInches = 24;
    double maxOutput = 60;
    double toleranceInches = 0.5;
    double maxSettledSpeed = 1; // inches/second
    double crossingDeadband = 0.5;
    double maxTravelInches = 36;
    double maxCrossTrackInches = 4;
    double maxHeadingErrorDegrees = 20;
    double maxSensorSpeed = 120; // inches/second
    double headingKp = 2;
    double integralWindupRange = 3;
    double maxKp = 30;
    double maxKd = 30;
    double maxKi = 0.05;
    double kiStep = 0.001;
    unsigned maxTrials = 20;
    unsigned confirmationTrials = 2;
    unsigned integralAfterPdAdjustments = 8;
    unsigned integralUndershootTrials = 3;
    bool allowIntegral = false; // Explicit opt-in; always tune P/D first.
    std::uint32_t trialTimeoutMs = 5000;
    std::uint32_t settleMs = 500;
    std::uint32_t postStopMs = 300; // Observe coasting after output reaches zero.
};

enum class AutonTuneStatus {
    COMPLETE, TRIAL_LIMIT, ABORTED, INVALID_CONFIG, DRIVE_UNAVAILABLE,
    SENSOR_FAULT, TRAVEL_LIMIT, GAIN_LIMIT
};

struct TrialReport {
    Gains gains{1.1, 0, 1.1};
    double distanceInches = 0; // Final displacement projected onto starting heading.
    double overshootInches = 0; // Peak displacement beyond the target.
    double finalErrorInches = 0;
    double finalSpeedInchesPerSecond = 0;
    unsigned crossings = 0; // Full deadband crossings, not noisy zero crossings.
    bool settled = false;
    bool timedOut = false;
    std::uint32_t elapsedMs = 0;
};

struct AutonTuneResult {
    AutonTuneStatus status = AutonTuneStatus::INVALID_CONFIG;
    Gains gains{1.1, 0, 1.1}; // Last tested gains; only COMPLETE means validated.
    std::vector<TrialReport> trials;
};

class SelfTuningAuton {
public:
    // Callback runs with motors stopped before EVERY trial, including trial zero.
    // Reposition/allow another 24-inch run, then return true; false aborts.
    using BeforeTrial = std::function<bool(unsigned, const Gains&)>;
    explicit SelfTuningAuton(TuningDrive& drive, AutonTuneConfig config = {});
    AutonTuneResult run(const BeforeTrial& beforeTrial);
    void abort(); // Can be called from another task. run() remains blocking.

    // Large changes below 6, medium below 12, fine above 12 or after reversal.
    static double gainStep(double gain, bool fine = false);

private:
    TuningDrive& drive_;
    AutonTuneConfig config_;
    std::atomic<bool> aborted_{false};
    std::atomic_flag running_ = ATOMIC_FLAG_INIT;
    AutonTuneStatus trial(const Gains&, TrialReport&);
};

using AutonTuner = SelfTuningAuton;

} // namespace autotune
