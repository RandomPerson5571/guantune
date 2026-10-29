#pragma once

#include "autotune/types.hpp"
#include "autotune/telemetry.hpp"
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
    virtual void setTrialGains(const Gains& gains, double windupRange) = 0;
    virtual void restoreGains() = 0;
    virtual void moveTo(const DrivePose& target, bool forwards,
                        std::uint32_t timeoutMs, double maxSpeed) = 0;
    virtual bool isMoving() = 0;
    virtual void driveBackward(double power) = 0; // Fixed low power; no PID.
    virtual void stop() = 0; // Cancel and wait until the owned motion has stopped.
    virtual std::uint32_t nowMs() = 0;
    virtual void delayMs(std::uint32_t ms) = 0;
    virtual double batteryMillivolts() { return 12000; } // Override on hardware.
    virtual std::vector<MotorTelemetry> motorTelemetry(const std::vector<std::int8_t>&) { return {}; }
    virtual bool calibrated() { return true; } // Hardware adapter uses caller's readiness check.
    virtual bool supportsAngular() const { return false; }
    virtual void selectMode(Mode) {}
    virtual void turnTo(double, std::uint32_t, double) {}
    virtual void turnBack(double) {}
    virtual bool available() = 0; // False on disable; owned motions remain available.
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
    double returnToleranceInches = 0.5;
    double returnPower = 20; // LemLib tank command magnitude, 1-40.
    std::uint32_t returnTimeoutMs = 5000;
    Mode mode = Mode::LATERAL;
    // In angular mode, distance/overshoot/speed settings use degrees instead of inches.
    std::vector<double> testDistances = {}; // Empty uses distanceInches. Validate every target.
    double minimumBatteryMillivolts = 7000;
    std::function<bool()> requireConfirmation = {}; // Once before any movement; false aborts.
    // Independent signed displacement from session origin, in mode units.
    std::function<double()> independentDisplacement = {};
    double maxMeasurementDisagreement = 2;
    bool bisectOnReversal = false;
    // Called after a failed trial; return the next candidate. Bounds still enforced.
    std::function<Gains(const Gains&, double, double, unsigned)> adjustmentPolicy = {};
    // Independent x/y jump limit for angular tests; always inches/second.
    // Lateral tests continue to use maxSensorSpeed for translation.
    double maxTranslationalSensorSpeed = 120;
    bool enableTelemetry = false;
    std::uint32_t telemetryPeriodMs = 20; // At least the 10 ms control sample interval.
    std::size_t maxTelemetrySamples = 4096; // Stop recording when full; tuning continues.
    std::vector<std::int8_t> telemetryMotorPorts = {}; // Explicit drivetrain ports, including reversal.
    // LemLib does not expose its live commands. Supply a prompt, thread-safe probe
    // of instrumentation in your controller to capture trial output/voltage.
    std::function<TelemetryCommand()> telemetryCommandProbe = {};
};

enum class AutonTuneStatus {
    COMPLETE, TRIAL_LIMIT, ABORTED, INVALID_CONFIG, DRIVE_UNAVAILABLE,
    SENSOR_FAULT, TRAVEL_LIMIT, GAIN_LIMIT, RETURN_FAILED,
    NOT_CALIBRATED, LOW_BATTERY, MEASUREMENT_DISAGREEMENT, UNSUPPORTED_MODE
};

inline const char* statusName(AutonTuneStatus status) {
    switch (status) {
        case AutonTuneStatus::COMPLETE: return "COMPLETE";
        case AutonTuneStatus::TRIAL_LIMIT: return "TRIAL_LIMIT";
        case AutonTuneStatus::ABORTED: return "ABORTED";
        case AutonTuneStatus::INVALID_CONFIG: return "INVALID_CONFIG";
        case AutonTuneStatus::DRIVE_UNAVAILABLE: return "DRIVE_UNAVAILABLE";
        case AutonTuneStatus::SENSOR_FAULT: return "SENSOR_FAULT";
        case AutonTuneStatus::TRAVEL_LIMIT: return "TRAVEL_LIMIT";
        case AutonTuneStatus::GAIN_LIMIT: return "GAIN_LIMIT";
        case AutonTuneStatus::RETURN_FAILED: return "RETURN_FAILED";
        case AutonTuneStatus::NOT_CALIBRATED: return "NOT_CALIBRATED";
        case AutonTuneStatus::LOW_BATTERY: return "LOW_BATTERY";
        case AutonTuneStatus::MEASUREMENT_DISAGREEMENT: return "MEASUREMENT_DISAGREEMENT";
        case AutonTuneStatus::UNSUPPORTED_MODE: return "UNSUPPORTED_MODE";
    }
    return "UNKNOWN";
}

struct TrialReport {
    Gains gains{1.1, 0, 1.1};
    double target = 0;
    double batteryStartMillivolts = 0;
    double batteryMinimumMillivolts = 0;
    std::optional<double> independentDistance;
    double distanceInches = 0; // Final displacement projected onto starting heading.
    double overshootInches = 0; // Peak displacement beyond the target.
    double finalErrorInches = 0;
    double finalSpeedInchesPerSecond = 0;
    unsigned crossings = 0; // Full deadband crossings, not noisy zero crossings.
    bool settled = false;
    bool timedOut = false;
    std::uint32_t elapsedMs = 0;
    bool returnedToStart = false;
};

struct AutonTuneResult {
    AutonTuneStatus status = AutonTuneStatus::INVALID_CONFIG;
    Gains gains{1.1, 0, 1.1}; // Best measured candidate at trial limit; COMPLETE alone is validated.
    std::optional<unsigned> bestTrial;
    Mode mode = Mode::LATERAL;
    std::vector<TrialReport> trials;
    double integralWindupRange = 3; // Preserve the PID behavior tested by this session.
    std::vector<TelemetryRecord> telemetry;
    bool telemetryTruncated = false;
};

class SelfTuningAuton {
public:
    // Optional callback runs before each forward trial. Returns are automatic.
    // Return false to abort; do not reposition the robot inside this callback.
    using BeforeTrial = std::function<bool(unsigned, const Gains&)>;
    explicit SelfTuningAuton(TuningDrive& drive, AutonTuneConfig config = {});
    AutonTuneResult run(const BeforeTrial& beforeTrial = {});
    void abort(); // Can be called from another task. run() remains blocking.

    // Large changes below 6, medium below 12, fine above 12 or after reversal.
    static double gainStep(double gain, bool fine = false);

private:
    TuningDrive& drive_;
    AutonTuneConfig config_;
    std::atomic<bool> aborted_{false};
    std::atomic_flag running_ = ATOMIC_FLAG_INIT;
    struct TelemetrySession;
    AutonTuneStatus trial(const Gains&, const DrivePose& start, TrialReport&, TelemetrySession&);
    AutonTuneStatus returnToStart(const DrivePose& start, double displacement, TelemetrySession&);
    void recordTelemetry(TelemetrySession&, TelemetryPhase, std::uint32_t now,
                         double target, double actual, std::optional<double> velocity,
                         double battery, TelemetryCommand knownCommand = {});
    AutonTuneStatus checkIndependentDisplacement(double displacement,
                                                 std::optional<double>* reading = nullptr);
};

using AutonTuner = SelfTuningAuton;

} // namespace autotune
