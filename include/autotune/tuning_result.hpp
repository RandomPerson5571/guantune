#pragma once

#include "autotune/telemetry.hpp"
#include <optional>
#include <vector>

namespace autotune {

// COMPLETE alone means all confirmations and the final return succeeded.
enum class AutonTuneStatus {
    COMPLETE,
    TRIAL_LIMIT,
    ABORTED,
    INVALID_CONFIG,
    DRIVE_UNAVAILABLE,
    SENSOR_FAULT,
    TRAVEL_LIMIT,
    GAIN_LIMIT,
    RETURN_FAILED,
    NOT_CALIBRATED,
    MEASUREMENT_DISAGREEMENT,
    UNSUPPORTED_MODE,
    LOW_BATTERY,
    ALREADY_RUNNING
};

inline const char* statusName(AutonTuneStatus status) {
    switch (status) {
    case AutonTuneStatus::COMPLETE:
        return "COMPLETE";
    case AutonTuneStatus::TRIAL_LIMIT:
        return "TRIAL_LIMIT";
    case AutonTuneStatus::ABORTED:
        return "ABORTED";
    case AutonTuneStatus::INVALID_CONFIG:
        return "INVALID_CONFIG";
    case AutonTuneStatus::DRIVE_UNAVAILABLE:
        return "DRIVE_UNAVAILABLE";
    case AutonTuneStatus::SENSOR_FAULT:
        return "SENSOR_FAULT";
    case AutonTuneStatus::TRAVEL_LIMIT:
        return "TRAVEL_LIMIT";
    case AutonTuneStatus::GAIN_LIMIT:
        return "GAIN_LIMIT";
    case AutonTuneStatus::RETURN_FAILED:
        return "RETURN_FAILED";
    case AutonTuneStatus::NOT_CALIBRATED:
        return "NOT_CALIBRATED";
    case AutonTuneStatus::MEASUREMENT_DISAGREEMENT:
        return "MEASUREMENT_DISAGREEMENT";
    case AutonTuneStatus::UNSUPPORTED_MODE:
        return "UNSUPPORTED_MODE";
    case AutonTuneStatus::LOW_BATTERY:
        return "LOW_BATTERY";
    case AutonTuneStatus::ALREADY_RUNNING:
        return "ALREADY_RUNNING";
    }
    return "UNKNOWN";
}

// Forward measurements stay unchanged when the robot returns to the origin.
struct TrialReport {
    Gains gains{1.1, 0, 1.1};
    double target = 0;
    double batteryStartMillivolts = 0;
    double batteryMinimumMillivolts = 0;
    std::optional<double> independentDistance;
    double distanceInches = 0;  // Projected inches, or unwrapped degrees in angular mode.
    double overshootInches = 0; // Peak displacement beyond the target.
    double finalErrorInches = 0;
    double finalSpeedInchesPerSecond = 0;
    unsigned crossings = 0; // Full deadband crossings, not noisy zero crossings.
    bool settled = false;
    bool timedOut = false;
    std::uint32_t elapsedMs = 0;
    bool returnedToStart = false;
};

// A failed session may contain useful measurements, but its gains are not
// accepted for persistence or automatic application.
struct AutonTuneResult {
    AutonTuneStatus status = AutonTuneStatus::INVALID_CONFIG;
    // Confirmed on COMPLETE; best safely returned candidate on every failure.
    Gains gains{1.1, 0, 1.1};
    std::optional<unsigned> bestTrial;
    Mode mode = Mode::LATERAL;
    std::vector<TrialReport> trials;
    double integralWindupRange = 3; // Preserve the PID behavior tested by this session.
    std::vector<TelemetryRecord> telemetry;
    bool telemetryTruncated = false;
};

} // namespace autotune
