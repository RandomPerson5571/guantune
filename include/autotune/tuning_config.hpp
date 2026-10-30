#pragma once

#include "autotune/telemetry.hpp"
#include <functional>
#include <optional>
#include <vector>

namespace autotune {

// Field names retain the original API. In angular mode, target/error/speed
// values use degrees and degrees/second despite the legacy "Inches" names.
// maxCrossTrackInches and maxTranslationalSensorSpeed always describe translation.
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

    // Gain bounds and the deterministic search schedule.
    double integralWindupRange = 3;
    double maxKp = 30;
    double maxKd = 30;
    double maxKi = 0.05;
    double kiStep = 0.001;
    unsigned maxTrials = 20;         // Total forward trials, including confirmations.
    unsigned confirmationTrials = 2; // Complete successful cycles through all targets.
    unsigned integralAfterPdAdjustments = 8;
    unsigned integralUndershootTrials = 3;
    bool allowIntegral = false; // Explicit opt-in; always tune P/D first.

    // Motion deadlines and how long a stationary reading must persist.
    std::uint32_t trialTimeoutMs = 5000;
    std::uint32_t settleMs = 500;
    std::uint32_t postStopMs = 300; // Observe coasting after output reaches zero.

    // The return uses fixed power, not the candidate controller.
    double returnToleranceInches = 0.5;
    double returnPower = 20; // LemLib tank command magnitude, 1-40.
    std::uint32_t returnTimeoutMs = 5000;

    // Effective targets and optional callbacks. Callbacks must be prompt;
    // none may start another motion while the tuner owns the drivetrain.
    Mode mode = Mode::LATERAL;
    std::vector<double> testDistances = {}; // Empty uses distanceInches. Validate every target.
    double minimumBatteryMillivolts = 7000; // Strict at rest; brief load sag is tolerated.
    std::function<bool()> requireConfirmation = {}; // Once before any movement; false aborts.
    // Independent signed displacement from session origin, in mode units.
    std::function<double()> independentDisplacement = {};
    double maxMeasurementDisagreement = 2;
    bool bisectOnReversal = false;
    // Called after a failed trial only when another trial remains. Arguments:
    // tested gains, final error, peak overshoot, zero-based trial index.
    // Return the next candidate; finite values and gain caps remain mandatory.
    std::function<Gains(const Gains&, double, double, unsigned)> adjustmentPolicy = {};

    // Independent x/y jump limit for angular tests; always inches/second.
    // Lateral tests continue to use maxSensorSpeed for translation.
    double maxTranslationalSensorSpeed = 120;

    // Optional bounded recording. Motor I/O runs every fifth recorded sample.
    bool enableTelemetry = false;
    std::uint32_t telemetryPeriodMs = 20;
    std::size_t maxTelemetrySamples = 4096; // Valid range: 1..100000 when enabled.
    std::vector<std::int8_t> telemetryMotorPorts =
        {}; // Signed ports, no duplicate physical motors.
    std::function<TelemetryCommand()> telemetryCommandProbe = {}; // Prompt, thread-safe snapshot.
};

} // namespace autotune
