#include "autotune/pid_validation.hpp"
#include "session_internal.hpp"
#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace autotune {
namespace detail {
namespace {
bool allPositive(std::initializer_list<double> values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value) && value > 0; });
}

bool validTarget(double target, const AutonTuneConfig& config) {
    // Reserve room to measure a tunable overshoot before hitting the travel
    // limit. Reject 180-degree targets, which are ambiguous for shortest turns.
    const double headroom = std::max(2 * config.toleranceInches, 0.1 * target);
    return std::isfinite(target) && target > config.toleranceInches &&
           target > config.crossingDeadband && target > config.returnToleranceInches &&
           target + headroom < config.maxTravelInches &&
           (config.mode != Mode::ANGULAR || target < 180);
}

const char* telemetryError(const AutonTuneConfig& config) {
    if (!config.enableTelemetry)
        return nullptr;
    if (config.telemetryPeriodMs < controlPeriodMs || config.telemetryPeriodMs >= 0x80000000u)
        return "telemetryPeriodMs out of range";
    if (config.maxTelemetrySamples == 0 || config.maxTelemetrySamples > maxTelemetrySamplesCap)
        return "maxTelemetrySamples must be in [1, 100000]";
    bool seen[22] = {};
    for (auto port : config.telemetryMotorPorts) {
        const int physicalPort = std::abs(static_cast<int>(port));
        if (physicalPort < 1 || physicalPort > 21 || seen[physicalPort])
            return "telemetryMotorPorts contains an invalid or duplicate port";
        seen[physicalPort] = true;
    }
    return nullptr;
}
} // namespace

const char* validationError(const AutonTuneConfig& config) {
    if (!allPositive({config.maxOutput, config.toleranceInches, config.maxSettledSpeed,
                      config.crossingDeadband, config.maxTravelInches, config.maxCrossTrackInches,
                      config.maxHeadingErrorDegrees, config.maxSensorSpeed,
                      config.maxTranslationalSensorSpeed, config.returnToleranceInches,
                      config.returnPower, config.integralWindupRange, config.maxKp, config.maxKd,
                      config.maxKi, config.kiStep}))
        return "a required positive limit is not finite and > 0";
    if (!std::isfinite(config.minimumBatteryMillivolts) || config.minimumBatteryMillivolts < 0)
        return "minimumBatteryMillivolts must be finite and >= 0";
    if (!std::isfinite(config.maxMeasurementDisagreement) || config.maxMeasurementDisagreement <= 0)
        return "maxMeasurementDisagreement must be finite and > 0";
    if (config.mode != Mode::LATERAL && config.mode != Mode::ANGULAR)
        return "unknown mode";
    if (const auto error = telemetryError(config))
        return error;
    if (config.testDistances.empty()) {
        if (!validTarget(config.distanceInches, config))
            return "distanceInches is out of range";
    } else {
        for (double target : config.testDistances)
            if (!validTarget(target, config))
                return "a testDistances entry is out of range";
    }
    if (!validPidGains({config.maxKp, config.maxKi, config.maxKd}) ||
        !validPidScalar(config.integralWindupRange) || !validPidScalar(config.kiStep))
        return "PID limits are invalid";
    // LemLib may raise moveToPoint's near-target output cap to at least 60.
    if (config.maxOutput < 60 || config.maxOutput > 127)
        return "maxOutput must be in [60, 127]";
    if (config.returnPower < 1 || config.returnPower > 40)
        return "returnPower must be in [1, 40]";
    if (config.maxKp < initialKp || config.maxKd < initialKd)
        return "maxKp/maxKd are below the starting gains";
    if (config.kiStep > config.maxKi)
        return "kiStep must be <= maxKi";
    const auto targetCount = std::max<std::size_t>(1, config.testDistances.size());
    // Division avoids overflowing a large confirmation-count/target-count product.
    if (config.maxTrials == 0 || config.confirmationTrials == 0 ||
        config.confirmationTrials > config.maxTrials / targetCount)
        return "trial/confirmation counts are inconsistent";
    if (config.integralAfterPdAdjustments == 0 || config.integralUndershootTrials == 0)
        return "integral gating counts must be > 0";
    if (config.settleMs < controlPeriodMs || config.postStopMs < controlPeriodMs ||
        config.postStopMs >= 0x80000000u)
        return "settleMs/postStopMs out of range";
    if (config.trialTimeoutMs <= config.settleMs || config.trialTimeoutMs >= 0x80000000u)
        return "trialTimeoutMs out of range";
    // Retain the audited timing fix: a return must fit BOTH settling intervals.
    if (config.returnTimeoutMs <= std::max(config.settleMs, config.postStopMs) ||
        config.returnTimeoutMs >= 0x80000000u)
        return "returnTimeoutMs must exceed settleMs and postStopMs";
    return nullptr;
}
} // namespace detail
} // namespace autotune
