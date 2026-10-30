#include "session_internal.hpp"
#include <algorithm>
#include <cmath>

namespace autotune {
namespace detail {
bool ClockWatchdog::healthy(std::uint32_t nowMs) {
    constexpr unsigned maxStalledPolls = 1000;
    if (nowMs != lastMs_) {
        lastMs_ = nowMs;
        stalledPolls_ = 0;
        return true;
    }
    return ++stalledPolls_ < maxStalledPolls;
}
} // namespace detail

AutonTuneStatus SelfTuningAuton::checkCancellation() {
    if (aborted_.load())
        return AutonTuneStatus::ABORTED;
    if (!drive_.available())
        return AutonTuneStatus::DRIVE_UNAVAILABLE;
    return AutonTuneStatus::COMPLETE;
}

AutonTuneStatus SelfTuningAuton::checkReadiness() {
    const auto status = checkCancellation();
    if (status != AutonTuneStatus::COMPLETE)
        return status;
    if (drive_.isMoving())
        return AutonTuneStatus::DRIVE_UNAVAILABLE;
    if (!drive_.calibrated())
        return AutonTuneStatus::NOT_CALIBRATED;
    if (!detail::finitePose(drive_.pose()))
        return AutonTuneStatus::SENSOR_FAULT;
    return detail::restingBatteryStatus(drive_.batteryMillivolts(), config_);
}

AutonTuneStatus SelfTuningAuton::waitForIdle() {
    const auto beginMs = drive_.nowMs();
    detail::ClockWatchdog clock(beginMs);
    const auto timeoutMs = std::max<std::uint32_t>(config_.postStopMs, 100);
    while (drive_.isMoving()) {
        const auto status = checkCancellation();
        if (status != AutonTuneStatus::COMPLETE)
            return status;
        const auto nowMs = drive_.nowMs();
        if (static_cast<std::uint32_t>(nowMs - beginMs) >= timeoutMs || !clock.healthy(nowMs))
            return AutonTuneStatus::DRIVE_UNAVAILABLE;
        drive_.delayMs(detail::controlPeriodMs);
    }
    return checkCancellation();
}

AutonTuneStatus SelfTuningAuton::checkIndependentDisplacement(double displacement,
                                                              std::optional<double>* reading) {
    if (!config_.independentDisplacement)
        return AutonTuneStatus::COMPLETE;
    const double independent = config_.independentDisplacement();
    if (reading)
        *reading = independent;
    if (!std::isfinite(independent))
        return AutonTuneStatus::SENSOR_FAULT;
    if (std::abs(independent - displacement) > config_.maxMeasurementDisagreement)
        return AutonTuneStatus::MEASUREMENT_DISAGREEMENT;
    return AutonTuneStatus::COMPLETE;
}
} // namespace autotune
