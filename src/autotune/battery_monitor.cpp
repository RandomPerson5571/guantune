#include "session_internal.hpp"
#include <algorithm>
#include <cmath>

namespace autotune {
namespace detail {
namespace {
constexpr std::uint32_t allowedSagMs = 100;
constexpr double hardLowFraction = 0.85;
} // namespace

AutonTuneStatus restingBatteryStatus(double millivolts, const AutonTuneConfig& config) {
    if (!std::isfinite(millivolts) || millivolts <= 0)
        return AutonTuneStatus::SENSOR_FAULT;
    if (millivolts < config.minimumBatteryMillivolts)
        return AutonTuneStatus::LOW_BATTERY;
    return AutonTuneStatus::COMPLETE;
}

AutonTuneStatus BatteryMonitor::check(double millivolts, std::uint32_t deltaMs) {
    if (!std::isfinite(millivolts) || millivolts <= 0)
        return AutonTuneStatus::SENSOR_FAULT;
    if (millivolts >= minimumMillivolts_) {
        lowForMs_ = 0;
        return AutonTuneStatus::COMPLETE;
    }
    // Saturate at the deadline instead of overflowing after a large clock step.
    lowForMs_ += std::min(deltaMs, allowedSagMs - lowForMs_);
    if (lowForMs_ >= allowedSagMs || millivolts < minimumMillivolts_ * hardLowFraction)
        return AutonTuneStatus::LOW_BATTERY;
    return AutonTuneStatus::COMPLETE;
}
} // namespace detail
} // namespace autotune
