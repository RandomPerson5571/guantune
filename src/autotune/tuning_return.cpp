#include "session_internal.hpp"
#include <algorithm>
#include <cmath>

namespace autotune {
namespace {
enum class ReturnPhase { Ready, Reversing, Coasting };
constexpr double stopLeadSeconds = 2.0 * detail::controlPeriodMs / 1000.0;
} // namespace

AutonTuneStatus SelfTuningAuton::returnToStart(detail::MotionTracker& motion,
                                               detail::TelemetryRecorder& telemetry) {
    auto status = checkCancellation();
    if (status != AutonTuneStatus::COMPLETE)
        return status;
    detail::DriveCleanup cleanup{drive_};
    const auto beginMs = drive_.nowMs();
    detail::ClockWatchdog clock(beginMs);
    detail::BatteryMonitor batteryMonitor(config_);
    std::uint32_t settledForMs = 0;
    auto phase = ReturnPhase::Ready;
    const bool angular = config_.mode == Mode::ANGULAR;

    // Use measured speed to stop two samples early, then observe coasting. If
    // the robot stops short, creep again within the same bounded return deadline.
    // The session tracker is never rebased here: phase-boundary motion counts.
    while (true) {
        status = checkCancellation();
        if (status != AutonTuneStatus::COMPLETE)
            return status;
        const auto nowMs = drive_.nowMs();
        if (!clock.healthy(nowMs))
            return AutonTuneStatus::SENSOR_FAULT;
        const auto pose = drive_.pose();
        if (!detail::finitePose(pose))
            return AutonTuneStatus::SENSOR_FAULT;
        const auto sample = motion.sample(pose, nowMs);
        const auto elapsedMs = static_cast<std::uint32_t>(nowMs - beginMs);
        const double distance = angular ? std::abs(sample.position) : sample.translationFromOrigin;
        const double battery = drive_.batteryMillivolts();
        telemetry.record(sample,
                         phase == ReturnPhase::Coasting ? TelemetryPhase::RETURN_SETTLE
                                                        : TelemetryPhase::RETURN,
                         0, battery,
                         phase == ReturnPhase::Reversing ? detail::returnCommand(config_)
                                                         : detail::stoppedCommand());
        status = batteryMonitor.check(battery, sample.deltaMs);
        if (status != AutonTuneStatus::COMPLETE)
            return status;
        status = checkIndependentDisplacement(sample.position);
        if (status != AutonTuneStatus::COMPLETE)
            return status;
        status = detail::motionStatus(sample, config_);
        if (status != AutonTuneStatus::COMPLETE)
            return status;

        const bool nearOrigin =
            distance <= config_.returnToleranceInches &&
            (!angular || std::abs(sample.headingFromOrigin) <= config_.returnToleranceInches);
        if (phase != ReturnPhase::Coasting) {
            if (distance <= config_.returnToleranceInches + sample.speed * stopLeadSeconds) {
                drive_.stop();
                phase = ReturnPhase::Coasting;
            } else if (sample.position <= config_.returnToleranceInches) {
                return AutonTuneStatus::RETURN_FAILED; // Past the origin or offset sideways.
            } else if (phase == ReturnPhase::Ready) {
                if (angular)
                    drive_.turnBack(config_.returnPower);
                else
                    drive_.driveBackward(config_.returnPower);
                phase = ReturnPhase::Reversing;
            }
        }
        if (phase == ReturnPhase::Coasting && !nearOrigin) {
            if (sample.position <= config_.returnToleranceInches)
                return AutonTuneStatus::RETURN_FAILED;
            if (sample.speed <= config_.maxSettledSpeed) {
                phase = ReturnPhase::Ready;
                settledForMs = 0;
            }
        }
        if (phase == ReturnPhase::Coasting && sample.deltaMs && nearOrigin &&
            sample.speed <= config_.maxSettledSpeed)
            settledForMs += sample.deltaMs;
        else
            settledForMs = 0;
        if (settledForMs >= std::max(config_.settleMs, config_.postStopMs))
            return AutonTuneStatus::COMPLETE;
        if (elapsedMs >= config_.returnTimeoutMs)
            return AutonTuneStatus::RETURN_FAILED;
        drive_.delayMs(detail::controlPeriodMs);
    }
}
} // namespace autotune
