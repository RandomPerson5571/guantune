#include "autotune/angle.hpp"
#include "session_internal.hpp"
#include <algorithm>
#include <cmath>

namespace autotune {
AutonTuneStatus SelfTuningAuton::runTrial(const Gains& gains, double target,
                                          detail::MotionTracker& motion, TrialReport& report,
                                          detail::TelemetryRecorder& telemetry) {
    auto status = checkCancellation();
    if (status != AutonTuneStatus::COMPLETE)
        return status;
    detail::DriveCleanup cleanup{drive_};
    if (!drive_.calibrated())
        return AutonTuneStatus::NOT_CALIBRATED;

    report.target = target;
    report.batteryStartMillivolts = report.batteryMinimumMillivolts = drive_.batteryMillivolts();
    status = detail::restingBatteryStatus(report.batteryStartMillivolts, config_);
    if (status != AutonTuneStatus::COMPLETE)
        return status;
    detail::BatteryMonitor batteryMonitor(config_);
    const auto beginMs = drive_.nowMs();
    detail::ClockWatchdog clock(beginMs);
    const auto initialPose = drive_.pose();
    if (!detail::finitePose(initialPose))
        return AutonTuneStatus::SENSOR_FAULT;
    const auto initialSample = motion.sample(initialPose, beginMs);
    status = detail::motionStatus(initialSample, config_);
    if (status != AutonTuneStatus::COMPLETE)
        return status;
    const double initialPosition = initialSample.position;
    const bool angular = config_.mode == Mode::ANGULAR;
    const auto& origin = motion.origin();
    drive_.setTrialGains(gains, config_.integralWindupRange);
    if (angular) {
        // Normalize only the absolute command, never the measured displacement.
        const double heading = detail::normalizeHeading(origin.headingDegrees + target);
        drive_.turnTo(heading, config_.trialTimeoutMs, config_.maxOutput);
    } else {
        constexpr double pi = 3.14159265358979323846;
        const double radians = origin.headingDegrees * pi / 180;
        const DrivePose destination{origin.x + target * std::sin(radians),
                                    origin.y + target * std::cos(radians), origin.headingDegrees};
        drive_.moveTo(destination, true, config_.trialTimeoutMs, config_.maxOutput);
    }

    // The motion phase is followed by an observation phase with zero output.
    // A valid trial may miss its target: COMPLETE here means measurements are
    // usable. The session runner decides whether the gains passed confirmation.
    int previousSide = 1;
    std::uint32_t settledForMs = 0;
    std::optional<std::uint32_t> stoppedAtMs;
    bool motionObserved = false;
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
        const double battery = drive_.batteryMillivolts();
        telemetry.record(sample, stoppedAtMs ? TelemetryPhase::COAST : TelemetryPhase::TRIAL,
                         target, battery,
                         stoppedAtMs ? detail::stoppedCommand() : TelemetryCommand{});
        status = batteryMonitor.check(battery, sample.deltaMs);
        if (std::isfinite(battery) && battery > 0)
            report.batteryMinimumMillivolts = std::min(report.batteryMinimumMillivolts, battery);
        if (status != AutonTuneStatus::COMPLETE)
            return status;
        status = checkIndependentDisplacement(sample.position, &report.independentDistance);
        if (status != AutonTuneStatus::COMPLETE)
            return status;
        status = detail::motionStatus(sample, config_);
        if (status != AutonTuneStatus::COMPLETE)
            return status;

        const double error = target - sample.position;
        const int side = error > config_.crossingDeadband    ? 1
                         : error < -config_.crossingDeadband ? -1
                                                             : previousSide;
        if (side != previousSide)
            ++report.crossings;
        previousSide = side;
        report.distanceInches = sample.position;
        report.finalErrorInches = error;
        report.overshootInches = std::max(report.overshootInches, -error);
        report.elapsedMs = elapsedMs;
        report.finalSpeedInchesPerSecond = sample.speed;

        const bool moving = drive_.isMoving();
        // Evidence of startup must be relative to THIS trial. A valid return
        // can leave an offset from the session origin while a new motion waits
        // in the queue. That offset is not evidence the new motion has started.
        motionObserved = motionObserved || moving ||
                         std::abs(sample.position - initialPosition) >=
                             std::max(config_.toleranceInches, config_.crossingDeadband);
        if (!stoppedAtMs && !motionObserved && elapsedMs >= detail::motionStartupTimeoutMs)
            return AutonTuneStatus::DRIVE_UNAVAILABLE;

        if (sample.deltaMs && std::abs(error) <= config_.toleranceInches &&
            sample.speed <= config_.maxSettledSpeed)
            settledForMs += sample.deltaMs;
        else
            settledForMs = 0;

        if (stoppedAtMs) {
            report.settled = settledForMs >= config_.settleMs;
            if (static_cast<std::uint32_t>(nowMs - *stoppedAtMs) >=
                std::max(config_.postStopMs, config_.settleMs))
                return AutonTuneStatus::COMPLETE;
        } else if ((!moving && motionObserved) || settledForMs >= config_.settleMs ||
                   elapsedMs >= config_.trialTimeoutMs) {
            report.settled = settledForMs >= config_.settleMs;
            report.timedOut = elapsedMs >= config_.trialTimeoutMs && !report.settled;
            stoppedAtMs = nowMs;
            drive_.stop();
        }
        drive_.delayMs(detail::controlPeriodMs);
    }
}

} // namespace autotune
