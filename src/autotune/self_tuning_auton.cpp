#include "autotune/self_tuning_auton.hpp"
#include <algorithm>
#include <cmath>

namespace autotune {
namespace {
constexpr std::uint32_t periodMs = 10;
constexpr double pi = 3.14159265358979323846;
double headingError(double current, double start) {
    return std::remainder(start - current, 360.0);
}
bool finitePose(const DrivePose& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.headingDegrees);
}
bool valid(const AutonTuneConfig& c) {
    const double positive[] = {c.distanceInches, c.maxOutput, c.toleranceInches,
        c.maxSettledSpeed, c.crossingDeadband, c.maxTravelInches,
        c.maxCrossTrackInches, c.maxHeadingErrorDegrees, c.maxSensorSpeed,
        c.returnToleranceInches, c.returnPower, c.integralWindupRange, c.maxKp, c.maxKd, c.maxKi, c.kiStep};
    for (double value : positive) if (!std::isfinite(value) || value <= 0) return false;
    if (!std::isfinite(c.minimumBatteryMillivolts) || c.minimumBatteryMillivolts < 0 ||
        !std::isfinite(c.maxMeasurementDisagreement) || c.maxMeasurementDisagreement <= 0) return false;
    if (c.mode != Mode::LATERAL && c.mode != Mode::ANGULAR) return false;
    for (double distance : c.testDistances)
        if (!std::isfinite(distance) || distance <= c.toleranceInches ||
            distance <= c.crossingDeadband || distance <= c.returnToleranceInches ||
            distance >= c.maxTravelInches || (c.mode == Mode::ANGULAR && distance > 180)) return false;
    if (c.mode == Mode::ANGULAR && c.distanceInches > 180) return false;
    return c.maxOutput >= 60 && c.maxOutput <= 127 && c.returnPower >= 1 && c.returnPower <= 40 &&
        c.maxTravelInches > c.distanceInches && c.toleranceInches < c.distanceInches &&
        c.crossingDeadband < c.distanceInches && c.maxKp >= 1.1 && c.maxKd >= 1.1 &&
        c.kiStep <= c.maxKi && c.maxTrials > 0 && c.confirmationTrials > 0 &&
        c.confirmationTrials <= c.maxTrials / std::max<std::size_t>(1, c.testDistances.size()) && c.integralAfterPdAdjustments > 0 &&
        c.integralUndershootTrials > 0 && c.settleMs >= periodMs &&
        c.postStopMs >= periodMs && c.postStopMs < 0x80000000u &&
        c.trialTimeoutMs > c.settleMs && c.trialTimeoutMs < 0x80000000u &&
        c.returnTimeoutMs > c.settleMs && c.returnTimeoutMs < 0x80000000u &&
        c.returnToleranceInches < c.distanceInches;
}
struct Stop {
    TuningDrive& drive;
    ~Stop() { drive.stop(); drive.restoreGains(); }
};
}

SelfTuningAuton::SelfTuningAuton(TuningDrive& drive, AutonTuneConfig config)
    : drive_(drive), config_(config) {
    if (config_.range) config_.toleranceInches = *config_.range;
}

void SelfTuningAuton::abort() { aborted_.store(true); }

double SelfTuningAuton::gainStep(double gain, bool fine) {
    if (fine || gain >= 12) return 0.1;
    return gain >= 6 ? 1.0 : 3.0;
}

AutonTuneStatus SelfTuningAuton::trial(const Gains& gains, const DrivePose& start, TrialReport& report) {
    if (!drive_.available()) return AutonTuneStatus::DRIVE_UNAVAILABLE;
    Stop stop{drive_};
    if (!finitePose(start)) return AutonTuneStatus::SENSOR_FAULT;
    const double radians = start.headingDegrees * pi / 180;
    const double sx = std::sin(radians), sy = std::cos(radians);
    auto previous = drive_.pose();
    if (!finitePose(previous)) return AutonTuneStatus::SENSOR_FAULT;
    const auto begin = drive_.nowMs();
    auto previousTime = begin;
    int side = 1;
    std::uint32_t settledFor = 0;
    bool observingStop = false;
    std::uint32_t stoppedAt = 0;
    report.target = config_.distanceInches;
    if (!drive_.calibrated()) return AutonTuneStatus::NOT_CALIBRATED;
    report.batteryStartMillivolts = report.batteryMinimumMillivolts = drive_.batteryMillivolts();
    if (!std::isfinite(report.batteryStartMillivolts) || report.batteryStartMillivolts <= 0)
        return AutonTuneStatus::SENSOR_FAULT;
    if (report.batteryStartMillivolts < config_.minimumBatteryMillivolts) return AutonTuneStatus::LOW_BATTERY;
    drive_.setTrialGains(gains, config_.integralWindupRange);
    DrivePose target = start;
    target.x += config_.distanceInches * sx;
    target.y += config_.distanceInches * sy;
    if (config_.mode == Mode::ANGULAR)
        drive_.turnTo(start.headingDegrees + config_.distanceInches, config_.trialTimeoutMs, config_.maxOutput);
    else drive_.moveTo(target, true, config_.trialTimeoutMs, config_.maxOutput);
    double angularDistance = 0;
    double previousHeading = start.headingDegrees;
    while (true) {
        if (aborted_.load()) return AutonTuneStatus::ABORTED;
        if (!drive_.available()) return AutonTuneStatus::DRIVE_UNAVAILABLE;
        const auto now = drive_.nowMs();
        const auto elapsed = static_cast<std::uint32_t>(now - begin);
        const auto dt = static_cast<std::uint32_t>(now - previousTime);
        const auto pose = drive_.pose();
        if (!finitePose(pose)) return AutonTuneStatus::SENSOR_FAULT;
        const double dx = pose.x - start.x, dy = pose.y - start.y;
        const double battery = drive_.batteryMillivolts();
        if (!std::isfinite(battery) || battery <= 0) return AutonTuneStatus::SENSOR_FAULT;
        report.batteryMinimumMillivolts = std::min(report.batteryMinimumMillivolts, battery);
        if (battery < config_.minimumBatteryMillivolts) return AutonTuneStatus::LOW_BATTERY;
        angularDistance += std::remainder(pose.headingDegrees - previousHeading, 360.0);
        previousHeading = pose.headingDegrees;
        const bool angular = config_.mode == Mode::ANGULAR;
        const double distance = angular ? angularDistance : dx * sx + dy * sy;
        if (config_.independentDisplacement) {
            const double independent = config_.independentDisplacement();
            report.independentDistance = independent;
            if (!std::isfinite(independent)) return AutonTuneStatus::SENSOR_FAULT;
            if (std::abs(independent - distance) > config_.maxMeasurementDisagreement)
                return AutonTuneStatus::MEASUREMENT_DISAGREEMENT;
        }
        const double crossTrack = dx * sy - dy * sx;
        const double heading = headingError(pose.headingDegrees, start.headingDegrees);
        if (std::hypot(dx, dy) > (angular ? config_.maxCrossTrackInches : config_.maxTravelInches) ||
            std::abs(crossTrack) > config_.maxCrossTrackInches ||
            (angular ? std::abs(distance) > config_.maxTravelInches :
             std::abs(heading) > config_.maxHeadingErrorDegrees))
            return AutonTuneStatus::TRAVEL_LIMIT;
        if (dt > 0 && std::hypot(pose.x - previous.x, pose.y - previous.y) * 1000 / dt >
            config_.maxSensorSpeed) return AutonTuneStatus::SENSOR_FAULT;
        const double error = config_.distanceInches - distance;
        const int nextSide = error > config_.crossingDeadband ? 1 :
            error < -config_.crossingDeadband ? -1 : side;
        if (nextSide != side) ++report.crossings;
        side = nextSide;
        report.distanceInches = distance;
        report.finalErrorInches = error;
        report.overshootInches = std::max(report.overshootInches, -error);
        report.elapsedMs = elapsed;
        const double delta = angular ? std::abs(std::remainder(pose.headingDegrees - previous.headingDegrees, 360.0)) :
            std::hypot(pose.x - previous.x, pose.y - previous.y);
        const double speed = dt ? delta * 1000 / dt : 0;
        if (speed > config_.maxSensorSpeed) return AutonTuneStatus::SENSOR_FAULT;
        report.finalSpeedInchesPerSecond = speed;
        if (dt && std::abs(error) <= config_.toleranceInches && speed <= config_.maxSettledSpeed)
            settledFor += dt;
        else settledFor = 0;
        if (observingStop) {
            report.settled = settledFor >= config_.settleMs;
            if (static_cast<std::uint32_t>(now - stoppedAt) >= std::max(config_.postStopMs, config_.settleMs))
                return AutonTuneStatus::COMPLETE; // Valid trial; may still have missed target.
        } else if (!drive_.isMoving() || settledFor >= config_.settleMs || elapsed >= config_.trialTimeoutMs) {
            report.settled = settledFor >= config_.settleMs;
            report.timedOut = elapsed >= config_.trialTimeoutMs && !report.settled;
            observingStop = true;
            stoppedAt = now;
            drive_.stop();
        }
        previousTime = now;
        previous = pose;
        drive_.delayMs(periodMs);
    }
}

AutonTuneStatus SelfTuningAuton::returnToStart(const DrivePose& start, double displacement) {
    if (aborted_.load()) return AutonTuneStatus::ABORTED;
    if (!drive_.available()) return AutonTuneStatus::DRIVE_UNAVAILABLE;
    Stop stop{drive_};
    auto previous = drive_.pose();
    if (!finitePose(previous)) return AutonTuneStatus::SENSOR_FAULT;
    const auto begin = drive_.nowMs();
    auto previousTime = begin;
    std::uint32_t settledFor = 0;
    bool reversing = false;
    bool stopped = false;
    // Back up at fixed low power. Chassis pose, rather than moveToPoint's exit
    // conditions or the candidate PID, determines when the return stops.
    while (true) {
        if (aborted_.load()) return AutonTuneStatus::ABORTED;
        if (!drive_.available()) return AutonTuneStatus::DRIVE_UNAVAILABLE;
        const auto now = drive_.nowMs();
        const auto dt = static_cast<std::uint32_t>(now - previousTime);
        const auto elapsed = static_cast<std::uint32_t>(now - begin);
        const auto p = drive_.pose();
        if (!finitePose(p)) return AutonTuneStatus::SENSOR_FAULT;
        const bool angular = config_.mode == Mode::ANGULAR;
        displacement += std::remainder(p.headingDegrees - previous.headingDegrees, 360.0);
        const double angle = displacement;
        const double distance = angular ? std::abs(angle) : std::hypot(p.x - start.x, p.y - start.y);
        const double battery = drive_.batteryMillivolts();
        if (!std::isfinite(battery) || battery <= 0) return AutonTuneStatus::SENSOR_FAULT;
        if (battery < config_.minimumBatteryMillivolts) return AutonTuneStatus::LOW_BATTERY;
        const double radians = start.headingDegrees * pi / 180;
        const double remaining = angular ? angle : (p.x - start.x) * std::sin(radians) +
                                 (p.y - start.y) * std::cos(radians);
        const double crossTrack = (p.x - start.x) * std::cos(radians) -
                                  (p.y - start.y) * std::sin(radians);
        const double delta = angular ? std::abs(std::remainder(p.headingDegrees - previous.headingDegrees, 360.0)) :
            std::hypot(p.x - previous.x, p.y - previous.y);
        const double speed = dt ? delta * 1000 / dt : 0;
        if (speed > config_.maxSensorSpeed) return AutonTuneStatus::SENSOR_FAULT;
        if ((angular && std::hypot(p.x - start.x, p.y - start.y) > config_.maxCrossTrackInches) ||
            distance > config_.maxTravelInches || std::abs(crossTrack) > config_.maxCrossTrackInches ||
            (!angular && std::abs(headingError(p.headingDegrees, start.headingDegrees)) > config_.maxHeadingErrorDegrees))
            return AutonTuneStatus::TRAVEL_LIMIT;
        if (!stopped) {
            if (distance <= config_.returnToleranceInches) {
                drive_.stop();
                stopped = true;
            } else if (remaining <= config_.returnToleranceInches) {
                // Past the start or offset sideways: further reversing cannot
                // reliably reach the start. Stop instead of driving away.
                return AutonTuneStatus::RETURN_FAILED;
            } else if (!reversing) {
                if (angular) drive_.turnBack(config_.returnPower);
                else drive_.driveBackward(config_.returnPower);
                reversing = true;
            }
        }
        if (stopped && distance > config_.returnToleranceInches)
            return AutonTuneStatus::RETURN_FAILED; // Coasting carried us outside the range.
        if (stopped && dt && distance <= config_.returnToleranceInches &&
            speed <= config_.maxSettledSpeed) settledFor += dt;
        else settledFor = 0;
        if (settledFor >= std::max(config_.settleMs, config_.postStopMs)) return AutonTuneStatus::COMPLETE;
        if (elapsed >= config_.returnTimeoutMs) return AutonTuneStatus::RETURN_FAILED;
        previous = p;
        previousTime = now;
        drive_.delayMs(periodMs);
    }
}

AutonTuneResult SelfTuningAuton::run(const BeforeTrial& beforeTrial) {
    AutonTuneResult result;
    result.mode = config_.mode;
    if (running_.test_and_set()) { result.status = AutonTuneStatus::DRIVE_UNAVAILABLE; return result; }
    struct Unlock { std::atomic_flag& flag; ~Unlock() { flag.clear(); } } unlock{running_};
    if (!valid(config_)) return result;
    aborted_.store(false);
    if (!drive_.available() || drive_.isMoving()) { result.status = AutonTuneStatus::DRIVE_UNAVAILABLE; return result; }
    Stop stop{drive_};
    if (config_.mode == Mode::ANGULAR && !drive_.supportsAngular()) {
        result.status = AutonTuneStatus::UNSUPPORTED_MODE; return result;
    }
    drive_.selectMode(config_.mode);
    if (!drive_.calibrated()) { result.status = AutonTuneStatus::NOT_CALIBRATED; return result; }
    if (!finitePose(drive_.pose())) { result.status = AutonTuneStatus::SENSOR_FAULT; return result; }
    const double battery = drive_.batteryMillivolts();
    if (!std::isfinite(battery) || battery <= 0) { result.status = AutonTuneStatus::SENSOR_FAULT; return result; }
    if (battery < config_.minimumBatteryMillivolts) { result.status = AutonTuneStatus::LOW_BATTERY; return result; }
    if (config_.requireConfirmation && !config_.requireConfirmation()) {
        result.status = AutonTuneStatus::ABORTED; return result;
    }
    Gains gains{1.1, 0, 1.1};
    const auto distances = config_.testDistances.empty() ? std::vector<double>{config_.distanceInches} : config_.testDistances;
    struct RestoreDistance { double& value; double original; ~RestoreDistance() { value = original; } }
        restoreDistance{config_.distanceInches, config_.distanceInches};
    std::size_t targetIndex = 0;
    double bestScore = INFINITY;
    Gains previousGains = gains;
    unsigned confirmations = 0, pdAdjustments = 0, undershoots = 0;
    int previousAction = 0;
    bool fine = false;
    std::optional<DrivePose> origin;
    for (unsigned index = 0; index < config_.maxTrials; ++index) {
        if (aborted_.load() || (beforeTrial && !beforeTrial(index, gains)) || aborted_.load()) {
            result.status = AutonTuneStatus::ABORTED; return result;
        }
        if (!drive_.available() || drive_.isMoving()) {
            result.status = AutonTuneStatus::DRIVE_UNAVAILABLE; return result;
        }
        if (!origin) origin = drive_.pose();
        TrialReport report;
        report.gains = gains;
        config_.distanceInches = distances[targetIndex];
        const auto status = trial(gains, *origin, report);
        result.trials.push_back(report);
        result.gains = gains;
        if (status != AutonTuneStatus::COMPLETE) { result.status = status; return result; }
        // Return after every valid trial, including the final confirmation/limit
        // trial. Preserve the forward measurements for tuning decisions.
        const auto returned = returnToStart(*origin, report.distanceInches);
        if (returned != AutonTuneStatus::COMPLETE) { result.status = returned; return result; }
        result.trials.back().returnedToStart = true;
        const double score = std::abs(report.finalErrorInches) + report.overshootInches + report.crossings * config_.toleranceInches;
        if (score < bestScore) { bestScore = score; result.bestTrial = index; }
        const bool overshoots = report.overshootInches > config_.toleranceInches;
        const bool oscillates = report.crossings >= 3;
        if (report.settled && !overshoots && !oscillates) {
            undershoots = 0;
            targetIndex = (targetIndex + 1) % distances.size();
            if (++confirmations >= config_.confirmationTrials * distances.size()) {
                result.status = AutonTuneStatus::COMPLETE; return result;
            }
            continue; // Confirm the same gains on another trial before accepting.
        }
        confirmations = 0;
        targetIndex = 0;
        const Gains testedGains = gains;
        int action = 0;
        if (overshoots || oscillates) {
            if (previousAction > 0) fine = true;
            undershoots = 0;
            if (gains.kI > 0) { gains.kI = std::max(0.0, gains.kI - config_.kiStep); action = -2; }
            else { gains.kD = std::min(config_.maxKd, gains.kD + gainStep(gains.kD, fine)); action = -1; }
        } else {
            if (previousAction < 0) fine = true;
            ++undershoots;
            if (config_.allowIntegral && pdAdjustments >= config_.integralAfterPdAdjustments &&
                undershoots >= config_.integralUndershootTrials && gains.kP >= 12 &&
                report.finalErrorInches > config_.toleranceInches &&
                report.finalErrorInches <= config_.integralWindupRange && report.crossings == 0 &&
                report.finalSpeedInchesPerSecond <= config_.maxSettledSpeed) {
                gains.kI = std::min(config_.maxKi, gains.kI + config_.kiStep); action = 2;
            } else { gains.kP = std::min(config_.maxKp, gains.kP + gainStep(gains.kP, fine)); action = 1; }
        }
        if (previousAction && action * previousAction < 0) {
            fine = true;
            if (config_.bisectOnReversal) {
                gains = {(testedGains.kP + previousGains.kP) / 2,
                         (testedGains.kI + previousGains.kI) / 2,
                         (testedGains.kD + previousGains.kD) / 2};
            }
        }
        if (config_.adjustmentPolicy)
            gains = config_.adjustmentPolicy(testedGains, report.finalErrorInches, report.overshootInches, index);
        if (!std::isfinite(gains.kP) || !std::isfinite(gains.kI) || !std::isfinite(gains.kD) ||
            gains.kP < 0 || gains.kP > config_.maxKp || gains.kI < 0 || gains.kI > config_.maxKi ||
            gains.kD < 0 || gains.kD > config_.maxKd) {
            result.status = AutonTuneStatus::GAIN_LIMIT; return result;
        }
        previousGains = testedGains;
        previousAction = action;
        if (std::abs(action) == 1) ++pdAdjustments;
        if (gains.kP == report.gains.kP && gains.kD == report.gains.kD && gains.kI == report.gains.kI) {
            result.status = AutonTuneStatus::GAIN_LIMIT; return result;
        }
    }
    if (result.bestTrial) result.gains = result.trials[*result.bestTrial].gains;
    result.status = AutonTuneStatus::TRIAL_LIMIT;
    return result;
}

} // namespace autotune
