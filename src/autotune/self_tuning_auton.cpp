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
        c.integralWindupRange, c.maxKp, c.maxKd, c.maxKi, c.kiStep};
    for (double value : positive) if (!std::isfinite(value) || value <= 0) return false;
    return std::isfinite(c.headingKp) && c.headingKp >= 0 && c.maxOutput <= 127 &&
        c.maxTravelInches > c.distanceInches && c.toleranceInches < c.distanceInches &&
        c.crossingDeadband < c.distanceInches && c.maxKp >= 1.1 && c.maxKd >= 1.1 &&
        c.kiStep <= c.maxKi && c.maxTrials > 0 && c.confirmationTrials > 0 &&
        c.confirmationTrials <= c.maxTrials && c.integralAfterPdAdjustments > 0 &&
        c.integralUndershootTrials > 0 && c.settleMs >= periodMs &&
        c.postStopMs >= periodMs && c.postStopMs < 0x80000000u &&
        c.trialTimeoutMs > c.settleMs && c.trialTimeoutMs < 0x80000000u;
}
struct Stop {
    TuningDrive& drive;
    ~Stop() { drive.tank(0, 0); }
};
}

SelfTuningAuton::SelfTuningAuton(TuningDrive& drive, AutonTuneConfig config)
    : drive_(drive), config_(config) {}

void SelfTuningAuton::abort() { aborted_.store(true); }

double SelfTuningAuton::gainStep(double gain, bool fine) {
    if (fine || gain >= 12) return 0.1;
    return gain >= 6 ? 1.0 : 3.0;
}

AutonTuneStatus SelfTuningAuton::trial(const Gains& gains, TrialReport& report) {
    if (!drive_.available()) return AutonTuneStatus::DRIVE_UNAVAILABLE;
    Stop stop{drive_};
    const auto start = drive_.pose();
    if (!finitePose(start)) return AutonTuneStatus::SENSOR_FAULT;
    const double radians = start.headingDegrees * pi / 180;
    const double sx = std::sin(radians), sy = std::cos(radians);
    const auto begin = drive_.nowMs();
    auto previousTime = begin;
    auto previous = start;
    double previousError = config_.distanceInches, integral = 0;
    int side = 1;
    std::uint32_t settledFor = 0;
    bool observingStop = false;
    std::uint32_t stoppedAt = 0;
    while (true) {
        if (aborted_.load()) return AutonTuneStatus::ABORTED;
        if (!drive_.available()) return AutonTuneStatus::DRIVE_UNAVAILABLE;
        const auto now = drive_.nowMs();
        const auto elapsed = static_cast<std::uint32_t>(now - begin);
        const auto dt = static_cast<std::uint32_t>(now - previousTime);
        const auto pose = drive_.pose();
        if (!finitePose(pose)) return AutonTuneStatus::SENSOR_FAULT;
        const double dx = pose.x - start.x, dy = pose.y - start.y;
        const double distance = dx * sx + dy * sy;
        const double crossTrack = dx * sy - dy * sx;
        const double heading = headingError(pose.headingDegrees, start.headingDegrees);
        if (std::hypot(dx, dy) > config_.maxTravelInches ||
            std::abs(crossTrack) > config_.maxCrossTrackInches ||
            std::abs(heading) > config_.maxHeadingErrorDegrees)
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
        const double speed = dt ? std::hypot(pose.x - previous.x, pose.y - previous.y) * 1000 / dt : 0;
        report.finalSpeedInchesPerSecond = speed;
        if (dt && std::abs(error) <= config_.toleranceInches && speed <= config_.maxSettledSpeed)
            settledFor += dt;
        else settledFor = 0;
        if (observingStop) {
            report.settled = report.settled && std::abs(error) <= config_.toleranceInches &&
                speed <= config_.maxSettledSpeed;
            if (static_cast<std::uint32_t>(now - stoppedAt) >= config_.postStopMs)
                return AutonTuneStatus::COMPLETE; // Valid trial; may still have missed target.
        } else if (settledFor >= config_.settleMs || elapsed >= config_.trialTimeoutMs) {
            report.settled = settledFor >= config_.settleMs;
            report.timedOut = !report.settled;
            observingStop = true;
            stoppedAt = now;
            drive_.tank(0, 0);
        }
        // LemLib uses discrete integral sums and error differences at 10 ms.
        if (error * previousError < 0 || std::abs(error) > config_.integralWindupRange) integral = 0;
        if (std::abs(error) <= config_.integralWindupRange) integral += error;
        if (gains.kI > 0) integral = std::clamp(integral, -config_.maxOutput / gains.kI,
                                             config_.maxOutput / gains.kI);
        const double output = gains.kP * error + gains.kI * integral + gains.kD * (error - previousError);
        const double turn = config_.headingKp * heading;
        const double left = output + turn, right = output - turn;
        const double scale = std::max(1.0, std::max(std::abs(left), std::abs(right)) / config_.maxOutput);
        if (!std::isfinite(output) || !std::isfinite(turn)) return AutonTuneStatus::SENSOR_FAULT;
        if (!observingStop) drive_.tank(left / scale, right / scale);
        previousError = error;
        previousTime = now;
        previous = pose;
        drive_.delayMs(periodMs);
    }
}

AutonTuneResult SelfTuningAuton::run(const BeforeTrial& beforeTrial) {
    AutonTuneResult result;
    if (running_.test_and_set()) { result.status = AutonTuneStatus::DRIVE_UNAVAILABLE; return result; }
    struct Unlock { std::atomic_flag& flag; ~Unlock() { flag.clear(); } } unlock{running_};
    if (!valid(config_) || !beforeTrial) return result;
    aborted_.store(false);
    if (!drive_.available()) { result.status = AutonTuneStatus::DRIVE_UNAVAILABLE; return result; }
    Stop stop{drive_};
    Gains gains{1.1, 0, 1.1};
    unsigned confirmations = 0, pdAdjustments = 0, undershoots = 0;
    int previousAction = 0;
    bool fine = false;
    for (unsigned index = 0; index < config_.maxTrials; ++index) {
        drive_.tank(0, 0);
        if (aborted_.load() || !beforeTrial(index, gains) || aborted_.load()) {
            result.status = AutonTuneStatus::ABORTED; return result;
        }
        TrialReport report;
        report.gains = gains;
        const auto status = trial(gains, report);
        result.trials.push_back(report);
        result.gains = gains;
        if (status != AutonTuneStatus::COMPLETE) { result.status = status; return result; }
        const bool overshoots = report.overshootInches > config_.toleranceInches;
        const bool oscillates = report.crossings >= 3;
        if (report.settled && !overshoots && !oscillates) {
            undershoots = 0;
            if (++confirmations >= config_.confirmationTrials) {
                result.status = AutonTuneStatus::COMPLETE; return result;
            }
            continue; // Confirm the same gains on another trial before accepting.
        }
        confirmations = 0;
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
        if (previousAction && action * previousAction < 0) fine = true;
        previousAction = action;
        if (std::abs(action) == 1) ++pdAdjustments;
        if (gains.kP == report.gains.kP && gains.kD == report.gains.kD && gains.kI == report.gains.kI) {
            result.status = AutonTuneStatus::GAIN_LIMIT; return result;
        }
    }
    result.status = AutonTuneStatus::TRIAL_LIMIT;
    return result;
}

} // namespace autotune
