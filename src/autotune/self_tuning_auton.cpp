#include "session_internal.hpp"
#include <limits>
#include <utility>

namespace autotune {
SelfTuningAuton::SelfTuningAuton(TuningDrive& drive, AutonTuneConfig config)
    : drive_(drive), config_(std::move(config)) {
    if (config_.range)
        config_.toleranceInches = *config_.range;
}

void SelfTuningAuton::abort() { aborted_.store(true); }

AutonTuneResult SelfTuningAuton::run(const BeforeTrial& beforeTrial) {
    AutonTuneResult result;
    result.mode = config_.mode;
    result.integralWindupRange = config_.integralWindupRange;
    // Rejected callers must not consume the active run's abort request or touch
    // the drive. Only the caller that acquired this guard may release it.
    if (running_.test_and_set()) {
        result.status = AutonTuneStatus::ALREADY_RUNNING;
        return result;
    }
    struct RunGuard {
        std::atomic<bool>& abortFlag;
        std::atomic_flag& running;
        ~RunGuard() {
            abortFlag.store(false);
            running.clear();
        }
    } guard{aborted_, running_};

    if (detail::validationError(config_))
        return result;
    result.status = checkCancellation();
    if (result.status != AutonTuneStatus::COMPLETE)
        return result;
    // Refuse unrelated active motion before acquiring cleanup ownership.
    if (drive_.isMoving()) {
        result.status = AutonTuneStatus::DRIVE_UNAVAILABLE;
        return result;
    }
    detail::DriveCleanup cleanup{drive_};
    if (config_.mode == Mode::ANGULAR && !drive_.supportsAngular()) {
        result.status = AutonTuneStatus::UNSUPPORTED_MODE;
        return result;
    }
    drive_.selectMode(config_.mode);
    result.status = checkReadiness();
    if (result.status != AutonTuneStatus::COMPLETE)
        return result;
    if (config_.requireConfirmation && !config_.requireConfirmation()) {
        result.status = AutonTuneStatus::ABORTED;
        return result;
    }

    const auto targets = config_.testDistances.empty() ? std::vector<double>{config_.distanceInches}
                                                       : config_.testDistances;
    std::size_t targetIndex = 0;
    std::size_t successfulTrials = 0;
    const auto requiredSuccesses = config_.confirmationTrials * targets.size();
    double bestScore = std::numeric_limits<double>::infinity();
    Gains candidate{detail::initialKp, 0, detail::initialKd};
    detail::GainSearch search(config_);
    detail::TelemetryRecorder telemetry(drive_, config_, result);
    std::optional<detail::MotionTracker> motion;

    // Every exit after a trial uses the same recommendation rule. Only safely
    // returned trials can be "best"; none completed means the starting gains.
    const auto finish = [&](AutonTuneStatus status) {
        result.status = status;
        if (status == AutonTuneStatus::COMPLETE)
            result.gains = candidate;
        else if (result.bestTrial)
            result.gains = result.trials[*result.bestTrial].gains;
        else
            result.gains = {detail::initialKp, 0, detail::initialKd};
    };

    for (unsigned index = 0; index < config_.maxTrials; ++index) {
        telemetry.beginTrial(index);
        if (aborted_.load() || (beforeTrial && !beforeTrial(index, candidate)) || aborted_.load()) {
            finish(AutonTuneStatus::ABORTED);
            return result;
        }
        auto status = waitForIdle();
        if (status != AutonTuneStatus::COMPLETE) {
            finish(status);
            return result;
        }
        if (!motion) {
            const auto origin = drive_.pose();
            if (!detail::finitePose(origin)) {
                finish(AutonTuneStatus::SENSOR_FAULT);
                return result;
            }
            motion.emplace(origin, config_.mode, drive_.nowMs());
        }

        TrialReport report;
        report.gains = candidate;
        status = runTrial(candidate, targets[targetIndex], *motion, report, telemetry);
        result.trials.push_back(report);
        if (status != AutonTuneStatus::COMPLETE) {
            finish(status);
            return result;
        }
        // A measurement alone is not a complete trial. Return even after the
        // final confirmation or the last trial allowed by the budget.
        status = returnToStart(*motion, telemetry);
        if (status != AutonTuneStatus::COMPLETE) {
            finish(status);
            return result;
        }
        result.trials.back().returnedToStart = true;
        const double score = detail::trialScore(report, config_);
        if (score < bestScore) {
            bestScore = score;
            result.bestTrial = index;
        }

        if (detail::trialPassed(report, config_)) {
            search.recordSuccess();
            targetIndex = (targetIndex + 1) % targets.size();
            if (++successfulTrials >= requiredSuccesses) {
                finish(AutonTuneStatus::COMPLETE);
                return result;
            }
            continue;
        }
        successfulTrials = 0;
        targetIndex = 0;
        // Keep the audited fix: a policy cannot override the final outcome
        // with a candidate for which there is no remaining test budget.
        if (index == config_.maxTrials - 1)
            break;
        const auto decision = search.nextCandidate(report, index);
        if (decision.action == detail::SearchAction::GainLimit) {
            finish(AutonTuneStatus::GAIN_LIMIT);
            return result;
        }
        if (decision.action == detail::SearchAction::RetryLimit)
            break;
        candidate = decision.gains; // RetryCurrent deliberately keeps these gains.
    }
    finish(AutonTuneStatus::TRIAL_LIMIT);
    return result;
}
} // namespace autotune
