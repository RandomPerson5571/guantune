#include "autotune/pid_validation.hpp"
#include "session_internal.hpp"
#include <algorithm>
#include <cmath>

namespace autotune {
namespace detail {
bool trialPassed(const TrialReport& report, const AutonTuneConfig& config) {
    return report.settled && report.overshootInches <= config.toleranceInches &&
           report.crossings < 3;
}

double trialScore(const TrialReport& report, const AutonTuneConfig& config) {
    // Relative error allows short and long targets to compete fairly for the
    // diagnostic recommendation. It does not replace confirmation of all targets.
    return (std::abs(report.finalErrorInches) + report.overshootInches +
            report.crossings * config.toleranceInches) /
           std::max(report.target, 1e-6);
}

int GainSearch::direction(Adjustment adjustment) {
    switch (adjustment) {
    case Adjustment::IncreaseP:
    case Adjustment::DecreaseD:
    case Adjustment::IncreaseI:
        return 1;
    case Adjustment::DecreaseP:
    case Adjustment::IncreaseD:
    case Adjustment::DecreaseI:
        return -1;
    case Adjustment::None:
        return 0;
    }
    return 0;
}

bool GainSearch::adjustsPd(Adjustment adjustment) {
    return adjustment == Adjustment::IncreaseP || adjustment == Adjustment::DecreaseP ||
           adjustment == Adjustment::IncreaseD || adjustment == Adjustment::DecreaseD;
}

GainSearch::Adjustment GainSearch::classifyChange(const Gains& before, const Gains& after) {
    // A custom policy's actual output, rather than the overridden heuristic,
    // determines the next trial's reversal and integral-gating history.
    if (after.kI != before.kI)
        return after.kI > before.kI ? Adjustment::IncreaseI : Adjustment::DecreaseI;
    if (after.kP != before.kP)
        return after.kP > before.kP ? Adjustment::IncreaseP : Adjustment::DecreaseP;
    if (after.kD != before.kD)
        return after.kD > before.kD ? Adjustment::IncreaseD : Adjustment::DecreaseD;
    return Adjustment::None;
}

bool GainSearch::canAddIntegral(const TrialReport& report) const {
    return config_.allowIntegral && pdAdjustments_ >= config_.integralAfterPdAdjustments &&
           consecutiveUndershoots_ >= config_.integralUndershootTrials &&
           report.gains.kP >= std::min(12.0, config_.maxKp) &&
           report.finalErrorInches <= config_.integralWindupRange && report.crossings == 0 &&
           report.finalSpeedInchesPerSecond <= config_.maxSettledSpeed;
}

SearchDecision GainSearch::nextCandidate(const TrialReport& report, unsigned trialIndex) {
    Gains candidate = report.gains;
    Adjustment adjustment = Adjustment::None;
    const bool overshoots =
        report.overshootInches > config_.toleranceInches || report.crossings >= 3;
    const bool undershoots = report.finalErrorInches > config_.toleranceInches;
    const int nextDirection = overshoots ? -1 : 1;
    const bool reversesDirection = direction(previousAdjustment_) * nextDirection < 0;

    if (!overshoots && !undershoots) {
        // At the target but still moving/noisy: higher gains are not a remedy.
        // Each retry is an ordinary trial and still consumes the session budget.
        consecutiveUndershoots_ = 0;
        if (++unsettledRetries_ > maxUnsettledRetries)
            return {SearchAction::RetryLimit, report.gains};
        return {SearchAction::RetryCurrent, report.gains};
    }
    unsettledRetries_ = 0;
    if (reversesDirection)
        fineSteps_ = true;
    if (overshoots) {
        consecutiveUndershoots_ = 0;
        if (candidate.kI > 0) {
            candidate.kI = std::max(0.0, candidate.kI - config_.kiStep);
            adjustment = Adjustment::DecreaseI;
        } else if (candidate.kD < config_.maxKd) {
            candidate.kD = std::min(
                config_.maxKd, candidate.kD + SelfTuningAuton::gainStep(candidate.kD, fineSteps_));
            adjustment = Adjustment::IncreaseD;
        } else if (candidate.kP > initialKp) {
            candidate.kP =
                std::max(initialKp, candidate.kP - SelfTuningAuton::gainStep(candidate.kP, true));
            adjustment = Adjustment::DecreaseP;
        }
    } else {
        ++consecutiveUndershoots_;
        if (canAddIntegral(report)) {
            candidate.kI = std::min(config_.maxKi, candidate.kI + config_.kiStep);
            adjustment = Adjustment::IncreaseI;
        } else {
            candidate.kP = std::min(
                config_.maxKp, candidate.kP + SelfTuningAuton::gainStep(candidate.kP, fineSteps_));
            adjustment = Adjustment::IncreaseP;
        }
    }
    if (reversesDirection && config_.bisectOnReversal) {
        candidate = {(report.gains.kP + previousGains_.kP) / 2,
                     (report.gains.kI + previousGains_.kI) / 2,
                     (report.gains.kD + previousGains_.kD) / 2};
    }
    if (config_.adjustmentPolicy) {
        candidate = config_.adjustmentPolicy(report.gains, report.finalErrorInches,
                                             report.overshootInches, trialIndex);
        adjustment = classifyChange(report.gains, candidate);
        // A policy can reverse direction even when the heuristic did not.
        // Its current candidate is authoritative; future heuristic steps go fine.
        if (direction(previousAdjustment_) * direction(adjustment) < 0)
            fineSteps_ = true;
    }
    if (!validPidGains(candidate) || candidate.kP > config_.maxKp || candidate.kI > config_.maxKi ||
        candidate.kD > config_.maxKd)
        return {SearchAction::GainLimit, report.gains};
    if (candidate.kP == report.gains.kP && candidate.kI == report.gains.kI &&
        candidate.kD == report.gains.kD)
        return {SearchAction::GainLimit, report.gains};

    previousGains_ = report.gains;
    previousAdjustment_ = adjustment;
    if (adjustsPd(adjustment))
        ++pdAdjustments_;
    return {SearchAction::UseCandidate, candidate};
}
} // namespace detail

double SelfTuningAuton::gainStep(double gain, bool fine) {
    if (fine)
        return std::max(0.1, gain * 0.02);
    if (gain < 6)
        return 3.0;
    return std::max(1.0, gain * 0.2);
}
} // namespace autotune
