// Unit tests for gain selection; these do not simulate the LemLib chassis.
#include "autotune/detail/gain_search.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
using namespace autotune;

void testPolicyChangesDriveHistory() {
    AutonTuneConfig config;
    config.adjustmentPolicy = [](const Gains&, double, double, unsigned) {
        return Gains{1.1, 0, 1.3};
    };
    detail::GainSearch search(config);
    TrialReport report;
    report.target = 24;
    report.finalErrorInches = 2;
    auto decision = search.nextCandidate(report, 0);
    assert(decision.gains.kD == 1.3);
    // Let the heuristic choose the next candidate. The policy's actual D
    // increase made this undershoot a reversal, so P should increase finely.
    config.adjustmentPolicy = {};
    report.gains = decision.gains;
    decision = search.nextCandidate(report, 1);
    assert(std::abs(decision.gains.kP - 1.2) < 1e-9);
}

void testDampingCapBacksOffP() {
    AutonTuneConfig config;
    config.maxKd = 1.1;
    detail::GainSearch search(config);
    TrialReport report;
    report.target = 24;
    report.gains = {4.1, 0, 1.1};
    report.overshootInches = 2;
    const auto decision = search.nextCandidate(report, 0);
    assert(decision.action == detail::SearchAction::UseCandidate);
    assert(std::abs(decision.gains.kP - 4.0) < 1e-9 && decision.gains.kD == 1.1);
}

void testUnsettledRetriesAreBounded() {
    AutonTuneConfig config;
    unsigned policyCalls = 0;
    config.adjustmentPolicy = [&](const Gains&, double, double, unsigned) {
        ++policyCalls;
        return Gains{20, 0, 20};
    };
    detail::GainSearch search(config);
    TrialReport report;
    report.target = 24;
    report.finalErrorInches = 0.1;
    report.settled = false;
    for (unsigned index = 0; index < 5; ++index) {
        const auto decision = search.nextCandidate(report, index);
        assert(decision.action == detail::SearchAction::RetryCurrent);
        assert(decision.gains.kP == report.gains.kP);
    }
    assert(search.nextCandidate(report, 5).action == detail::SearchAction::RetryLimit);
    assert(policyCalls == 0); // The supplied behavior retries without applying a policy.
    search.recordSuccess();
    assert(search.nextCandidate(report, 6).action == detail::SearchAction::RetryCurrent);
}

void testScoresUseRelativeError() {
    AutonTuneConfig config;
    TrialReport shortTrial;
    shortTrial.target = 6;
    shortTrial.finalErrorInches = 1;
    TrialReport longTrial;
    longTrial.target = 24;
    longTrial.finalErrorInches = 2;
    assert(detail::trialScore(longTrial, config) < detail::trialScore(shortTrial, config));
}

void testIntegralGateUsesConfiguredPCap() {
    AutonTuneConfig config;
    config.allowIntegral = true;
    config.maxKp = 6;
    config.integralAfterPdAdjustments = 1;
    config.integralUndershootTrials = 1;
    detail::GainSearch search(config);
    TrialReport report;
    report.target = 24;
    report.finalErrorInches = 1;
    for (unsigned index = 0; index < 20 && report.gains.kP < 6; ++index)
        report.gains = search.nextCandidate(report, index).gains;
    assert(report.gains.kP == 6);
    const auto decision = search.nextCandidate(report, 20);
    assert(decision.action == detail::SearchAction::UseCandidate);
    assert(decision.gains.kI == config.kiStep && decision.gains.kP == 6);
}

void testPolicyReversalEnablesFineSteps() {
    AutonTuneConfig config;
    config.adjustmentPolicy = [](const Gains&, double, double, unsigned index) {
        return index == 0 ? Gains{1.1, 0, 1.3} : Gains{1.3, 0, 1.3};
    };
    detail::GainSearch search(config);
    TrialReport report;
    report.target = 24;
    report.finalErrorInches = 2;
    report.gains = search.nextCandidate(report, 0).gains; // Policy increases D.
    report.overshootInches = 2;
    report.gains = search.nextCandidate(report, 1).gains; // Policy reverses by increasing P.
    config.adjustmentPolicy = {};
    report.overshootInches = 0;
    const auto decision = search.nextCandidate(report, 2);
    assert(std::abs(decision.gains.kP - 1.4) < 1e-9);
}

void testInvalidMeasurementsCannotPassOrTune() {
    AutonTuneConfig config;
    TrialReport report;
    report.target = 24;
    report.settled = true;
    assert(detail::trialPassed(report, config));
    report.finalErrorInches = 2;
    assert(!detail::trialPassed(report, config));
    report.finalErrorInches = 0;
    report.finalSpeedInchesPerSecond = 2;
    assert(!detail::trialPassed(report, config));
    report.finalSpeedInchesPerSecond = 0;
    report.timedOut = true;
    assert(!detail::trialPassed(report, config));
    report.finalErrorInches = std::numeric_limits<double>::quiet_NaN();
    detail::GainSearch search(config);
    assert(!detail::trialPassed(report, config));
    assert(std::isinf(detail::trialScore(report, config)));
    assert(search.nextCandidate(report, 0).action == detail::SearchAction::InvalidReport);
}

void testCandidateEnvelopeAndIntegralOptIn() {
    AutonTuneConfig config;
    TrialReport report;
    report.target = 24;
    report.finalErrorInches = 2;
    detail::GainSearch search(config);
    const auto decision = search.nextCandidate(report, 0);
    assert(decision.action == detail::SearchAction::UseCandidate);
    assert(decision.gains.kP <= report.gains.kP * 1.25);
    config.adjustmentPolicy = [](const Gains&, double, double, unsigned) { return Gains{30, 0, 30}; };
    assert(search.nextCandidate(report, 1).action == detail::SearchAction::GainLimit);
    config.adjustmentPolicy = [](const Gains&, double, double, unsigned) { return Gains{1.1, 0.001, 1.1}; };
    assert(search.nextCandidate(report, 2).action == detail::SearchAction::GainLimit);
}

void testSuccessClearsPreviousTargetBracket() {
    AutonTuneConfig config;
    config.bisectOnReversal = true;
    detail::GainSearch search(config);
    TrialReport report;
    report.target = 24;
    report.overshootInches = 2;
    report.gains = search.nextCandidate(report, 0).gains;
    search.recordSuccess();
    report.target = 6;
    report.overshootInches = 0;
    report.finalErrorInches = 1;
    const auto next = search.nextCandidate(report, 1);
    assert(next.gains.kP > report.gains.kP);
    assert(next.gains.kD == report.gains.kD);
}

int main() {
    testInvalidMeasurementsCannotPassOrTune();
    testCandidateEnvelopeAndIntegralOptIn();
    testSuccessClearsPreviousTargetBracket();
    assert(detail::gainStep(100) == 20);
    assert(detail::gainStep(100, true) == 2);
    testPolicyChangesDriveHistory();
    testDampingCapBacksOffP();
    testUnsettledRetriesAreBounded();
    testScoresUseRelativeError();
    testIntegralGateUsesConfiguredPCap();
    testPolicyReversalEnablesFineSteps();
    std::cout << "Gain stepping, policy history, retries and ranking tests passed\n";
}
