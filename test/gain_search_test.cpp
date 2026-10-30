// Unit tests for the pure policy component. Hardware/motion tests use TuningDrive.
#include "../src/autotune/session_internal.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace autotune;

void testPolicyChangesDriveHistory() {
    AutonTuneConfig config;
    config.adjustmentPolicy = [](const Gains&, double, double, unsigned) {
        return Gains{1.1, 0, 4.1};
    };
    detail::GainSearch search(config);
    TrialReport report;
    report.target = 24;
    report.finalErrorInches = 2;
    auto decision = search.nextCandidate(report, 0);
    assert(decision.gains.kD == 4.1);
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
    report.gains = search.nextCandidate(report, 0).gains;
    report.gains = search.nextCandidate(report, 1).gains;
    assert(report.gains.kP == 6);
    const auto decision = search.nextCandidate(report, 2);
    assert(decision.action == detail::SearchAction::UseCandidate);
    assert(decision.gains.kI == config.kiStep && decision.gains.kP == 6);
}

void testPolicyReversalEnablesFineSteps() {
    AutonTuneConfig config;
    config.adjustmentPolicy = [](const Gains&, double, double, unsigned index) {
        return index == 0 ? Gains{1.1, 0, 4.1} : Gains{4.1, 0, 4.1};
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
    assert(std::abs(decision.gains.kP - 4.2) < 1e-9);
}

int main() {
    assert(SelfTuningAuton::gainStep(100) == 20);
    assert(SelfTuningAuton::gainStep(100, true) == 2);
    testPolicyChangesDriveHistory();
    testDampingCapBacksOffP();
    testUnsettledRetriesAreBounded();
    testScoresUseRelativeError();
    testIntegralGateUsesConfiguredPCap();
    testPolicyReversalEnablesFineSteps();
    std::cout << "Gain stepping, policy history, retries and ranking tests passed\n";
}
