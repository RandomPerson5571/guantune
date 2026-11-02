#pragma once

// Private implementation detail; not part of the installed public API.
#include "autotune/tuning_config.hpp"
#include "autotune/tuning_result.hpp"
#include "constants.hpp"

namespace autotune {
namespace detail {
double gainStep(double gain, bool fine = false);

enum class SearchAction { UseCandidate, RetryCurrent, RetryLimit, GainLimit, InvalidReport };
struct SearchDecision {
    SearchAction action;
    Gains gains;
};

// Gain history only. The runner owns target cycling, confirmations and budgets.
class GainSearch {
  public:
    explicit GainSearch(const AutonTuneConfig& config) : config_(config) {}
    void recordSuccess() {
        consecutiveUndershoots_ = 0;
        unsettledRetries_ = 0;
        // A passed target breaks the failed-trial bracket. Its gains must not
        // be bisected against a failure at a different target later on.
        previousAdjustment_ = Adjustment::None;
    }
    SearchDecision nextCandidate(const TrialReport& report, unsigned trialIndex);

  private:
    enum class Adjustment {
        None,
        IncreaseP,
        DecreaseP,
        IncreaseD,
        DecreaseD,
        IncreaseI,
        DecreaseI
    };
    static int direction(Adjustment adjustment);
    static bool adjustsPd(Adjustment adjustment);
    static Adjustment classifyChange(const Gains& before, const Gains& after);
    bool canAddIntegral(const TrialReport& report) const;

    const AutonTuneConfig& config_;
    Gains previousGains_{initialKp, 0, initialKd};
    Adjustment previousAdjustment_ = Adjustment::None;
    unsigned pdAdjustments_ = 0;
    unsigned consecutiveUndershoots_ = 0;
    unsigned unsettledRetries_ = 0;
    bool fineSteps_ = false;
};
bool validTrialReport(const TrialReport& report, const AutonTuneConfig& config);
bool trialPassed(const TrialReport& report, const AutonTuneConfig& config);
double trialScore(const TrialReport& report, const AutonTuneConfig& config);
} // namespace detail
} // namespace autotune
