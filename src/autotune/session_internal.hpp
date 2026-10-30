#pragma once

// Private shared types. Hardware I/O stays behind TuningDrive; policy, timing,
// measurement and recording do not depend on PROS or LemLib.
#include "autotune/self_tuning_auton.hpp"
#include <optional>

namespace autotune {
namespace detail {
constexpr std::uint32_t controlPeriodMs = 10;
constexpr std::uint32_t motionStartupTimeoutMs = 250;
constexpr double initialKp = 1.1;
constexpr double initialKd = 1.1;
constexpr unsigned maxUnsettledRetries = 5;
constexpr std::size_t maxTelemetrySamplesCap = 100000;

const char* validationError(const AutonTuneConfig& config);
bool finitePose(const DrivePose& pose);
double headingDelta(double current, double previous);

// Best effort during exception unwinding: attempt restoration even if stop()
// throws. Hardware implementations must still make both operations nonthrowing;
// swallowing a cleanup exception cannot guarantee that hardware actually stopped.
struct DriveCleanup {
    TuningDrive& drive;
    ~DriveCleanup() noexcept {
        try {
            drive.stop();
        } catch (...) {
        }
        try {
            drive.restoreGains();
        } catch (...) {
        }
    }
};

// Deadlines alone cannot terminate loops when the drive's clock has stalled.
// Poll count is independent of that clock, while unsigned deltas handle wrap.
class ClockWatchdog {
  public:
    explicit ClockWatchdog(std::uint32_t nowMs) : lastMs_(nowMs) {}
    bool healthy(std::uint32_t nowMs);

  private:
    std::uint32_t lastMs_;
    unsigned stalledPolls_ = 0;
};

AutonTuneStatus restingBatteryStatus(double millivolts, const AutonTuneConfig& config);
class BatteryMonitor {
  public:
    explicit BatteryMonitor(const AutonTuneConfig& config)
        : minimumMillivolts_(config.minimumBatteryMillivolts) {}
    AutonTuneStatus check(double millivolts, std::uint32_t deltaMs);

  private:
    double minimumMillivolts_;
    std::uint32_t lowForMs_ = 0;
};

struct MotionSample {
    std::uint32_t timestampMs;
    std::uint32_t deltaMs;
    double position; // Signed displacement from the fixed session origin.
    std::optional<double> velocity;
    double speed;
    double sensorSpeed; // At least 1 ms: same-tick jumps must not disappear.
    double translationSpeed;
    double translationFromOrigin;
    double crossTrack;
    double headingFromOrigin;
};

// Lives for the entire session, so trial/coast/return transitions never discard
// a pose or heading change. Absolute compass targets wrap; displacement does not.
class MotionTracker {
  public:
    MotionTracker(DrivePose origin, Mode mode, std::uint32_t timestampMs);
    MotionSample sample(DrivePose pose, std::uint32_t timestampMs);
    const DrivePose& origin() const { return origin_; }

  private:
    DrivePose origin_;
    DrivePose previous_;
    Mode mode_;
    std::uint32_t previousTimeMs_;
    double forwardX_;
    double forwardY_;
    double position_ = 0;
};
AutonTuneStatus motionStatus(const MotionSample& sample, const AutonTuneConfig& config);

TelemetryCommand stoppedCommand();
TelemetryCommand returnCommand(const AutonTuneConfig& config);
class TelemetryRecorder {
  public:
    TelemetryRecorder(TuningDrive& drive, const AutonTuneConfig& config, AutonTuneResult& result);
    void beginTrial(unsigned index);
    void record(const MotionSample& sample, TelemetryPhase phase, double target, double battery,
                TelemetryCommand command = {});

  private:
    TuningDrive& drive_;
    const AutonTuneConfig& config_;
    AutonTuneResult& result_;
    std::uint32_t beginMs_;
    std::optional<std::uint32_t> lastSampleMs_;
    std::optional<TelemetryPhase> lastPhase_;
    unsigned trialIndex_ = 0;
    std::size_t recordCount_ = 0;
};

enum class SearchAction { UseCandidate, RetryCurrent, RetryLimit, GainLimit };
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
bool trialPassed(const TrialReport& report, const AutonTuneConfig& config);
double trialScore(const TrialReport& report, const AutonTuneConfig& config);
} // namespace detail
} // namespace autotune
