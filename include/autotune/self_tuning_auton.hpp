#pragma once

#include "autotune/tuning_config.hpp"
#include "autotune/tuning_drive.hpp"
#include "autotune/tuning_result.hpp"
#include <atomic>
#include <functional>

namespace autotune {
namespace detail {
class MotionTracker;
class TelemetryRecorder;
} // namespace detail

// Runs one blocking tuning session. The caller grants exclusive drivetrain
// access until run() returns; joystick loops and other motions must wait.
class SelfTuningAuton {
  public:
    // Called before each trial. Return false to abort; never reposition the robot
    // here. Trial motion and the return to the saved origin are automatic.
    using BeforeTrial = std::function<bool(unsigned, const Gains&)>;

    explicit SelfTuningAuton(TuningDrive& drive, AutonTuneConfig config = {});
    AutonTuneResult run(const BeforeTrial& beforeTrial = {});
    // Latched: cancels the active run, or exactly the next run when called idle.
    // The owning run consumes the request on exit; rejected concurrent calls do not.
    void abort();

    // Coarse: 3 below 6, then 20% (at least 1). Fine: 2% (at least 0.1).
    static double gainStep(double gain, bool fine = false);

  private:
    TuningDrive& drive_;
    AutonTuneConfig config_;
    std::atomic<bool> aborted_{false};
    std::atomic_flag running_ = ATOMIC_FLAG_INIT;

    AutonTuneStatus checkReadiness();
    AutonTuneStatus waitForIdle();
    AutonTuneStatus checkCancellation();
    AutonTuneStatus checkIndependentDisplacement(double displacement,
                                                 std::optional<double>* reading = nullptr);
    AutonTuneStatus runTrial(const Gains& gains, double target, detail::MotionTracker& motion,
                             TrialReport& report, detail::TelemetryRecorder& telemetry);
    AutonTuneStatus returnToStart(detail::MotionTracker& motion,
                                  detail::TelemetryRecorder& telemetry);
};

using AutonTuner = SelfTuningAuton;

} // namespace autotune
