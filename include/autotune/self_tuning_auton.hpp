#pragma once

#include "autotune/tuning_config.hpp"
#include "autotune/lemlib_tuning_drive.hpp"
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

    // chassis is borrowed and must outlive this object. config is copied so
    // caller edits cannot change a running session. ready is a prompt readiness
    // probe; it should report calibration completion without starting motion.
    explicit SelfTuningAuton(lemlib::Chassis& chassis, AutonTuneConfig config = {},
                             std::function<bool()> ready = [] { return true; });
    // Runs trials, returns, and confirmations synchronously. Restores the
    // original PID on exit; successful gains are returned for deliberate use.
    // The per-object run guard cannot coordinate other tuners or driver tasks.
    AutonTuneResult run(const BeforeTrial& beforeTrial = {});
    // Latched: cancels the active run, or exactly the next run when called idle.
    // The owning run consumes the request on exit; rejected concurrent calls do not.
    void abort();

  private:
    LemLibTuningDrive drive_;
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
