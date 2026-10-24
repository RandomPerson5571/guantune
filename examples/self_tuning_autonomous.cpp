// Add to an existing PROS/LemLib project. Call tuneAutonomous(chassis) from
// autonomous(), after odometry calibration and with no other drivetrain task.
#include "autotune/lemlib_tuning_drive.hpp"
#include "pros/misc.hpp"
#include <cstdio>

autotune::AutonTuneResult tuneAutonomous(lemlib::Chassis& chassis) {
    autotune::LemLibTuningDrive drive(chassis);
    autotune::AutonTuneConfig config;
    config.range = 0.1; // Accept distance error within +/-0.1 inch.
    config.maxTrials = 15;
    config.returnPower = 20; // Slowly back up while checking chassis pose.
    config.returnToleranceInches = 0.5;
    config.allowIntegral = false;
    autotune::AutonTuner tuner(drive, config);
    // Forward trials use LemLib moveToPoint. Return uses a low-power LemLib
    // tank command until chassis pose is near the saved start, then settles.
    auto result = tuner.run([](unsigned trial, const autotune::Gains& gains) {
        std::printf("Trial %u: P=%.3f I=%.4f D=%.3f\n",
                    trial + 1, gains.kP, gains.kI, gains.kD);
        return true;
    });
    for (const auto& trial : result.trials) {
        std::printf("P=%.3f I=%.4f D=%.3f distance=%.2f error=%.2f overshoot=%.2f crossings=%u settled=%d timeout=%d returned=%d battery_start=%.0f battery_min=%.0f\n",
                    trial.gains.kP, trial.gains.kI, trial.gains.kD, trial.distanceInches,
                    trial.finalErrorInches, trial.overshootInches, trial.crossings,
                    trial.settled, trial.timedOut, trial.returnedToStart,
                    trial.batteryStartMillivolts, trial.batteryMinimumMillivolts);
    }
    std::printf("Status=%d, candidate P=%.3f I=%.4f D=%.3f\n",
                static_cast<int>(result.status), result.gains.kP, result.gains.kI, result.gains.kD);
    return result; // Apply only COMPLETE results to your chassis configuration.
}
