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
    config.allowIntegral = false;
    autotune::AutonTuner tuner(drive, config);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    // Each trial moves 24 inches forward from its own starting pose.
    // Motors are stopped here. Reposition the robot, release A, then press A
    // to authorize EACH trial. B aborts this waiting phase.
    // For hands-free operation, replace this callback with your own bounded
    // repositioning routine that completes before returning true.
    auto result = tuner.run([&](unsigned trial, const autotune::Gains& gains) {
        std::printf("Trial %u: P=%.3f I=%.4f D=%.3f. Reposition; A=run, B=abort\n",
                    trial + 1, gains.kP, gains.kI, gains.kD);
        bool released = false;
        while (!pros::competition::is_disabled()) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) return false;
            const bool pressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
            if (!pressed) released = true;
            if (released && pressed) return true;
            pros::delay(10);
        }
        return false;
    });
    for (const auto& trial : result.trials) {
        std::printf("P=%.3f I=%.4f D=%.3f distance=%.2f error=%.2f overshoot=%.2f crossings=%u settled=%d timeout=%d\n",
                    trial.gains.kP, trial.gains.kI, trial.gains.kD, trial.distanceInches,
                    trial.finalErrorInches, trial.overshootInches, trial.crossings,
                    trial.settled, trial.timedOut);
    }
    std::printf("Status=%d, last tested P=%.3f I=%.4f D=%.3f\n",
                static_cast<int>(result.status), result.gains.kP, result.gains.kI, result.gains.kD);
    return result; // Apply only COMPLETE results to your chassis configuration.
}
