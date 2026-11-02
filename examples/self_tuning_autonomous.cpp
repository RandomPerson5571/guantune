// Call after calibration with exclusive access to the existing LemLib chassis.
#include "autotune/self_tuning_auton.hpp"

autotune::AutonTuneResult tuneAutonomous(lemlib::Chassis& chassis) {
    autotune::AutonTuneConfig config;
    config.range = 0.1;
    config.maxTrials = 15;
    config.returnPower = 20;
    config.returnToleranceInches = 0.5;
    config.allowIntegral = false;
    autotune::AutonTuner tuner(chassis, config);
    // The returned object contains status, gains, windup range and trial reports.
    // Only result.succeeded() means the gains passed confirmation and return.
    return tuner.run();
}
