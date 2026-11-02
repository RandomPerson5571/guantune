// Angular tuning returns an object; your caller chooses how to present it.
#include "autotune/api.hpp"

guantune::AutonTuneResult tuneAngularExample(lemlib::Chassis& chassis) {
    guantune::AutonTuneConfig config;
    config.angleDegrees = 90;
    config.testAngles = {90, 170};
    config.maxTravelInches = 210;
    config.maxSensorSpeed = 400;
    config.trialTimeoutMs = 15000;
    config.returnTimeoutMs = 20000;
    config.maxTrials = 60;
    return guantune::tuneAngular(chassis, config);
}
