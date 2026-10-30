// Angular tuning in an existing calibrated PROS/LemLib robot project.
#include "autotune/api.hpp"
#include <cstdio>

void tuneAngularExample(lemlib::Chassis& chassis) {
    guantune::AutonTuneConfig config;
    config.distanceInches = 90; // Angular mode uses degrees for distance fields.
    config.testDistances = {90, 170};
    config.maxTravelInches = 210;
    config.maxSensorSpeed = 400;
    config.trialTimeoutMs = 15000;
    config.returnTimeoutMs = 20000;
    config.maxTrials = 60;
    auto result = guantune::tuneAngular(chassis, config);
    if (result.status == guantune::AutonTuneStatus::COMPLETE)
        std::printf("%s", guantune::controllerSettingsSnippet(result).c_str());
}
