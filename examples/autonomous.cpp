// API usage sketch for an existing PROS + LemLib robot project.
// Not linkable until the library methods are implemented.
#include "autotune/api.hpp"

// Supply your existing calibrated chassis; no robot wiring is assumed here.
void tuneAngular(lemlib::Chassis& chassis) {
    autotune::Config config;
    config.mode = autotune::Mode::ANGULAR;
    config.testMagnitude = 90;
    config.maxOutput = 60;
    config.maxTravel = 180;

    autotune::Tuner tuner(chassis, config);
    const auto result = tuner.run();
    result.print();

    // Once implemented: inspect a COMPLETE result, convert using toLemLib(result,
    // yourExistingSettings), and use the returned settings when creating a chassis.
    // Do not assume LemLib exposes mutable angularSettings on an existing chassis.
}
