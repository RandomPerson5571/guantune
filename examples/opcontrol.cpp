// Call tuningOpcontrol(chassis, controller) from your project's opcontrol().
// Keep your existing calibrated chassis and controller objects. This function
// owns driver control; do not run another task that writes drivetrain commands.
#include "autotune/api.hpp"
#include <cstdio>

void tuningOpcontrol(lemlib::Chassis& chassis, pros::Controller& controller) {
    chassis.tank(0, 0, true);
    std::printf("Guantune starting automatically (driver control paused)\n");
    std::fflush(stdout);
    // Blocking call: no buttons or joystick writes during the tuning session.
    // Uses the default 20-trial limit, including confirmation trials.
    auto result = guantune::tuneLateral(chassis);
    std::printf("Guantune: %s, trials=%u\n",
                guantune::statusName(result.status),
                static_cast<unsigned>(result.trials.size()));
    if (result.status == guantune::AutonTuneStatus::COMPLETE) {
        std::printf("%s", guantune::controllerSettingsSnippet(result).c_str());
    }
    std::fflush(stdout);

    // Resume normal driver control after the single session exits. Do not retry
    // automatically: failed runs and trial limits need their status inspected.
    while (!pros::competition::is_disabled()) {
        chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}
