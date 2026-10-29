// Call tuningOpcontrol(chassis, controller) from your project's opcontrol().
// Keep your existing calibrated chassis and controller objects. This function
// owns driver control; do not run another task that writes drivetrain commands.
#include "autotune/api.hpp"
#include <cstdio>

void tuningOpcontrol(lemlib::Chassis& chassis, pros::Controller& controller) {
    while (!pros::competition::is_disabled()) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            chassis.tank(0, 0, true);
            std::printf("Guantune starting (driver control paused)\n");
            std::fflush(stdout);
            // Blocking call: joystick commands resume only after tuning exits.
            auto result = guantune::tuneLateral(chassis);
            std::printf("Guantune: %s, trials=%u\n",
                        guantune::statusName(result.status),
                        static_cast<unsigned>(result.trials.size()));
            if (result.status == guantune::AutonTuneStatus::COMPLETE) {
                std::printf("%s", guantune::controllerSettingsSnippet(result).c_str());
            }
            std::fflush(stdout);
            continue;
        }
        chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}
