// Tune once before driver control. An optional caller callback can display the
// returned result on a screen or send it over the team's telemetry connection.
#include "autotune/api.hpp"
#include <functional>

void tuningOpcontrol(
    lemlib::Chassis& chassis, pros::Controller& controller,
    const std::function<void(const guantune::AutonTuneResult&)>& onResult = {}) {
    chassis.tank(0, 0, true);
    auto result = guantune::tuneLateral(chassis);
    if (onResult) onResult(result);

    // Resume driver control after the one session and caller callback finish.
    while (!pros::competition::is_disabled()) {
        chassis.arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                       controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}
