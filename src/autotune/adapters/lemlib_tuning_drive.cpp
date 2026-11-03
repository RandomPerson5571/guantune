#include "autotune/lemlib_tuning_drive.hpp"
#include "autotune/self_tuning_auton.hpp"
#include "autotune/angle.hpp"
#include <cmath>
#include <limits>
#include <pros/misc.hpp>
#include <pros/motors.h>
#include <pros/rtos.hpp>
#include <utility>

namespace autotune {
LemLibTuningDrive::LemLibTuningDrive(lemlib::Chassis& chassis, std::function<bool()> ready)
    : chassis_(chassis), ready_(std::move(ready)) {}

LemLibTuningDrive::~LemLibTuningDrive() {
    stop();
    restoreGains();
}

ChassisPosition LemLibTuningDrive::pose() {
    const auto pose = chassis_.getPose();
    return {pose.x, pose.y, pose.theta};
}

lemlib::PID& LemLibTuningDrive::selectedPID() {
    return mode_ == Mode::ANGULAR ? chassis_.angularPID : chassis_.lateralPID;
}

// Save the original selected PID once per guard lifetime, including controller
// state. LemLib's const gain members require guarded reconstruction rather than
// assignment; no motion may still be using that PID when it is replaced.
void LemLibTuningDrive::setTrialGains(const Gains& gains, double windupRange) {
    stop(); // Finish owned motion before copying or reconstructing PID state.
    if (!pidGuard_)
        pidGuard_.emplace(selectedPID(), [this] { stop(); });
    // Argument 5 resets integral on sign flip; it is not the large-error exit.
    const lemlib::PID candidate(gains.kP, gains.kI, gains.kD, windupRange, true);
    pidGuard_->install(candidate);
}

void LemLibTuningDrive::restoreGains() {
    if (!pidGuard_)
        return;
    pidGuard_->restore();
    pidGuard_.reset();
}

bool LemLibTuningDrive::calibrated() { return ready_(); }
double LemLibTuningDrive::batteryMillivolts() {
    const auto voltage = pros::battery::get_voltage();
    return voltage == PROS_ERR ? std::numeric_limits<double>::quiet_NaN()
                               : static_cast<double>(voltage);
}

std::vector<MotorTelemetry>
LemLibTuningDrive::motorTelemetry(const std::vector<std::int8_t>& ports) {
    std::vector<MotorTelemetry> readings;
    readings.reserve(ports.size());
    for (auto port : ports) {
        MotorTelemetry reading;
        reading.port = port;
        const auto voltage = pros::c::motor_get_voltage(port);
        const auto current = pros::c::motor_get_current_draw(port);
        const auto temperature = pros::c::motor_get_temperature(port);
        if (voltage != PROS_ERR)
            reading.measuredMillivolts = voltage;
        if (current != PROS_ERR)
            reading.currentMilliamps = current;
        if (std::isfinite(temperature))
            reading.temperatureCelsius = temperature;
        readings.push_back(reading);
    }
    return readings;
}

void LemLibTuningDrive::selectMode(Mode mode) {
    restoreGains();
    mode_ = mode;
}

void LemLibTuningDrive::turnTo(double heading, std::uint32_t timeoutMs, double maxOutput) {
    lemlib::TurnToHeadingParams params;
    params.maxSpeed = static_cast<int>(maxOutput);
    params.direction = lemlib::AngularDirection::CW_CLOCKWISE;
    ownsMotion_ = true;
    chassis_.turnToHeading(detail::normalizeHeadingFloat(heading), static_cast<int>(timeoutMs),
                           params, true);
}

void LemLibTuningDrive::moveTo(const ChassisPosition& target, std::uint32_t timeoutMs,
                               double maxOutput) {
    lemlib::MoveToPointParams params;
    params.forwards = true;
    params.maxSpeed = maxOutput;
    params.minSpeed = 0;
    ownsMotion_ = true;
    chassis_.moveToPoint(target.x, target.y, static_cast<int>(timeoutMs), params, true);
}

bool LemLibTuningDrive::isMoving() { return chassis_.isInMotion(); }

void LemLibTuningDrive::turnBack(double power) {
    ownsManualDrive_ = true;
    chassis_.tank(-power, power, true);
}

void LemLibTuningDrive::driveBackward(double power) {
    ownsManualDrive_ = true;
    chassis_.tank(-power, -power, true); // Bypass driver curves for fixed power.
}

// Ownership flags prevent early failures from cancelling unrelated chassis
// work. Joining an owned asynchronous LemLib motion is also the barrier that
// makes later PID restoration valid. Manual returns need an explicit tank zero.
void LemLibTuningDrive::stop() {
    if (ownsMotion_) {
        chassis_.cancelAllMotions();
        chassis_.waitUntilDone(); // Do not reconstruct a PID while it is in use.
        ownsMotion_ = false;
    }
    if (ownsManualDrive_) {
        chassis_.tank(0, 0, true);
        ownsManualDrive_ = false;
    }
}

std::uint32_t LemLibTuningDrive::nowMs() { return pros::millis(); }
void LemLibTuningDrive::delayMs(std::uint32_t ms) { pros::delay(ms); }
bool LemLibTuningDrive::available() { return !pros::competition::is_disabled(); }

AutonTuneResult tuneLateral(lemlib::Chassis& chassis, AutonTuneConfig config,
                            std::function<bool()> ready) {
    config.mode = Mode::LATERAL;
    return SelfTuningAuton(chassis, std::move(config), std::move(ready)).run();
}

AutonTuneResult tuneAngular(lemlib::Chassis& chassis, AutonTuneConfig config,
                            std::function<bool()> ready) {
    config.mode = Mode::ANGULAR;
    return SelfTuningAuton(chassis, std::move(config), std::move(ready)).run();
}
} // namespace autotune
