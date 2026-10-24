#pragma once

#include "autotune/self_tuning_auton.hpp"
#include "lemlib/api.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace autotune {

class LemLibTuningDrive final : public TuningDrive {
public:
    explicit LemLibTuningDrive(lemlib::Chassis& chassis) : chassis_(chassis) {}
    DrivePose pose() override {
        const auto p = chassis_.getPose();
        return {p.x, p.y, p.theta};
    }
    void tank(double left, double right) override {
        chassis_.tank(static_cast<int>(left), static_cast<int>(right), true);
    }
    std::uint32_t nowMs() override { return pros::millis(); }
    void delayMs(std::uint32_t ms) override { pros::delay(ms); }
    bool available() override {
        return !pros::competition::is_disabled() && !chassis_.isInMotion();
    }
private:
    lemlib::Chassis& chassis_;
};

// Gains use LemLib's per-update convention at a 10 ms interval.
inline std::optional<lemlib::ControllerSettings> toLemLib(
    const AutonTuneResult& result, lemlib::ControllerSettings base) {
    if (result.status != AutonTuneStatus::COMPLETE) return std::nullopt;
    base.kP = result.gains.kP;
    base.kI = result.gains.kI;
    base.kD = result.gains.kD;
    return base;
}

} // namespace autotune
