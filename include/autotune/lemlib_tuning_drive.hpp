#pragma once

#include "autotune/self_tuning_auton.hpp"
#include <lemlib/api.hpp>
#if __has_include("autotune/dependency_version.hpp")
#include "autotune/dependency_version.hpp"
#endif
#if defined(GUANTUNE_LEMLIB_VERSION) && GUANTUNE_LEMLIB_VERSION != 506 && !defined(GUANTUNE_ALLOW_UNTESTED_LEMLIB)
#error "Guantune targets LemLib 0.5.6. Audit the adapter or define GUANTUNE_ALLOW_UNTESTED_LEMLIB to probe compatibility."
#endif
#include <pros/misc.hpp>
#include <pros/misc.h>
#include "autotune/idle_pid_guard.hpp"
#include <pros/rtos.hpp>
#include <memory>
#include <new>

namespace autotune {

class LemLibTuningDrive final : public TuningDrive {
public:
    // LemLib exposes no calibrated flag: the caller supplies a readiness probe.
    explicit LemLibTuningDrive(lemlib::Chassis& chassis,
                              std::function<bool()> ready = [] { return true; })
        : chassis_(chassis), ready_(std::move(ready)) {}
    ~LemLibTuningDrive() override { stop(); restoreGains(); }
    LemLibTuningDrive(const LemLibTuningDrive&) = delete;
    LemLibTuningDrive& operator=(const LemLibTuningDrive&) = delete;
    DrivePose pose() override {
        const auto p = chassis_.getPose();
        return {p.x, p.y, p.theta};
    }
    void setTrialGains(const Gains& gains, double windupRange) override {
        stop(); // Join any motion this adapter owns before copying PID state.
        if (!pidGuard_) pidGuard_.emplace(selectedPID(), [this] { stop(); });
        // Construct before replacing the live object. Argument 5 resets integral
        // on sign flip; it is unrelated to ControllerSettings::largeError.
        const lemlib::PID candidate(gains.kP, gains.kI, gains.kD, windupRange, true);
        pidGuard_->install(candidate);
    }
    void restoreGains() override {
        if (pidGuard_) { pidGuard_->restore(); pidGuard_.reset(); }
    }
    bool calibrated() override { return ready_(); }
    double batteryMillivolts() override { return pros::battery::get_voltage(); }
    bool supportsAngular() const override { return true; }
    void selectMode(Mode mode) override { restoreGains(); mode_ = mode; }
    void turnTo(double heading, std::uint32_t timeout, double output) override {
        lemlib::TurnToHeadingParams params;
        params.maxSpeed = static_cast<int>(output);
        params.direction = lemlib::AngularDirection::CW_CLOCKWISE;
        ownsMotion_ = true;
        chassis_.turnToHeading(heading, static_cast<int>(timeout), params, true);
    }
    void turnBack(double power) override {
        ownsManualDrive_ = true;
        chassis_.tank(-power, power, true);
    }
    void moveTo(const DrivePose& target, bool forwards,
                std::uint32_t timeoutMs, double maxSpeed) override {
        lemlib::MoveToPointParams params;
        params.forwards = forwards;
        params.maxSpeed = maxSpeed;
        params.minSpeed = 0;
        ownsMotion_ = true;
        chassis_.moveToPoint(target.x, target.y, static_cast<int>(timeoutMs), params, true);
    }
    bool isMoving() override { return chassis_.isInMotion(); }
    void driveBackward(double power) override {
        ownsManualDrive_ = true;
        chassis_.tank(-power, -power, true); // Bypass driver curves at low power.
    }
    void stop() override {
        if (ownsMotion_) {
            chassis_.cancelAllMotions();
            chassis_.waitUntilDone(); // Finish the motion task before changing its PID.
            ownsMotion_ = false;
        }
        if (ownsManualDrive_) {
            chassis_.tank(0, 0, true);
            ownsManualDrive_ = false;
        }
    }
    std::uint32_t nowMs() override { return pros::millis(); }
    void delayMs(std::uint32_t ms) override { pros::delay(ms); }
    bool available() override {
        return !pros::competition::is_disabled();
    }
private:
    lemlib::Chassis& chassis_;
    lemlib::PID& selectedPID() { return mode_ == Mode::ANGULAR ? chassis_.angularPID : chassis_.lateralPID; }
    std::function<bool()> ready_;
    Mode mode_ = Mode::LATERAL;
    std::optional<IdlePidGuard<lemlib::PID>> pidGuard_;
    bool ownsMotion_ = false;
    bool ownsManualDrive_ = false;
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

inline AutonTuneResult tuneLateral(lemlib::Chassis& chassis, AutonTuneConfig config = {},
                                   std::function<bool()> ready = [] { return true; }) {
    config.mode = Mode::LATERAL;
    LemLibTuningDrive drive(chassis, std::move(ready));
    return SelfTuningAuton(drive, std::move(config)).run();
}
inline AutonTuneResult tuneAngular(lemlib::Chassis& chassis, AutonTuneConfig config,
                                   std::function<bool()> ready = [] { return true; }) {
    config.mode = Mode::ANGULAR;
    LemLibTuningDrive drive(chassis, std::move(ready));
    return SelfTuningAuton(drive, std::move(config)).run();
}
} // namespace autotune
