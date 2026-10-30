#pragma once

#include "autotune/idle_pid_guard.hpp"
#include "autotune/pid_validation.hpp"
#include "autotune/self_tuning_auton.hpp"
#include <lemlib/api.hpp>
#if __has_include("autotune/dependency_version.hpp")
#include "autotune/dependency_version.hpp"
#endif
#if defined(GUANTUNE_LEMLIB_VERSION) && GUANTUNE_LEMLIB_VERSION != 506 &&                          \
    !defined(GUANTUNE_ALLOW_UNTESTED_LEMLIB)
#error                                                                                             \
    "Guantune targets LemLib 0.5.6. Audit the adapter or define GUANTUNE_ALLOW_UNTESTED_LEMLIB to probe compatibility."
#endif

namespace autotune {
// Adapts only hardware operations. Search decisions and motion measurements
// live in the platform-independent core, so host tests exercise the real logic.
class LemLibTuningDrive final : public TuningDrive {
  public:
    // LemLib has no calibrated flag. The default assumes the caller calibrated;
    // provide a readiness probe when that cannot be guaranteed by the caller.
    explicit LemLibTuningDrive(
        lemlib::Chassis& chassis, std::function<bool()> ready = [] { return true; });
    ~LemLibTuningDrive() override;
    LemLibTuningDrive(const LemLibTuningDrive&) = delete;
    LemLibTuningDrive& operator=(const LemLibTuningDrive&) = delete;

    DrivePose pose() override;
    void setTrialGains(const Gains& gains, double windupRange) override;
    void restoreGains() override;
    bool calibrated() override;
    double batteryMillivolts() override;
    std::vector<MotorTelemetry> motorTelemetry(const std::vector<std::int8_t>& ports) override;
    bool supportsAngular() const override;
    void selectMode(Mode mode) override;
    void turnTo(double heading, std::uint32_t timeoutMs, double maxOutput) override;
    void turnBack(double power) override;
    void moveTo(const DrivePose& target, bool forwards, std::uint32_t timeoutMs,
                double maxOutput) override;
    bool isMoving() override;
    void driveBackward(double power) override;
    void stop() override;
    std::uint32_t nowMs() override;
    void delayMs(std::uint32_t ms) override;
    bool available() override;

  private:
    lemlib::PID& selectedPID();
    lemlib::Chassis& chassis_;
    std::function<bool()> ready_;
    Mode mode_ = Mode::LATERAL;
    std::optional<IdlePidGuard<lemlib::PID>> pidGuard_;
    bool ownsMotion_ = false;
    bool ownsManualDrive_ = false;
};

// Pure conversion stays inline so native tests can check it against real
// LemLib headers without linking hardware code. Preserve caller exit/slew settings.
inline std::optional<lemlib::ControllerSettings> toLemLib(const AutonTuneResult& result,
                                                          lemlib::ControllerSettings base) {
    if (result.status != AutonTuneStatus::COMPLETE || !detail::validPidGains(result.gains) ||
        !detail::validPidScalar(result.integralWindupRange) || result.integralWindupRange <= 0)
        return std::nullopt;
    base.kP = result.gains.kP;
    base.kI = result.gains.kI;
    base.kD = result.gains.kD;
    base.windupRange = result.integralWindupRange;
    return base;
}

// Blocking helpers use LemLib's gain convention at a 10 ms update interval.
AutonTuneResult tuneLateral(
    lemlib::Chassis& chassis, AutonTuneConfig config = {},
    std::function<bool()> ready = [] { return true; });
AutonTuneResult tuneAngular(
    lemlib::Chassis& chassis, AutonTuneConfig config,
    std::function<bool()> ready = [] { return true; });
} // namespace autotune
