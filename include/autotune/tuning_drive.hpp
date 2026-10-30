#pragma once

#include "autotune/telemetry.hpp"
#include <cstdint>

namespace autotune {

// LemLib coordinates: inches, clockwise degrees, zero heading along +y.
struct DrivePose {
    double x;
    double y;
    double headingDegrees;
};

// Hardware boundary for the tuner. Host tests implement this interface;
// LemLibTuningDrive supplies the PROS implementation. The caller grants
// exclusive drivetrain access for the entire session, including returns.
class TuningDrive {
  public:
    virtual ~TuningDrive() = default;
    virtual DrivePose pose() = 0;
    // Install/restore only while the owned motion is idle. Cleanup must not throw.
    virtual void setTrialGains(const Gains& gains, double windupRange) = 0;
    virtual void restoreGains() = 0;

    // Motion requests may queue asynchronously; isMoving() can initially be false.
    virtual void moveTo(const DrivePose& target, bool forwards, std::uint32_t timeoutMs,
                        double maxSpeed) = 0;
    virtual bool isMoving() = 0;
    virtual void driveBackward(double power) = 0; // Fixed low power; no PID.
    virtual void stop() = 0; // Cancel and wait until the owned motion has stopped.

    // Monotonic uint32 clock in milliseconds; ordinary wrap is supported.
    virtual std::uint32_t nowMs() = 0;
    virtual void delayMs(std::uint32_t ms) = 0;
    // Custom hardware adapters must supply real readings. The default supports
    // minimal host fixtures; LemLibTuningDrive reads the actual battery/motors.
    virtual double batteryMillivolts() { return 12000; }
    virtual std::vector<MotorTelemetry> motorTelemetry(const std::vector<std::int8_t>&) {
        return {};
    }
    virtual bool calibrated() { return true; } // Hardware adapter uses caller's readiness check.
    virtual bool supportsAngular() const { return false; }
    virtual void selectMode(Mode) {}
    // Absolute angular commands from the tuner are normalized to [0, 360).
    virtual void turnTo(double, std::uint32_t, double) {}
    virtual void turnBack(double) {}
    virtual bool available() = 0; // False on disable; owned motions remain available.
};

} // namespace autotune
