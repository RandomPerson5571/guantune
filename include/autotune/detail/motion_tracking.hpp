#pragma once

// Private implementation detail; not part of the installed public API.
#include "autotune/tuning_config.hpp"
#include "autotune/position.hpp"
#include "autotune/tuning_result.hpp"
#include <optional>

namespace autotune {
namespace detail {
bool finitePose(const ChassisPosition& pose);
double headingDelta(double current, double previous);

struct MotionSample {
    std::uint32_t timestampMs;
    std::uint32_t deltaMs;
    double position; // Signed displacement from the fixed session origin.
    std::optional<double> velocity;
    double speed;
    double sensorSpeed; // At least 1 ms: same-tick jumps must not disappear.
    double translationSpeed;
    double translationFromOrigin;
    double crossTrack;
    double headingFromOrigin;
};

// Lives for the entire session, so trial/coast/return transitions never discard
// a pose or heading change. Absolute compass targets wrap; displacement does not.
class MotionTracker {
  public:
    MotionTracker(ChassisPosition origin, Mode mode, std::uint32_t timestampMs);
    MotionSample sample(ChassisPosition pose, std::uint32_t timestampMs);
    const ChassisPosition& origin() const { return origin_; }

  private:
    ChassisPosition origin_;
    ChassisPosition previous_;
    Mode mode_;
    std::uint32_t previousTimeMs_;
    double forwardX_;
    double forwardY_;
    double position_ = 0;
};
AutonTuneStatus motionStatus(const MotionSample& sample, const AutonTuneConfig& config);
} // namespace detail
} // namespace autotune
