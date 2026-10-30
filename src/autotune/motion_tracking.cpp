#include "session_internal.hpp"
#include <algorithm>
#include <cmath>

namespace autotune {
namespace detail {
namespace {
constexpr double pi = 3.14159265358979323846;
}

bool finitePose(const DrivePose& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.headingDegrees);
}

double headingDelta(double current, double previous) {
    return std::remainder(current - previous, 360.0);
}

MotionTracker::MotionTracker(DrivePose origin, Mode mode, std::uint32_t timestampMs)
    : origin_(origin), previous_(origin), mode_(mode), previousTimeMs_(timestampMs),
      forwardX_(std::sin(origin.headingDegrees * pi / 180)),
      forwardY_(std::cos(origin.headingDegrees * pi / 180)) {}

MotionSample MotionTracker::sample(DrivePose pose, std::uint32_t timestampMs) {
    const auto deltaMs = static_cast<std::uint32_t>(timestampMs - previousTimeMs_);
    const double dx = pose.x - origin_.x;
    const double dy = pose.y - origin_.y;
    const double stepX = pose.x - previous_.x;
    const double stepY = pose.y - previous_.y;
    const double translation = std::hypot(stepX, stepY);
    const double rotation = headingDelta(pose.headingDegrees, previous_.headingDegrees);
    const bool angular = mode_ == Mode::ANGULAR;
    const double absoluteStep = angular ? std::abs(rotation) : translation;
    const double signedStep = angular ? rotation : stepX * forwardX_ + stepY * forwardY_;
    const double sensorIntervalMs = std::max(std::uint32_t{1}, deltaMs);

    // Unwrap incrementally across 0/360, including every phase boundary. Lateral
    // position remains a projection onto the original heading, not path length.
    position_ = angular ? position_ + rotation : dx * forwardX_ + dy * forwardY_;
    MotionSample sample{timestampMs,
                        deltaMs,
                        position_,
                        deltaMs ? std::optional<double>(signedStep * 1000 / deltaMs) : std::nullopt,
                        deltaMs ? absoluteStep * 1000 / deltaMs : 0,
                        absoluteStep * 1000 / sensorIntervalMs,
                        translation * 1000 / sensorIntervalMs,
                        std::hypot(dx, dy),
                        dx * forwardY_ - dy * forwardX_,
                        headingDelta(pose.headingDegrees, origin_.headingDegrees)};
    previous_ = pose;
    previousTimeMs_ = timestampMs;
    return sample;
}

AutonTuneStatus motionStatus(const MotionSample& sample, const AutonTuneConfig& config) {
    const bool angular = config.mode == Mode::ANGULAR;
    const double translationLimit = angular ? config.maxCrossTrackInches : config.maxTravelInches;
    const bool outsideTravel =
        sample.translationFromOrigin > translationLimit ||
        std::abs(sample.crossTrack) > config.maxCrossTrackInches ||
        (angular ? std::abs(sample.position) > config.maxTravelInches
                 : std::abs(sample.headingFromOrigin) > config.maxHeadingErrorDegrees);
    if (outsideTravel)
        return AutonTuneStatus::TRAVEL_LIMIT;

    const double translationSpeedLimit =
        angular ? config.maxTranslationalSensorSpeed : config.maxSensorSpeed;
    if (sample.translationSpeed > translationSpeedLimit ||
        sample.sensorSpeed > config.maxSensorSpeed)
        return AutonTuneStatus::SENSOR_FAULT;
    return AutonTuneStatus::COMPLETE;
}
} // namespace detail
} // namespace autotune
