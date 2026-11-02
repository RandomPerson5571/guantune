#include "autotune/detail/motion_tracking.hpp"
#include <cassert>
#include <cmath>
#include <limits>
using namespace autotune;

int main() {
    // Nonzero origins and a 90 degree starting heading project along +x.
    detail::MotionTracker lateral({10, 20, 90}, Mode::LATERAL, 100);
    auto sample = lateral.sample({11, 20, 90}, 110);
    assert(std::abs(sample.position - 1) < 1e-9);
    assert(sample.velocity && std::abs(*sample.velocity - 100) < 1e-9);
    sample = lateral.sample({11, 22, 90}, 130);
    assert(std::abs(sample.position - 1) < 1e-9);
    assert(std::abs(std::abs(sample.crossTrack) - 2) < 1e-9);
    AutonTuneConfig config;
    sample = lateral.sample({11, 25, 90}, 140);
    assert(detail::motionStatus(sample, config) == AutonTuneStatus::TRAVEL_LIMIT);

    // One tracker spans phases: wrapping and changes at a boundary are retained.
    detail::MotionTracker angular({0, 0, 350}, Mode::ANGULAR, 0xfffffff0u);
    sample = angular.sample({0, 0, 5}, 4);
    assert(sample.deltaMs == 20 && sample.position == 15);
    sample = angular.sample({0, 0, 10}, 14);
    assert(sample.position == 20 && sample.headingFromOrigin == 20);
    sample = angular.sample({0, 0, 350}, 114);
    assert(sample.position == 0 && sample.velocity && *sample.velocity < 0);
    // Same-tick jumps still trigger sensor limits, even without a velocity sample.
    sample = angular.sample({2, 0, 350}, 114);
    config.mode = Mode::ANGULAR;
    config.maxSensorSpeed = 400;
    assert(!sample.velocity);
    assert(detail::motionStatus(sample, config) == AutonTuneStatus::SENSOR_FAULT);
    assert(!detail::finitePose({0, 0, std::numeric_limits<double>::quiet_NaN()}));
}
