#pragma once

#include "autotune/types.hpp"
#include <cmath>
#include <limits>

namespace autotune {
namespace detail {
// LemLib stores PID parameters as floats, even though the public tuner uses doubles.
inline bool validPidScalar(double value) {
    return std::isfinite(value) && value >= 0 && value <= std::numeric_limits<float>::max() &&
           (value == 0 || static_cast<float>(value) > 0);
}
inline bool validPidGains(const Gains& gains) {
    return validPidScalar(gains.kP) && validPidScalar(gains.kI) && validPidScalar(gains.kD);
}
} // namespace detail
} // namespace autotune
