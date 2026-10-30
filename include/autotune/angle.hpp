#pragma once

#include <cmath>

namespace autotune {
namespace detail {
// Absolute compass headings wrap; measured angular displacement stays unwrapped.
inline double normalizeHeading(double degrees) {
    const double wrapped = std::fmod(degrees, 360.0);
    if (wrapped == 0)
        return 0; // Canonicalize negative zero too.
    if (wrapped < 0) {
        const double positive = wrapped + 360.0;
        return positive < 360.0 ? positive : 0.0;
    }
    return wrapped;
}
// LemLib accepts float headings. A double just below 360 can round to 360
// during that conversion, so wrap once more at the controller's precision.
inline float normalizeHeadingFloat(double degrees) {
    const float heading = static_cast<float>(normalizeHeading(degrees));
    return heading == 360.0f ? 0.0f : heading;
}
} // namespace detail
} // namespace autotune
