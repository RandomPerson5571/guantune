#include "autotune/angle.hpp"
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <limits>

int main() {
    using autotune::detail::normalizeHeading;
    assert(normalizeHeading(0) == 0);
    assert(normalizeHeading(360) == 0);
    assert(normalizeHeading(720) == 0);
    assert(normalizeHeading(-720) == 0);
    assert(normalizeHeading(-260) == 100);
    assert(normalizeHeading(350 + 90) == 80);
    assert(normalizeHeading(-260 + 90) == 190);
    assert(normalizeHeading(719.5) == 359.5);
    assert(normalizeHeading(-360.5) == 359.5);
    assert(!std::signbit(normalizeHeading(-0.0)));
    using autotune::detail::normalizeHeadingFloat;
    assert(normalizeHeadingFloat(720) == 0);
    assert(normalizeHeadingFloat(-260) == 100);
    for (double angle : {359.999999, -0.000001, std::nextafter(360.0, 0.0)}) {
        const float heading = normalizeHeadingFloat(angle);
        assert(heading >= 0 && heading < 360);
        // Compare at float precision: converting before the final wrap can
        // produce exactly 360, which must become zero for LemLib.
        assert(heading == 0);
    }
    for (double angle : {-1e12, -720., -260., -std::numeric_limits<double>::denorm_min(), 0.,
                         359.99, 360., 720., 1e12}) {
        const double wrapped = normalizeHeading(angle);
        assert(wrapped >= 0 && wrapped < 360);
        assert(std::abs(std::remainder(wrapped - angle, 360.0)) < 1e-6);
    }
}
