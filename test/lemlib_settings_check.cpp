// Run against real installed PROS/LemLib headers; no hardware objects are created.
#include "autotune/lemlib_tuning_drive.hpp"
#include <cassert>
#include <limits>

int main() {
    lemlib::ControllerSettings base(10, 0, 5, 0, 1, 100, 3, 500, 5);
    autotune::AutonTuneResult result;
    result.status = autotune::AutonTuneStatus::COMPLETE;
    result.gains = {4, 0.01, 8};
    result.integralWindupRange = 7.5;
    const auto converted = autotune::toLemLib(result, base);
    assert(converted && converted->kP == 4 && converted->kI == static_cast<float>(0.01) && converted->kD == 8);
    assert(converted->windupRange == 7.5);
    assert(converted->smallError == base.smallError && converted->smallErrorTimeout == base.smallErrorTimeout);
    assert(converted->largeError == base.largeError && converted->largeErrorTimeout == base.largeErrorTimeout);
    assert(converted->slew == base.slew && base.windupRange == 0);
    result.status = autotune::AutonTuneStatus::TRIAL_LIMIT;
    assert(!autotune::toLemLib(result, base));
    result.status = autotune::AutonTuneStatus::COMPLETE;
    for (double invalid : {1e100, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        result.integralWindupRange = invalid;
        assert(!autotune::toLemLib(result, base));
        result.integralWindupRange = 3;
        result.gains.kP = invalid;
        assert(!autotune::toLemLib(result, base));
        result.gains.kP = 4;
    }
}
