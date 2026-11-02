#include "autotune/detail/config.hpp"
#include <cassert>
#include <limits>
using namespace autotune;

int main() {
    assert(detail::validationError(AutonTuneConfig{}) == nullptr);
    for (Mode mode : {Mode::LATERAL, Mode::ANGULAR}) {
        AutonTuneConfig config;
        config.mode = mode;
        auto& scalar = mode == Mode::ANGULAR ? config.angleDegrees : config.distanceInches;
        auto& targets = mode == Mode::ANGULAR ? config.testAngles : config.testDistances;
        (mode == Mode::ANGULAR ? config.distanceInches : config.angleDegrees) =
            std::numeric_limits<double>::quiet_NaN();
        (mode == Mode::ANGULAR ? config.testDistances : config.testAngles) = {0, 999};
        scalar = 6;
        assert(detail::validationError(config) == nullptr);
        assert(detail::configuredTarget(config) == 6);
        targets = {6, 12};
        scalar = std::numeric_limits<double>::quiet_NaN();
        assert(detail::validationError(config) == nullptr);
        assert(detail::configuredTargets(config) == targets);
        config.maxTrials = 3;
        assert(detail::validationError(config)); // Two targets need four confirmations.
        config.maxTrials = 20;
        for (double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), 999.0}) {
            targets = {6, invalid};
            assert(detail::validationError(config));
            targets.clear(); scalar = invalid;
            assert(detail::validationError(config));
        }
    }
    AutonTuneConfig seed;
    seed.startingGains = {4, 0, 0};
    assert(detail::validationError(seed) == nullptr);
    seed.startingGains.kI = 0.001;
    assert(detail::validationError(seed));
    seed.allowIntegral = true;
    assert(detail::validationError(seed) == nullptr);
    seed.startingGains.kP = 31;
    assert(detail::validationError(seed));
    seed.startingGains.kP = 4;
    seed.maxGainChangeRatio = 0;
    assert(detail::validationError(seed));
    AutonTuneConfig config;
    config.mode = Mode::ANGULAR;
    config.maxTravelInches = 220;
    config.angleDegrees = 180;
    assert(detail::validationError(config));
    config.angleDegrees = 170;
    assert(detail::validationError(config) == nullptr);
    config.postStopMs = config.returnTimeoutMs;
    assert(detail::validationError(config));
    config.postStopMs = 300;
    config.maxKp = 1e100;
    assert(detail::validationError(config));
    config.maxKp = 30;
    config.enableTelemetry = true;
    config.telemetryMotorPorts = {1, -1};
    assert(detail::validationError(config));
}
