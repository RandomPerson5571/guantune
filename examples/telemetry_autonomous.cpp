// Recording is optional and stays in memory. The caller chooses a transport.
#include "autotune/api.hpp"
#include <utility>

autotune::AutonTuneResult tuneWithTelemetry(
    lemlib::Chassis& chassis, const std::vector<std::int8_t>& motorPorts,
    std::function<autotune::TelemetryCommand()> commandProbe = {}) {
    autotune::AutonTuneConfig config;
    config.enableTelemetry = true;
    config.telemetryMotorPorts = motorPorts;
    config.telemetryPeriodMs = 20;
    config.maxTelemetrySamples = 4096;
    config.telemetryCommandProbe = std::move(commandProbe);
    // result.telemetry contains the samples, including those from failed runs.
    return autotune::tuneLateral(chassis, config);
}
