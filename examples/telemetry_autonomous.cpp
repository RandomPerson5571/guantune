// Call after calibration with exclusive drivetrain access. Pass the signed ports
// of the motors in this chassis; the example does not configure motors itself.
#include "autotune/api.hpp"
#include <cstdio>

autotune::AutonTuneResult tuneWithTelemetry(
    lemlib::Chassis& chassis, const std::vector<std::int8_t>& motorPorts,
    std::function<autotune::TelemetryCommand()> commandProbe = {}) {
    autotune::AutonTuneConfig config;
    config.enableTelemetry = true;
    config.telemetryMotorPorts = motorPorts;
    config.telemetryPeriodMs = 20;
    config.maxTelemetrySamples = 4096;
    config.telemetryCommandProbe = std::move(commandProbe);
    auto result = autotune::tuneLateral(chassis, config);
    // Failed sessions are useful too. Write only after movement has stopped.
    if (!autotune::saveTelemetryCsv(result, "/usd/guantune-telemetry.csv"))
        std::printf("Telemetry export failed or no samples were recorded\n");
    if (result.telemetryTruncated) std::printf("Telemetry buffer filled before the run ended\n");
    return result;
}
