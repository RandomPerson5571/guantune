#include "autotune/output.hpp"
#include "file_output_internal.hpp"
#include <cmath>
#include <ostream>

namespace autotune {
namespace {
void writeOptional(std::ostream& file, std::optional<double> value) {
    if (value && std::isfinite(*value))
        file << *value;
}

void writeRow(std::ostream& file, const TelemetryRecord& sample, const MotorTelemetry* motor,
              const AutonTuneResult& result) {
    const bool angular = sample.mode == Mode::ANGULAR;
    file << sample.timestampMs << ',' << sample.elapsedMs << ',' << sample.trial << ','
         << (angular ? "angular" : "lateral") << ',' << telemetryPhaseName(sample.phase) << ','
         << (angular ? "deg" : "in") << ',';
    writeOptional(file, sample.targetPosition);
    file << ',';
    writeOptional(file, sample.actualPosition);
    file << ',';
    writeOptional(file, sample.velocity);
    file << ',' << (angular ? "deg/s" : "in/s") << ',';
    writeOptional(file, sample.command.controllerOutput);
    file << ',';
    writeOptional(file, sample.command.leftCommandedMillivolts);
    file << ',';
    writeOptional(file, sample.command.rightCommandedMillivolts);
    file << ',' << sample.command.voltageIsPowerEquivalent << ',';
    writeOptional(file, sample.batteryMillivolts);
    file << ',';
    if (motor)
        file << static_cast<int>(motor->port);
    file << ',';
    writeOptional(file, motor ? motor->measuredMillivolts : std::nullopt);
    file << ',';
    writeOptional(file, motor ? motor->currentMilliamps : std::nullopt);
    file << ',';
    writeOptional(file, motor ? motor->temperatureCelsius : std::nullopt);
    file << ',' << static_cast<int>(result.status) << ',' << result.telemetryTruncated << '\n';
}
} // namespace

const char* telemetryPhaseName(TelemetryPhase phase) {
    switch (phase) {
    case TelemetryPhase::TRIAL:
        return "trial";
    case TelemetryPhase::COAST:
        return "coast";
    case TelemetryPhase::RETURN:
        return "return";
    case TelemetryPhase::RETURN_SETTLE:
        return "return_settle";
    }
    return "unknown";
}

bool saveTelemetryCsv(const AutonTuneResult& result, const std::string& path) {
    if (result.telemetry.empty())
        return false;
    return detail::writeAtomically(path, [&](std::ostream& file) {
        file << "timestamp_ms,elapsed_ms,trial,mode,phase,position_unit,target_position,actual_"
                "position,"
                "velocity,velocity_unit,controller_output,left_commanded_mV,right_commanded_mV,"
                "command_voltage_is_power_equivalent,battery_mV,motor_port,measured_motor_mV,"
                "current_mA,temperature_C,session_status,telemetry_truncated\n";
        for (const auto& sample : result.telemetry) {
            if (sample.motors.empty())
                writeRow(file, sample, nullptr, result);
            else
                for (const auto& motor : sample.motors)
                    writeRow(file, sample, &motor, result);
        }
    });
}
} // namespace autotune
