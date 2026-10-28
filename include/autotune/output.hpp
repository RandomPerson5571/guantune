#pragma once
#include "autotune/self_tuning_auton.hpp"
#include "autotune/pid_validation.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace autotune {
inline bool accepted(const AutonTuneResult& result) {
    return result.status == AutonTuneStatus::COMPLETE && detail::validPidGains(result.gains) &&
        detail::validPidScalar(result.integralWindupRange) && result.integralWindupRange > 0;
}
// Caller owns this path exclusively. On PROS use /usd/guantune.json with an SD
// card present. A failed write/rename returns false; never persists failed gains.
inline bool saveAcceptedGains(const AutonTuneResult& result, const std::string& path) {
    if (!accepted(result)) return false;
    const auto temporary = path + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    file.imbue(std::locale::classic());
    file << std::setprecision(17) << "{\"schema\":2,\"mode\":\""
         << (result.mode == Mode::ANGULAR ? "angular" : "lateral")
         << "\",\"kP\":" << result.gains.kP << ",\"kI\":" << result.gains.kI
         << ",\"kD\":" << result.gains.kD
         << ",\"windupRange\":" << result.integralWindupRange << "}\n";
    file.close();
    if (file.fail()) { std::remove(temporary.c_str()); return false; }
    if (std::rename(temporary.c_str(), path.c_str()) == 0) return true;
    std::remove(temporary.c_str());
    return false;
}
// baseSettings is the user's existing ControllerSettings, retaining exit/slew settings.
inline std::string controllerSettingsSnippet(const AutonTuneResult& result) {
    if (!accepted(result)) return {};
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17)
        << "lemlib::ControllerSettings tunedSettings = baseSettings;\n"
        << "tunedSettings.kP = " << result.gains.kP << ";\n"
        << "tunedSettings.kI = " << result.gains.kI << ";\n"
        << "tunedSettings.kD = " << result.gains.kD << ";\n"
        << "tunedSettings.windupRange = " << result.integralWindupRange << ";\n";
    return out.str();
}

inline const char* telemetryPhaseName(TelemetryPhase phase) {
    switch (phase) {
        case TelemetryPhase::TRIAL: return "trial";
        case TelemetryPhase::COAST: return "coast";
        case TelemetryPhase::RETURN: return "return";
        case TelemetryPhase::RETURN_SETTLE: return "return_settle";
    }
    return "unknown";
}

// Export failed sessions too. One row per motor per sample; absent readings are
// empty CSV fields. No file is opened when telemetry was disabled or empty.
inline bool saveTelemetryCsv(const AutonTuneResult& result, const std::string& path) {
    if (result.telemetry.empty()) return false;
    const auto temporary = path + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    file.imbue(std::locale::classic());
    file << std::setprecision(17)
         << "timestamp_ms,elapsed_ms,trial,mode,phase,position_unit,target_position,actual_position,"
            "velocity,velocity_unit,controller_output,left_commanded_mV,right_commanded_mV,"
            "command_voltage_is_power_equivalent,battery_mV,motor_port,measured_motor_mV,"
            "current_mA,temperature_C,session_status,telemetry_truncated\n";
    const auto value = [&](std::optional<double> v) {
        if (v && std::isfinite(*v)) file << *v;
    };
    for (const auto& sample : result.telemetry) {
        const auto row = [&](const MotorTelemetry* motor) {
            const bool angular = sample.mode == Mode::ANGULAR;
            file << sample.timestampMs << ',' << sample.elapsedMs << ',' << sample.trial << ','
                 << (angular ? "angular" : "lateral") << ',' << telemetryPhaseName(sample.phase) << ','
                 << (angular ? "deg" : "in") << ',';
            value(sample.targetPosition); file << ',';
            value(sample.actualPosition); file << ',';
            value(sample.velocity); file << ',' << (angular ? "deg/s" : "in/s") << ',';
            value(sample.command.controllerOutput); file << ',';
            value(sample.command.leftCommandedMillivolts); file << ',';
            value(sample.command.rightCommandedMillivolts); file << ','
                << sample.command.voltageIsPowerEquivalent << ',';
            value(sample.batteryMillivolts); file << ',';
            if (motor) file << static_cast<int>(motor->port);
            file << ','; value(motor ? motor->measuredMillivolts : std::nullopt);
            file << ','; value(motor ? motor->currentMilliamps : std::nullopt);
            file << ','; value(motor ? motor->temperatureCelsius : std::nullopt);
            file << ',' << static_cast<int>(result.status) << ',' << result.telemetryTruncated << '\n';
        };
        if (sample.motors.empty()) row(nullptr);
        else for (const auto& motor : sample.motors) row(&motor);
    }
    file.close();
    if (file.fail()) { std::remove(temporary.c_str()); return false; }
    if (std::rename(temporary.c_str(), path.c_str()) == 0) return true;
    std::remove(temporary.c_str());
    return false;
}
} // namespace autotune
