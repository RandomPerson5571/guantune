#pragma once

#include "autotune/types.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace autotune {
enum class TelemetryPhase { TRIAL, COAST, RETURN, RETURN_SETTLE };

struct MotorTelemetry {
    std::int8_t port = 0; // Signed PROS port; negative means reversed.
    std::optional<double> measuredMillivolts;
    std::optional<double> currentMilliamps;
    std::optional<double> temperatureCelsius;
};

struct TelemetryCommand {
    std::optional<double> controllerOutput; // Signed mode-axis output; motor power units.
    std::optional<double> leftCommandedMillivolts;
    std::optional<double> rightCommandedMillivolts;
    // True when voltage is the nominal 12000 * power / 127 equivalent of move()/tank().
    bool voltageIsPowerEquivalent = false;
};

struct TelemetryRecord {
    std::uint32_t timestampMs = 0; // Raw drive clock, wraps like PROS millis().
    std::uint32_t elapsedMs = 0; // Since session start, using wrap-safe subtraction.
    unsigned trial = 0; // Zero-based trial index.
    Mode mode = Mode::LATERAL;
    TelemetryPhase phase = TelemetryPhase::TRIAL;
    double targetPosition = 0; // Signed displacement from the fixed session origin.
    double actualPosition = 0; // Inches lateral; unwrapped degrees angular.
    std::optional<double> velocity; // Signed inches/s or degrees/s; unavailable at dt=0.
    std::optional<double> batteryMillivolts;
    TelemetryCommand command;
    std::vector<MotorTelemetry> motors;
};
} // namespace autotune
