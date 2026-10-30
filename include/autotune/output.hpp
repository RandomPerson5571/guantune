#pragma once

#include "autotune/self_tuning_auton.hpp"
#include <string>

namespace autotune {
// COMPLETE plus finite, representable PID parameters is required for persistence.
bool accepted(const AutonTuneResult& result);

// Writes schema 2 JSON via a temporary file and rename. The caller owns the
// path exclusively. Failed/unaccepted runs never replace a saved result.
bool saveAcceptedGains(const AutonTuneResult& result, const std::string& path);

// Copy-pasteable gain/windup assignments; exit thresholds and slew stay with
// the caller's baseSettings. Returns an empty string for an unaccepted result.
std::string controllerSettingsSnippet(const AutonTuneResult& result);

const char* telemetryPhaseName(TelemetryPhase phase);
// Write only after run() returns. Failed sessions may be exported as diagnostics;
// absent readings become empty CSV cells. No samples means no output file.
bool saveTelemetryCsv(const AutonTuneResult& result, const std::string& path);
} // namespace autotune
