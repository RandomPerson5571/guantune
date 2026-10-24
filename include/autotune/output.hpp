#pragma once
#include "autotune/self_tuning_auton.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>

namespace autotune {
inline bool accepted(const AutonTuneResult& result) {
    return result.status == AutonTuneStatus::COMPLETE && std::isfinite(result.gains.kP) &&
        std::isfinite(result.gains.kI) && std::isfinite(result.gains.kD);
}
// Caller owns this path exclusively. On PROS use /usd/guantune.json with an SD
// card present. A failed write/rename returns false; never persists failed gains.
inline bool saveAcceptedGains(const AutonTuneResult& result, const std::string& path) {
    if (!accepted(result)) return false;
    const auto temporary = path + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    file.imbue(std::locale::classic());
    file << std::setprecision(17) << "{\"schema\":1,\"mode\":\""
         << (result.mode == Mode::ANGULAR ? "angular" : "lateral")
         << "\",\"kP\":" << result.gains.kP << ",\"kI\":" << result.gains.kI
         << ",\"kD\":" << result.gains.kD << "}\n";
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
        << "tunedSettings.kD = " << result.gains.kD << ";\n";
    return out.str();
}
} // namespace autotune
