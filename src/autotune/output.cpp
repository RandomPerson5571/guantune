#include "autotune/output.hpp"
#include "autotune/pid_validation.hpp"
#include "file_output_internal.hpp"
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>

namespace autotune {

bool accepted(const AutonTuneResult& result) {
    return result.status == AutonTuneStatus::COMPLETE && detail::validPidGains(result.gains) &&
           detail::validPidScalar(result.integralWindupRange) && result.integralWindupRange > 0;
}

bool saveAcceptedGains(const AutonTuneResult& result, const std::string& path) {
    if (!accepted(result))
        return false;
    return detail::writeAtomically(path, [&](std::ostream& file) {
        file << "{\"schema\":2,\"mode\":\""
             << (result.mode == Mode::ANGULAR ? "angular" : "lateral")
             << "\",\"kP\":" << result.gains.kP << ",\"kI\":" << result.gains.kI
             << ",\"kD\":" << result.gains.kD << ",\"windupRange\":" << result.integralWindupRange
             << "}\n";
    });
}

std::string controllerSettingsSnippet(const AutonTuneResult& result) {
    if (!accepted(result))
        return {};
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::setprecision(17) << "lemlib::ControllerSettings tunedSettings = baseSettings;\n"
        << "tunedSettings.kP = " << result.gains.kP << ";\n"
        << "tunedSettings.kI = " << result.gains.kI << ";\n"
        << "tunedSettings.kD = " << result.gains.kD << ";\n"
        << "tunedSettings.windupRange = " << result.integralWindupRange << ";\n";
    return out.str();
}

} // namespace autotune
