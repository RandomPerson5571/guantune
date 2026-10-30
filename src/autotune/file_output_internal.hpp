#pragma once
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <locale>
#include <string>

namespace autotune {
namespace detail {
// Keep filesystem handling separate from JSON serialization. Only a fully written and closed
// temporary file may replace the destination. This is not a power-loss fsync.
template <typename Write> bool writeAtomically(const std::string& path, const Write& write) {
    const auto temporary = path + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    file.imbue(std::locale::classic());
    file << std::setprecision(17);
    write(file);
    file.close();
    if (file.fail()) {
        std::remove(temporary.c_str());
        return false;
    }
    if (std::rename(temporary.c_str(), path.c_str()) == 0)
        return true;
    std::remove(temporary.c_str());
    return false;
}

} // namespace detail
} // namespace autotune
