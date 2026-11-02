#pragma once

namespace autotune {
enum class Mode { LATERAL, ANGULAR };
struct Gains {
    double kP;
    double kI;
    double kD;
};
} // namespace autotune

namespace guantune = autotune;
