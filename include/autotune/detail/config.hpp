#pragma once

// Private implementation detail; not part of the installed public API.
#include "autotune/tuning_config.hpp"

namespace autotune {
namespace detail {
// Select only the current mode's targets, shared by validation and execution.
const std::vector<double>& configuredTargets(const AutonTuneConfig& config);
double configuredTarget(const AutonTuneConfig& config);
const char* validationError(const AutonTuneConfig& config);
} // namespace detail
} // namespace autotune
