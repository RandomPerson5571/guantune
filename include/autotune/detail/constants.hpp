#pragma once

// Private implementation detail; not part of the installed public API.
#include <cstddef>
#include <cstdint>

namespace autotune {
namespace detail {
constexpr std::uint32_t controlPeriodMs = 10;
constexpr std::uint32_t motionStartupTimeoutMs = 250;
constexpr double initialKp = 1.1;
constexpr double initialKd = 1.1;
constexpr unsigned maxUnsettledRetries = 5;
constexpr std::size_t maxTelemetrySamplesCap = 100000;
} // namespace detail
} // namespace autotune
