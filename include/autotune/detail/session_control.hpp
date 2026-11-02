#pragma once

// Private implementation detail; not part of the installed public API.
#include "autotune/lemlib_tuning_drive.hpp"

namespace autotune {
namespace detail {
// Best effort during exception unwinding: attempt restoration even if stop()
// throws. Hardware implementations must still make both operations nonthrowing;
// swallowing a cleanup exception cannot guarantee that hardware actually stopped.
struct DriveCleanup {
    LemLibTuningDrive& drive;
    ~DriveCleanup() noexcept {
        try {
            drive.stop();
        } catch (...) {
        }
        try {
            drive.restoreGains();
        } catch (...) {
        }
    }
};

// Deadlines alone cannot terminate loops when the drive's clock has stalled.
// Poll count is independent of that clock, while unsigned deltas handle wrap.
class ClockWatchdog {
  public:
    explicit ClockWatchdog(std::uint32_t nowMs) : lastMs_(nowMs) {}
    bool healthy(std::uint32_t nowMs);

  private:
    std::uint32_t lastMs_;
    unsigned stalledPolls_ = 0;
};
} // namespace detail
} // namespace autotune
