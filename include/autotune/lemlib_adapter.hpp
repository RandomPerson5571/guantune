#pragma once

#include "autotune/types.hpp"
#include <lemlib/api.hpp>

namespace autotune {

// Adapter API proposal. No tasks, sensor reads, or motor writes exist yet.
class Tuner {
public:
    Tuner(lemlib::Chassis& chassis, Config config) = delete;
    Tuner(const Tuner&) = delete;
    Tuner& operator=(const Tuner&) = delete;
    Tuner(Tuner&&) = delete;
    Tuner& operator=(Tuner&&) = delete;

    Result run() = delete;
    void runAsync() = delete;
    void abort() = delete;
    bool isRunning() const = delete;
    Result result() const = delete;

private:
    lemlib::Chassis& chassis_;
    Config config_;
    // TODO: Define task ownership, synchronization, cleanup, and hardware access.
};

// Preserve the user's exit conditions, windup range, and slew settings.
// Planned contract: return no settings for incomplete/invalid results.
// Declaration only; gain/timebase conversion must be verified before implementing.
std::optional<lemlib::ControllerSettings> toLemLib(
    const Result& result, const lemlib::ControllerSettings& base) = delete;

} // namespace autotune
