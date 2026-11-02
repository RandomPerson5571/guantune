#pragma once

// Private implementation detail; not part of the installed public API.
#include "motion_tracking.hpp"
#include <functional>

namespace autotune {
namespace detail {
TelemetryCommand stoppedCommand();
TelemetryCommand returnCommand(const AutonTuneConfig& config);
class TelemetryRecorder {
  public:
    using MotorProbe = std::function<std::vector<MotorTelemetry>()>;
    TelemetryRecorder(const AutonTuneConfig& config, AutonTuneResult& result,
                      std::uint32_t beginMs, MotorProbe motors = {});
    void beginTrial(unsigned index);
    void record(const MotionSample& sample, TelemetryPhase phase, double target, double battery,
                TelemetryCommand command = {});

  private:
    MotorProbe motors_;
    const AutonTuneConfig& config_;
    AutonTuneResult& result_;
    std::uint32_t beginMs_;
    std::optional<std::uint32_t> lastSampleMs_;
    std::optional<TelemetryPhase> lastPhase_;
    unsigned trialIndex_ = 0;
    std::size_t recordCount_ = 0;
};
} // namespace detail
} // namespace autotune
