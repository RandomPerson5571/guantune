#include "session_internal.hpp"
#include <algorithm>
#include <cmath>

namespace autotune {
namespace detail {
namespace {
std::optional<double> finiteReading(std::optional<double> value) {
    return value && std::isfinite(*value) ? value : std::nullopt;
}
constexpr std::size_t reserveCap = 4096;
constexpr unsigned motorSampleEvery = 5;
} // namespace

TelemetryCommand stoppedCommand() { return {0, 0, 0, false}; }
TelemetryCommand returnCommand(const AutonTuneConfig& config) {
    const double voltage = config.returnPower * 12000 / 127;
    return {-config.returnPower, -voltage, config.mode == Mode::ANGULAR ? voltage : -voltage, true};
}

TelemetryRecorder::TelemetryRecorder(TuningDrive& drive, const AutonTuneConfig& config,
                                     AutonTuneResult& result)
    : drive_(drive), config_(config), result_(result),
      beginMs_(config.enableTelemetry ? drive.nowMs() : 0) {
    if (config.enableTelemetry)
        result_.telemetry.reserve(std::min(config.maxTelemetrySamples, reserveCap));
}

void TelemetryRecorder::beginTrial(unsigned index) {
    trialIndex_ = index;
    lastPhase_.reset(); // Preserve a new trial's first record even within one period.
}

void TelemetryRecorder::record(const MotionSample& sample, TelemetryPhase phase, double target,
                               double battery, TelemetryCommand command) {
    if (!config_.enableTelemetry)
        return;
    if (lastSampleMs_ && lastPhase_ == phase &&
        static_cast<std::uint32_t>(sample.timestampMs - *lastSampleMs_) < config_.telemetryPeriodMs)
        return;
    if (result_.telemetry.size() >= config_.maxTelemetrySamples) {
        result_.telemetryTruncated = true;
        return;
    }
    TelemetryRecord record;
    record.timestampMs = sample.timestampMs;
    record.elapsedMs = static_cast<std::uint32_t>(sample.timestampMs - beginMs_);
    record.trial = trialIndex_;
    record.mode = config_.mode;
    record.phase = phase;
    record.targetPosition = target;
    record.actualPosition = sample.position;
    record.velocity = finiteReading(sample.velocity);
    record.batteryMillivolts = finiteReading(battery);
    record.command = config_.telemetryCommandProbe ? config_.telemetryCommandProbe() : command;
    record.command.controllerOutput = finiteReading(record.command.controllerOutput);
    record.command.leftCommandedMillivolts = finiteReading(record.command.leftCommandedMillivolts);
    record.command.rightCommandedMillivolts =
        finiteReading(record.command.rightCommandedMillivolts);
    // Avoid device I/O on every recorded pose. Empty motor arrays mean no fresh
    // motor reading; never carry old measurements forward as if they were new.
    if (!config_.telemetryMotorPorts.empty() && recordCount_ % motorSampleEvery == 0)
        record.motors = drive_.motorTelemetry(config_.telemetryMotorPorts);
    for (auto& motor : record.motors) {
        motor.measuredMillivolts = finiteReading(motor.measuredMillivolts);
        motor.currentMilliamps = finiteReading(motor.currentMilliamps);
        motor.temperatureCelsius = finiteReading(motor.temperatureCelsius);
    }
    result_.telemetry.push_back(std::move(record));
    ++recordCount_;
    lastSampleMs_ = sample.timestampMs;
    lastPhase_ = phase;
}
} // namespace detail
} // namespace autotune
