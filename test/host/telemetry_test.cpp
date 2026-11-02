#include "autotune/detail/telemetry_recorder.hpp"
#include <cassert>
#include <limits>
using namespace autotune;

int main() {
    AutonTuneConfig config;
    config.enableTelemetry = true;
    config.telemetryMotorPorts = {1};
    config.maxTelemetrySamples = 7;
    AutonTuneResult result;
    unsigned motorReads = 0;
    detail::TelemetryRecorder recorder(config, result, 0xfffffff0u, [&] {
        ++motorReads;
        MotorTelemetry reading;
        reading.port = 1;
        reading.measuredMillivolts = std::numeric_limits<double>::quiet_NaN();
        return std::vector<MotorTelemetry>{reading};
    });
    detail::MotionTracker motion({0, 0, 0}, Mode::LATERAL, 0xfffffff0u);
    auto sample = motion.sample({0, 1, 0}, 4);
    recorder.beginTrial(0);
    recorder.record(sample, TelemetryPhase::TRIAL, 24, 12000);
    assert(result.telemetry.size() == 1 && result.telemetry[0].elapsedMs == 20);
    assert(motorReads == 1 && !result.telemetry[0].motors[0].measuredMillivolts);
    recorder.record(sample, TelemetryPhase::TRIAL, 24, 12000);
    assert(result.telemetry.size() == 1); // Cadence suppresses duplicate samples.
    recorder.record(sample, TelemetryPhase::COAST, 24, 12000, detail::stoppedCommand());
    assert(result.telemetry.size() == 2); // Phase changes always record.
    assert(result.telemetry.back().motors.empty());
    for (unsigned i = 1; i <= 6; ++i) {
        sample = motion.sample({0, 1, 0}, 4 + i * 20);
        recorder.record(sample, TelemetryPhase::COAST, 24, 12000);
    }
    assert(result.telemetry.size() == 7 && result.telemetryTruncated);
    assert(motorReads == 2); // Fresh motor I/O every fifth recorded sample.
    config.mode = Mode::ANGULAR;
    const auto command = detail::returnCommand(config);
    assert(command.leftCommandedMillivolts && command.rightCommandedMillivolts);
    assert(*command.leftCommandedMillivolts == -*command.rightCommandedMillivolts);
    assert(command.voltageIsPowerEquivalent);
}
