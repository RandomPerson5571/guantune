#include "support/physics_drive.hpp"

struct RecordingDrive : PhysicsDrive {
    unsigned motorReads = 0;
    bool invalidMotorReadings = false;
    double batteryMillivolts() override { return voltage; }
    std::vector<MotorTelemetry> motorTelemetry(const std::vector<std::int8_t>& ports) override {
        ++motorReads;
        std::vector<MotorTelemetry> readings;
        for (auto port : ports) {
            readings.push_back(invalidMotorReadings ? MotorTelemetry{port, NAN, INFINITY, NAN}
                                                    : MotorTelemetry{port, 3200, 900, 34.5});
        }
        return readings;
    }
};

void testRecordingAndMotorCadence() {
    for (bool enabled : {false, true}) {
        RecordingDrive drive;
        drive.friction = 0.2;
        drive.noise = 0;
        AutonTuneConfig config;
        config.enableTelemetry = enabled;
        config.telemetryMotorPorts = {-1, 2};
        config.telemetryPeriodMs = 30;
        unsigned probeCalls = 0;
        config.telemetryCommandProbe = [&] {
            ++probeCalls;
            return TelemetryCommand{12, 3500, 3600, false};
        };
        const auto result = SelfTuningAuton(drive, config).run();
        assert(result.status == AutonTuneStatus::COMPLETE);
        if (!enabled) {
            assert(result.telemetry.empty() && drive.motorReads == 0 && probeCalls == 0);
            continue;
        }
        assert(probeCalls == result.telemetry.size());
        assert(drive.motorReads == (result.telemetry.size() + 4) / 5);
        bool forward = false, coast = false, reverse = false, returnSettle = false;
        bool positiveVelocity = false, negativeVelocity = false;
        for (std::size_t index = 0; index < result.telemetry.size(); ++index) {
            const auto& sample = result.telemetry[index];
            assert(sample.motors.size() == (index % 5 == 0 ? 2u : 0u));
            if (!sample.motors.empty()) {
                assert(sample.motors.front().port == -1);
                assert(sample.motors.front().measuredMillivolts == 3200);
            }
            assert(sample.command.controllerOutput == 12 && sample.batteryMillivolts == 12000);
            forward |= sample.phase == TelemetryPhase::TRIAL;
            coast |= sample.phase == TelemetryPhase::COAST;
            reverse |= sample.phase == TelemetryPhase::RETURN;
            returnSettle |= sample.phase == TelemetryPhase::RETURN_SETTLE;
            positiveVelocity |= sample.velocity && *sample.velocity > 0;
            negativeVelocity |= sample.velocity && *sample.velocity < 0;
            if (index) {
                const auto& previous = result.telemetry[index - 1];
                if (previous.trial == sample.trial && previous.phase == sample.phase)
                    assert(static_cast<std::uint32_t>(sample.timestampMs - previous.timestampMs) >=
                           30);
            }
        }
        assert(forward && coast && reverse && returnSettle && positiveVelocity && negativeVelocity);
    }
}

void testRecordingBoundsAndInvalidReadings() {
    RecordingDrive drive;
    drive.friction = 0.2;
    drive.noise = 0;
    drive.invalidMotorReadings = true;
    AutonTuneConfig config;
    config.enableTelemetry = true;
    config.telemetryMotorPorts = {1};
    config.maxTelemetrySamples = 3;
    const auto result = SelfTuningAuton(drive, config).run();
    assert(result.status == AutonTuneStatus::COMPLETE && result.telemetryTruncated);
    assert(result.telemetry.size() == 3 && drive.motorReads == 1);
    const auto& motor = result.telemetry.front().motors.front();
    assert(!motor.measuredMillivolts && !motor.currentMilliamps && !motor.temperatureCelsius);

    for (std::size_t invalid : {std::size_t{0}, std::size_t{100001}}) {
        RecordingDrive rejected;
        config.maxTelemetrySamples = invalid;
        assert(SelfTuningAuton(rejected, config).run().status == AutonTuneStatus::INVALID_CONFIG);
        assert(rejected.moves == 0);
    }
    config.maxTelemetrySamples = 4096;
    for (const auto& ports : {std::vector<std::int8_t>{0}, {22}, {-128}, {1, -1}}) {
        RecordingDrive rejected;
        config.telemetryMotorPorts = ports;
        assert(SelfTuningAuton(rejected, config).run().status == AutonTuneStatus::INVALID_CONFIG);
        assert(rejected.moves == 0);
    }
}

void testWrapAndKnownCommands() {
    for (auto mode : {Mode::LATERAL, Mode::ANGULAR}) {
        RecordingDrive drive;
        drive.friction = 0.2;
        drive.noise = 0;
        drive.time = 0xffffff00u;
        AutonTuneConfig config;
        config.enableTelemetry = true;
        config.mode = mode;
        const auto result = SelfTuningAuton(drive, config).run();
        assert(result.status == AutonTuneStatus::COMPLETE);
        bool wrapped = false, reverse = false;
        std::uint32_t previousElapsed = 0;
        for (const auto& sample : result.telemetry) {
            assert(sample.elapsedMs >= previousElapsed);
            previousElapsed = sample.elapsedMs;
            wrapped |= sample.timestampMs < 0xffffff00u;
            if (sample.phase == TelemetryPhase::TRIAL)
                assert(!sample.command.controllerOutput);
            if (sample.command.controllerOutput == -20) {
                reverse = true;
                assert(sample.command.voltageIsPowerEquivalent);
                assert(*sample.command.leftCommandedMillivolts < 0);
                assert(mode == Mode::ANGULAR ? *sample.command.rightCommandedMillivolts > 0
                                             : *sample.command.rightCommandedMillivolts < 0);
            }
        }
        assert(wrapped && reverse);
    }
}

void testThrowingProbeRestoresAndUnlocks() {
    RecordingDrive drive;
    drive.friction = 0.2;
    drive.noise = 0;
    bool shouldThrow = true;
    AutonTuneConfig config;
    config.enableTelemetry = true;
    config.telemetryCommandProbe = [&]() -> TelemetryCommand {
        if (shouldThrow)
            throw std::runtime_error("probe");
        return {};
    };
    SelfTuningAuton tuner(drive, config);
    try {
        tuner.run();
        assert(false);
    } catch (const std::runtime_error&) {
    }
    assert(!drive.active && !drive.swapped);
    shouldThrow = false;
    assert(tuner.run().status == AutonTuneStatus::COMPLETE);
}

void testCsvExport() {
    const auto directory =
        std::filesystem::temp_directory_path() /
        ("guantune-telemetry-" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    assert(std::filesystem::create_directory(directory));
    const auto path = (directory / "telemetry.csv").string();
    AutonTuneResult result;
    assert(!saveTelemetryCsv(result, path) && !std::filesystem::exists(path));
    TelemetryRecord sample;
    sample.batteryMillivolts = 12000;
    sample.motors = {{-1, 3200, 900, 34.5}, {2, std::nullopt, std::nullopt, std::nullopt}};
    result.telemetry.push_back(sample);
    result.status = AutonTuneStatus::SENSOR_FAULT;
    assert(saveTelemetryCsv(result, path));
    std::ifstream file(path);
    std::string header, first, second;
    std::getline(file, header);
    std::getline(file, first);
    std::getline(file, second);
    assert(std::count(header.begin(), header.end(), ',') == 20);
    assert(std::count(first.begin(), first.end(), ',') == 20);
    assert(std::count(second.begin(), second.end(), ',') == 20);
    assert(first.find(",12000,-1,3200,900,34.5,") != std::string::npos);
    assert(second.find(",12000,2,,,,") != std::string::npos);
    assert(!saveTelemetryCsv(result, directory.string()));
    assert(!std::filesystem::exists(directory.string() + ".tmp"));
    file.close();
    assert(std::filesystem::remove(path) && std::filesystem::remove(directory));
}

int main() {
    testRecordingAndMotorCadence();
    testRecordingBoundsAndInvalidReadings();
    testWrapAndKnownCommands();
    testThrowingProbeRestoresAndUnlocks();
    testCsvExport();
    std::cout << "Bounded telemetry, motor cadence, commands and CSV tests passed\n";
}
