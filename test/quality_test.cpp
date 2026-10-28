#include "autotune/self_tuning_auton.hpp"
#include "autotune/idle_pid_guard.hpp"
#include "autotune/output.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <filesystem>
#include <chrono>
#include <limits>
using namespace autotune;

// First-order motor response with static friction, saturation, braking and noise.
// This is an independent model; there are no scripted success poses.
struct PhysicsDrive : TuningDrive {
    double position = 0, velocity = 0, target = 0, previousError = 0;
    double response = 0.08, friction = 2, noise = 0.002, voltage = 12000;
    double command = 0, maximum = 60;
    bool active = false, manual = false, swapped = false, ready = true;
    Mode mode = Mode::LATERAL;
    Gains gains{10, 0, 5};
    std::uint32_t time = 0;
    unsigned moves = 0;
    int throwAt = 0;
    std::uint32_t sagAt = 0;
    DrivePose pose() override {
        if (throwAt == 2 && active) throw std::runtime_error("pose");
        const double measured = position + noise * std::sin(time * 0.017);
        return mode == Mode::LATERAL ? DrivePose{0, measured, 0} : DrivePose{0, 0, measured};
    }
    void setTrialGains(const Gains& g, double) override { assert(!active); gains = g; swapped = true; }
    void restoreGains() override { assert(!active); gains = {10, 0, 5}; swapped = false; }
    void moveTo(const DrivePose& p, bool, std::uint32_t, double output) override { begin(p.y, output); }
    void turnTo(double angle, std::uint32_t, double output) override { begin(angle, output); }
    void begin(double t, double output) {
        target = t; maximum = output; previousError = target - position;
        active = true; manual = false; ++moves;
        if (throwAt == 1) throw std::runtime_error("move");
    }
    bool supportsAngular() const override { return true; }
    void selectMode(Mode m) override { mode = m; }
    void turnBack(double power) override { driveBackward(power); }
    void driveBackward(double power) override { command = -power; active = true; manual = true; }
    bool isMoving() override { return active; }
    void stop() override { active = false; command = 0; }
    std::uint32_t nowMs() override { return time; }
    void delayMs(std::uint32_t ms) override {
        if (throwAt == 3 && active) throw std::runtime_error("delay");
        for (std::uint32_t i = 0; i < ms; ++i) {
            if (active && !manual && (time + i) % 10 == 0) {
                const double error = target - position;
                command = std::clamp(gains.kP * error + gains.kD * (error - previousError), -maximum, maximum);
                previousError = error;
            }
            const double effective = std::copysign(std::max(0.0, std::abs(command) - friction), command);
            const double desired = effective * 0.7 * voltage / 12000;
            const double tau = active ? response : 0.02;
            velocity += (desired - velocity) * 0.001 / tau;
            position += velocity * 0.001;
        }
        time += ms;
        if (sagAt && time >= sagAt) voltage = 6500;
    }
    bool available() override { return true; }
    bool calibrated() override { return ready; }
    double batteryMillivolts() override { return voltage; }
};

struct ConstPid {
    const double p;
    explicit ConstPid(double value) : p(value) {}
};

struct FrozenReturnDrive : PhysicsDrive {
    double lastForward = 0;
    DrivePose pose() override {
        if (!manual) lastForward = position;
        return {0, lastForward, 0};
    }
};

struct AngularReturnJumpDrive : PhysicsDrive {
    bool jumped = false;
    DrivePose pose() override {
        auto p = PhysicsDrive::pose();
        if (manual) jumped = true;
        if (jumped) p.x = 3;
        return p;
    }
};

struct WrappedPhysicsDrive : PhysicsDrive {
    DrivePose pose() override {
        auto p = PhysicsDrive::pose();
        p.headingDegrees = std::fmod(p.headingDegrees + 360, 360);
        return p;
    }
};

struct InitialReturnJumpDrive : PhysicsDrive {
    unsigned returnPoses = 0;
    bool returning = false;
    void restoreGains() override {
        if (swapped) returning = true;
        PhysicsDrive::restoreGains();
    }
    DrivePose pose() override {
        auto p = PhysicsDrive::pose();
        if (returning && ++returnPoses >= 2) p.x = 3;
        return p;
    }
};

struct TelemetryPhysicsDrive : PhysicsDrive {
    unsigned telemetryReads = 0;
    bool invalidTelemetry = false;
    std::vector<MotorTelemetry> motorTelemetry(const std::vector<std::int8_t>& ports) override {
        ++telemetryReads;
        std::vector<MotorTelemetry> samples;
        for (auto port : ports) {
            if (invalidTelemetry) samples.push_back({port, NAN, INFINITY, NAN});
            else samples.push_back({port, 3200, 900, 34.5});
        }
        return samples;
    }
};

int main() {
    for (bool enabled : {false, true}) {
        TelemetryPhysicsDrive d; d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c; c.enableTelemetry = enabled; c.telemetryMotorPorts = {-1, 2};
        c.telemetryPeriodMs = 30;
        unsigned probeReads = 0;
        c.telemetryCommandProbe = [&] { ++probeReads; return TelemetryCommand{12, 3500, 3600, false}; };
        const auto result = SelfTuningAuton(d, c).run();
        assert(result.status == AutonTuneStatus::COMPLETE && !result.telemetryTruncated);
        if (!enabled) {
            assert(result.telemetry.empty() && d.telemetryReads == 0 && probeReads == 0);
            continue;
        }
        assert(!result.telemetry.empty() && d.telemetryReads == result.telemetry.size());
        assert(probeReads == result.telemetry.size());
        bool forward = false, coast = false, backwards = false, settledReturn = false;
        bool positiveVelocity = false, negativeVelocity = false, secondTrial = false;
        const TelemetryRecord* previous = nullptr;
        for (const auto& sample : result.telemetry) {
            assert(sample.batteryMillivolts == 12000 && sample.motors.size() == 2);
            assert(sample.motors[0].port == -1 && sample.motors[1].port == 2);
            assert(sample.motors[0].measuredMillivolts == 3200);
            assert(sample.motors[1].currentMilliamps == 900 && sample.motors[1].temperatureCelsius == 34.5);
            assert(sample.command.controllerOutput == 12 && sample.command.leftCommandedMillivolts == 3500);
            assert(sample.command.rightCommandedMillivolts == 3600 && !sample.command.voltageIsPowerEquivalent);
            const bool returning = sample.phase == TelemetryPhase::RETURN || sample.phase == TelemetryPhase::RETURN_SETTLE;
            assert(sample.targetPosition == (returning ? 0 : 24));
            forward |= sample.phase == TelemetryPhase::TRIAL;
            coast |= sample.phase == TelemetryPhase::COAST;
            backwards |= sample.phase == TelemetryPhase::RETURN;
            settledReturn |= sample.phase == TelemetryPhase::RETURN_SETTLE;
            positiveVelocity |= sample.velocity && *sample.velocity > 0;
            negativeVelocity |= sample.velocity && *sample.velocity < 0;
            secondTrial |= sample.trial > 0;
            if (previous && previous->phase == sample.phase && previous->trial == sample.trial)
                assert(static_cast<std::uint32_t>(sample.timestampMs - previous->timestampMs) >= 30);
            previous = &sample;
        }
        assert(forward && coast && backwards && settledReturn && positiveVelocity && negativeVelocity && secondTrial);
        assert(!result.telemetry.front().velocity);
    }
    for (Mode mode : {Mode::LATERAL, Mode::ANGULAR}) {
        TelemetryPhysicsDrive d; d.friction = 0.2; d.noise = 0; d.time = 0xffffff00u;
        AutonTuneConfig c; c.enableTelemetry = true; c.mode = mode;
        const auto result = SelfTuningAuton(d, c).run();
        assert(result.status == AutonTuneStatus::COMPLETE && d.telemetryReads == 0);
        bool manualCommand = false, zeroCommand = false, wrapped = false;
        std::uint32_t lastElapsed = 0;
        for (const auto& sample : result.telemetry) {
            assert(sample.mode == mode && sample.elapsedMs >= lastElapsed);
            lastElapsed = sample.elapsedMs;
            wrapped |= sample.timestampMs < 0xffffff00u;
            if (sample.phase == TelemetryPhase::TRIAL) assert(!sample.command.controllerOutput);
            if (sample.phase == TelemetryPhase::RETURN && sample.command.controllerOutput == -20) {
                manualCommand = true;
                assert(sample.command.voltageIsPowerEquivalent);
                const double voltage = 20.0 * 12000 / 127;
                assert(sample.command.leftCommandedMillivolts == -voltage);
                assert(sample.command.rightCommandedMillivolts == (mode == Mode::ANGULAR ? voltage : -voltage));
            }
            if (sample.phase == TelemetryPhase::COAST || sample.phase == TelemetryPhase::RETURN_SETTLE) {
                zeroCommand = true;
                assert(sample.command.controllerOutput == 0 && sample.command.leftCommandedMillivolts == 0);
            }
        }
        assert(manualCommand && zeroCommand && wrapped);
    }
    {
        TelemetryPhysicsDrive d; d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c; c.enableTelemetry = true; c.maxTelemetrySamples = 3; c.telemetryMotorPorts = {1};
        const auto result = SelfTuningAuton(d, c).run();
        assert(result.status == AutonTuneStatus::COMPLETE && result.telemetry.size() == 3);
        assert(result.telemetryTruncated && d.telemetryReads == 3 && !d.active && !d.swapped);
    }
    {
        TelemetryPhysicsDrive d; d.invalidTelemetry = true; d.sagAt = 100;
        AutonTuneConfig c; c.enableTelemetry = true; c.telemetryMotorPorts = {1};
        const auto result = SelfTuningAuton(d, c).run();
        assert(result.status == AutonTuneStatus::LOW_BATTERY && !result.telemetry.empty());
        assert(result.telemetry.back().batteryMillivolts == 6500);
        const auto& motor = result.telemetry.front().motors[0];
        assert(!motor.measuredMillivolts && !motor.currentMilliamps && !motor.temperatureCelsius);
    }
    for (const auto& ports : {std::vector<std::int8_t>{0}, {22}, {-128}, {1, -1}}) {
        TelemetryPhysicsDrive d; AutonTuneConfig c; c.enableTelemetry = true; c.telemetryMotorPorts = ports;
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::INVALID_CONFIG && d.moves == 0);
    }
    for (std::uint32_t period : {0u, 9u, 0x80000000u}) {
        TelemetryPhysicsDrive d; AutonTuneConfig c; c.enableTelemetry = true; c.telemetryPeriodMs = period;
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::INVALID_CONFIG && d.moves == 0);
    }
    {
        TelemetryPhysicsDrive d; AutonTuneConfig c; c.enableTelemetry = true; c.maxTelemetrySamples = 0;
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::INVALID_CONFIG && d.moves == 0);
    }
    {
        TelemetryPhysicsDrive d; AutonTuneConfig c; c.enableTelemetry = true;
        c.telemetryCommandProbe = []() -> TelemetryCommand { throw std::runtime_error("telemetry"); };
        try { SelfTuningAuton(d, c).run(); assert(false); }
        catch (const std::runtime_error&) {}
        assert(!d.active && !d.swapped);
    }
    {
        const auto directory = std::filesystem::temp_directory_path() /
            ("guantune-telemetry-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        assert(std::filesystem::create_directory(directory));
        const auto path = (directory / "telemetry.csv").string();
        AutonTuneResult result;
        assert(!saveTelemetryCsv(result, path) && !std::filesystem::exists(path));
        TelemetryRecord sample; sample.command = {12, 3500, 3600, false};
        sample.actualPosition = 1; sample.batteryMillivolts = 12000;
        sample.motors = {{-1, 3200, 900, 34.5}, {2, std::nullopt, std::nullopt, std::nullopt}};
        result.telemetry.push_back(sample); result.status = AutonTuneStatus::SENSOR_FAULT;
        result.telemetryTruncated = true;
        assert(saveTelemetryCsv(result, path));
        std::ifstream file(path); std::string header, first, second;
        std::getline(file, header); std::getline(file, first); std::getline(file, second);
        assert(std::count(header.begin(), header.end(), ',') == 20);
        assert(std::count(first.begin(), first.end(), ',') == 20 && std::count(second.begin(), second.end(), ',') == 20);
        assert(header.find("measured_motor_mV,current_mA,temperature_C") != std::string::npos);
        assert(first.find(",12000,-1,3200,900,34.5,") != std::string::npos);
        assert(second.find(",12000,2,,,,") != std::string::npos);
        assert(first.substr(first.size() - 2) == ",1");
        assert(!saveTelemetryCsv(result, (directory / "missing/file.csv").string()));
        assert(!saveTelemetryCsv(result, directory.string())); // Rename failure removes its temp file.
        assert(!std::filesystem::exists(directory.string() + ".tmp"));
        assert(std::filesystem::remove(path) && std::filesystem::remove(directory));
    }
    // A return-only invalid sensor must stop reversing, in either mode.
    for (Mode mode : {Mode::LATERAL, Mode::ANGULAR}) {
        PhysicsDrive d; d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c; c.mode = mode;
        unsigned returnReads = 0;
        c.independentDisplacement = [&]() -> double {
            if (d.manual) { ++returnReads; return std::numeric_limits<double>::quiet_NaN(); }
            return d.position;
        };
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::SENSOR_FAULT && returnReads == 1);
        assert(!r.trials.back().returnedToStart && !d.active && !d.swapped);
    }
    // Validate the return's first reading before any reverse command is issued.
    for (Mode mode : {Mode::LATERAL, Mode::ANGULAR}) {
        PhysicsDrive d; d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c; c.mode = mode;
        c.independentDisplacement = [&] { return d.position + (d.swapped ? 0 : 3); };
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::MEASUREMENT_DISAGREEMENT && !d.manual);
        assert(!r.trials.back().returnedToStart && !d.active && !d.swapped);
    }
    {
        FrozenReturnDrive d; d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c;
        unsigned returnReads = 0;
        c.independentDisplacement = [&] {
            if (d.manual) ++returnReads;
            return d.position;
        };
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::MEASUREMENT_DISAGREEMENT && returnReads > 0);
        assert(std::abs(d.position) < c.maxTravelInches && d.position > 0);
        assert(!r.trials.back().returnedToStart && !d.active && !d.swapped);
    }
    // Matching signed measurements must work throughout return and after 0/360 wrap.
    for (Mode mode : {Mode::LATERAL, Mode::ANGULAR}) {
        WrappedPhysicsDrive d; d.position = mode == Mode::ANGULAR ? 350 : -10;
        const double origin = d.position;
        d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c; c.mode = mode;
        unsigned returnReads = 0;
        c.independentDisplacement = [&] {
            if (d.manual) ++returnReads;
            return d.position - origin;
        };
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::COMPLETE && returnReads > 0);
        assert(!d.active && !d.swapped);
    }
    {
        AngularReturnJumpDrive d; d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c; c.mode = Mode::ANGULAR;
        c.maxSensorSpeed = 400; // Raising angular speed must not relax the x/y limit.
        auto r = SelfTuningAuton(d, c).run();
        assert(d.jumped && r.status == AutonTuneStatus::SENSOR_FAULT);
        assert(!d.active && !d.swapped && !r.trials.back().returnedToStart);
    }
    {
        InitialReturnJumpDrive d; d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c; c.mode = Mode::ANGULAR;
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::SENSOR_FAULT && d.returnPoses == 2);
        assert(!d.manual && !d.active && !d.swapped);
    }
    // Only effective targets constrain travel and angular range.
    for (Mode mode : {Mode::LATERAL, Mode::ANGULAR}) {
        for (double unused : {24.0, 500.0, std::numeric_limits<double>::quiet_NaN()}) {
            PhysicsDrive d; d.friction = 0.2; d.noise = 0;
            AutonTuneConfig c; c.mode = mode; c.testDistances = {6};
            c.maxTravelInches = 12; c.distanceInches = unused;
            assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::COMPLETE);
        }
    }
    for (bool list : {false, true}) {
        for (double target : {0.0, -1.0, 0.5, 12.0, std::numeric_limits<double>::quiet_NaN()}) {
            PhysicsDrive d; AutonTuneConfig c; c.maxTravelInches = 12;
            if (list) c.testDistances = {6, target};
            else c.distanceInches = target;
            assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::INVALID_CONFIG && d.moves == 0);
        }
    }
    {
        PhysicsDrive d; AutonTuneConfig c; c.mode = Mode::ANGULAR;
        c.testDistances = {181}; c.maxTravelInches = 200;
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::INVALID_CONFIG && d.moves == 0);
    }
    // Finite doubles outside the target PID's float range never reach the drive.
    for (double AutonTuneConfig::* member : {&AutonTuneConfig::maxKp, &AutonTuneConfig::maxKi,
            &AutonTuneConfig::maxKd, &AutonTuneConfig::integralWindupRange, &AutonTuneConfig::kiStep}) {
        PhysicsDrive d; AutonTuneConfig c; c.*member = 1e100;
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::INVALID_CONFIG && d.moves == 0);
    }
    for (Gains candidate : {Gains{1e100, 0, 0}, Gains{1, 1e100, 0}, Gains{1, 0, 1e100}}) {
        PhysicsDrive d; AutonTuneConfig c;
        c.maxKp = c.maxKi = c.maxKd = std::numeric_limits<float>::max();
        c.adjustmentPolicy = [candidate](const Gains&, double, double, unsigned) { return candidate; };
        const auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::GAIN_LIMIT && d.moves == 1);
        assert(!d.active && !d.swapped);
    }
    // Actual guard implementation, including original copy and idempotent restore.
    for (bool throws : {false, true}) {
        ConstPid pid(9);
        unsigned stops = 0;
        try {
            IdlePidGuard<ConstPid> guard(pid, [&] { ++stops; });
            guard.install(ConstPid(2)); assert(pid.p == 2);
            if (throws) throw std::runtime_error("observe");
            guard.restore(); guard.restore(); assert(pid.p == 9);
        } catch (const std::runtime_error&) {}
        assert(pid.p == 9 && stops == 2);
    }
    for (int fault : {1, 2, 3}) {
        PhysicsDrive drive; drive.throwAt = fault;
        try { SelfTuningAuton(drive).run(); assert(false); }
        catch (const std::runtime_error&) {}
        assert(!drive.active && !drive.swapped && drive.gains.kP == 10);
    }
    {
        PhysicsDrive d; d.ready = false;
        assert(SelfTuningAuton(d).run().status == AutonTuneStatus::NOT_CALIBRATED && d.moves == 0);
        d.ready = true; d.voltage = 6500;
        assert(SelfTuningAuton(d).run().status == AutonTuneStatus::LOW_BATTERY && d.moves == 0);
        d.voltage = 12000;
        AutonTuneConfig c; c.requireConfirmation = [] { return false; };
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::ABORTED && d.moves == 0);
        c.requireConfirmation = {}; c.independentDisplacement = [] { return 99.; };
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::MEASUREMENT_DISAGREEMENT);
        assert(!d.active && !d.swapped);
    }
    {
        PhysicsDrive d; d.sagAt = 100;
        auto r = SelfTuningAuton(d).run();
        assert(r.status == AutonTuneStatus::LOW_BATTERY);
        assert(r.trials.size() == 1 && r.trials[0].batteryMinimumMillivolts == 6500);
        assert(!d.active && !d.swapped);
    }
    {
        PhysicsDrive d; d.friction = 126;
        AutonTuneConfig c; c.maxTrials = 4;
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::TRIAL_LIMIT && !d.active && !d.swapped);
    }
    std::mt19937 random(3004);
    for (unsigned seed = 0; seed < 12; ++seed) {
        PhysicsDrive d;
        d.response = std::uniform_real_distribution<double>(0.04, 0.12)(random);
        d.friction = std::uniform_real_distribution<double>(0.1, 0.8)(random);
        AutonTuneConfig c; c.maxTrials = 80; c.testDistances = {6, 24, 72}; c.maxTravelInches = 84; c.returnTimeoutMs = 15000;
        auto r = SelfTuningAuton(d, c).run();
        if (r.status != AutonTuneStatus::COMPLETE)
            std::cerr << "physics seed " << seed << " status " << static_cast<int>(r.status) << '\n';
        if (r.status != AutonTuneStatus::COMPLETE) for (const auto& t : r.trials)
            std::cerr << t.target << " P=" << t.gains.kP << " D=" << t.gains.kD << " error=" << t.finalErrorInches << " over=" << t.overshootInches << " settled=" << t.settled << '\n';
        assert(r.status == AutonTuneStatus::COMPLETE);
        assert(r.trials.size() >= 6 && !d.active && !d.swapped);
        for (unsigned i = 0; i < 6; ++i) {
            const auto& t = r.trials[r.trials.size() - 6 + i];
            assert(t.settled && t.returnedToStart && t.target == c.testDistances[i % 3]);
            assert(t.batteryMinimumMillivolts == 12000);
        }
    }
    {
        PhysicsDrive d; d.noise = 0;
        AutonTuneConfig c; c.mode = Mode::ANGULAR; c.distanceInches = 90;
        c.testDistances = {90, 180}; c.maxTravelInches = 210; c.maxSensorSpeed = 400;
        c.trialTimeoutMs = 15000; c.returnTimeoutMs = 20000; c.maxTrials = 60;
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::COMPLETE && r.mode == Mode::ANGULAR);
    }
    {
        PhysicsDrive d;
        AutonTuneConfig c; c.maxTrials = 2;
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::TRIAL_LIMIT && r.bestTrial);
        assert(r.gains.kP == r.trials[*r.bestTrial].gains.kP);
        assert(!saveAcceptedGains(r, "/tmp/should-not-write-guantune.json"));
        assert(controllerSettingsSnippet(r).empty());
        r.status = AutonTuneStatus::COMPLETE;
        r.gains.kI = 0.01;
        r.integralWindupRange = 7.5;
        const auto directory = std::filesystem::temp_directory_path() /
            ("guantune-quality-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        assert(std::filesystem::create_directory(directory));
        const std::string path = (directory / "result.json").string();
        assert(saveAcceptedGains(r, path));
        std::ifstream saved(path); std::string text; std::getline(saved, text);
        assert(text.find("\"kP\":") != std::string::npos);
        assert(text.find("\"schema\":2") != std::string::npos);
        assert(text.find("\"windupRange\":7.5") != std::string::npos);
        assert(controllerSettingsSnippet(r).find("tunedSettings.windupRange = 7.5;") != std::string::npos);
        assert(std::remove(path.c_str()) == 0);
        assert(std::filesystem::remove(directory));
        assert(!saveAcceptedGains(r, "/missing/guantune/result.json"));
        r.integralWindupRange = 1e100;
        assert(!accepted(r) && controllerSettingsSnippet(r).empty());
        r.integralWindupRange = 3;
        r.gains.kP = 1e100;
        assert(!accepted(r) && controllerSettingsSnippet(r).empty());
    }
    {
        PhysicsDrive d; d.friction = 0.2; d.noise = 0;
        AutonTuneConfig c; c.integralWindupRange = 7.5;
        auto r = SelfTuningAuton(d, c).run();
        assert(r.status == AutonTuneStatus::COMPLETE && r.integralWindupRange == 7.5);
    }
    {
        PhysicsDrive d; AutonTuneConfig c;
        c.adjustmentPolicy = [](const Gains&, double, double, unsigned) { return Gains{999, 0, 0}; };
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::GAIN_LIMIT);
    }
    std::cout << "Physics, safety, restoration and persistence tests passed\n";
}
