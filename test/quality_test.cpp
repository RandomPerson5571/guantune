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
int main() {
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
        const auto directory = std::filesystem::temp_directory_path() /
            ("guantune-quality-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        assert(std::filesystem::create_directory(directory));
        const std::string path = (directory / "result.json").string();
        assert(saveAcceptedGains(r, path));
        std::ifstream saved(path); std::string text; std::getline(saved, text);
        assert(text.find("\"kP\":") != std::string::npos);
        assert(!controllerSettingsSnippet(r).empty());
        assert(std::remove(path.c_str()) == 0);
        assert(std::filesystem::remove(directory));
        assert(!saveAcceptedGains(r, "/missing/guantune/result.json"));
    }
    {
        PhysicsDrive d; AutonTuneConfig c;
        c.adjustmentPolicy = [](const Gains&, double, double, unsigned) { return Gains{999, 0, 0}; };
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::GAIN_LIMIT);
    }
    std::cout << "Physics, safety, restoration and persistence tests passed\n";
}
