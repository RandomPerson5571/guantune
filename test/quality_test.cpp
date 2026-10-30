#include "support/physics_drive.hpp"

void testAngularPhaseBoundary() {
    AngularBoundaryJumpDrive drive;
    drive.friction = 0.2;
    drive.noise = 0;
    AutonTuneConfig config;
    config.mode = Mode::ANGULAR;
    const auto result = SelfTuningAuton(drive, config).run();
    assert(result.status == AutonTuneStatus::SENSOR_FAULT);
    assert(result.trials.size() == 1 && !result.trials.front().returnedToStart);
    assert(!drive.manual && !drive.active && !drive.swapped);
}

int main() {
    testAngularPhaseBoundary();
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
        assert(r.status == AutonTuneStatus::SENSOR_FAULT && d.returnPoses == 1);
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
        d.ready = true;
        AutonTuneConfig c; c.requireConfirmation = [] { return false; };
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::ABORTED && d.moves == 0);
        c.requireConfirmation = {}; c.independentDisplacement = [] { return 99.; };
        assert(SelfTuningAuton(d, c).run().status == AutonTuneStatus::MEASUREMENT_DISAGREEMENT);
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
        }
    }
    {
        PhysicsDrive d; d.noise = 0;
        AutonTuneConfig c; c.mode = Mode::ANGULAR; c.distanceInches = 90;
        c.testDistances = {90, 170}; c.maxTravelInches = 210; c.maxSensorSpeed = 400;
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
