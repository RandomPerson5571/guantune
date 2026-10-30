#include "autotune/angle.hpp"
#include "support/physics_drive.hpp"

// This drive enforces the target-heading contract before simulating a clockwise
// turn. Sensor headings can remain continuous or wrap independently at 0/360.
struct HeadingDrive : PhysicsDrive {
    bool wrapSensorHeading = false;
    std::vector<double> requestedHeadings;

    DrivePose pose() override {
        auto result = PhysicsDrive::pose();
        if (wrapSensorHeading)
            result.headingDegrees = detail::normalizeHeading(result.headingDegrees);
        return result;
    }

    void turnTo(double heading, std::uint32_t, double output) override {
        assert(heading >= 0 && heading < 360);
        requestedHeadings.push_back(heading);
        PhysicsDrive::turnTo(heading, 0, output);
    }
};

int main() {
    for (double start : {720.0, -260.0, 350.0, -720.0, 719.5}) {
        for (bool wrapSensor : {false, true}) {
            HeadingDrive drive;
            drive.position = start;
            drive.wrapSensorHeading = wrapSensor;
            drive.friction = 0.2;
            drive.noise = 0;
            AutonTuneConfig config;
            config.mode = Mode::ANGULAR;
            config.distanceInches = 90;
            config.maxTravelInches = 120;
            config.maxSensorSpeed = 400;
            config.trialTimeoutMs = 15000;
            config.returnTimeoutMs = 20000;
            config.maxTrials = 60;
            const auto result = SelfTuningAuton(drive, config).run();
            if (result.status != AutonTuneStatus::COMPLETE)
                std::cerr << "start=" << start << " wrapped=" << wrapSensor
                          << " status=" << statusName(result.status) << '\n';
            assert(result.status == AutonTuneStatus::COMPLETE);
            assert(!drive.requestedHeadings.empty());
            const double expectedHeading = detail::normalizeHeading(start + 90);
            for (double heading : drive.requestedHeadings)
                assert(std::abs(heading - expectedHeading) < 1e-8);
            // The target heading wraps, but the test distance and final return
            // are still measured relative to the original physical orientation.
            for (const auto& trial : result.trials)
                assert(trial.target == 90 && trial.returnedToStart);
            assert(std::abs(drive.position - start) <= config.returnToleranceInches);
            assert(!drive.active && !drive.swapped);
        }
    }
    std::cout << "Angular headings stay normalized across negative/multiple turns\n";
}
