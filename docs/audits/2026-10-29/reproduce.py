"""Reproduce the 2026-10-29 Guantune audit observations against the current core.

Run from any directory: python3 docs/audits/2026-10-29/reproduce.py
CXX (default clang++) and CXXSTD (default c++17) select the host compiler.
Fixtures are extracted from existing tests; no PROS/LemLib stubs are introduced.
The printed observations describe the current behavior, not desired assertions.
"""
import os
from pathlib import Path
import subprocess
import tempfile

LIBRARY = Path(__file__).resolve().parents[3]

DECISION_PROBES = r'''
#include "fake_drive.hpp"

struct AuditCountingDelayedDrive : DelayedStartDrive {
    unsigned starts = 0, cancelledWhilePending = 0;
    void delayMs(std::uint32_t ms) override {
        const bool wasPending = pending;
        DelayedStartDrive::delayMs(ms);
        if (wasPending && !pending) ++starts;
    }
    void stop() override {
        if (pending) ++cancelledWhilePending;
        DelayedStartDrive::stop();
    }
};

int main() {
    {
        AuditCountingDelayedDrive d;
        AutonTuneConfig c;
        c.range = 0.1;
        c.crossingDeadband = 0.1;
        c.maxTrials = 4;
        auto r = SelfTuningAuton(d, c).run();
        std::cout << "residual-start: status=" << statusName(r.status)
                  << " trials=" << r.trials.size() << " actual-starts=" << d.starts
                  << " pending-cancellations=" << d.cancelledWhilePending << '\n';
        for (const auto& t:r.trials)
            std::cout << "  measured=" << t.distanceInches << " settled=" << t.settled << '\n';
        d.stopped();
    }
    {
        FakeDrive d;
        AutonTuneConfig c;
        c.postStopMs = 6000; // Accepted, but a return cannot settle before 5000ms timeout.
        auto r = SelfTuningAuton(d,c).run();
        std::cout << "impossible-return: status=" << statusName(r.status)
                  << " trials=" << r.trials.size() << " writes=" << d.writes << '\n';
        d.stopped();
    }
    {
        FakeDrive d;
        AutonTuneConfig c; c.maxTrials = 3;
        bool requestedUntestableCandidate = false;
        c.adjustmentPolicy = [&](const Gains&, double, double, unsigned i) {
            requestedUntestableCandidate = i == 2;
            return i == 1 ? Gains{4.1,0,1.1} : Gains{999,0,0};
        };
        auto r = SelfTuningAuton(d,c).run([&](unsigned i,const Gains&) {
            d.profile = i == 0 ? FakeDrive::GOOD : FakeDrive::SHORT;
            return true;
        });
        std::cout << "budget-exhaustion: status=" << statusName(r.status)
                  << " trials=" << r.trials.size() << " policy-called-after-last="
                  << requestedUntestableCandidate << " best-P=" << r.trials[*r.bestTrial].gains.kP
                  << " returned-P=" << r.gains.kP << '\n';
        d.stopped();
    }
}
'''

ANGULAR_PROBE = r'''
#include "physics_drive.hpp"
struct BoundaryJumpDrive : PhysicsDrive {
    unsigned jumps = 0;
    void restoreGains() override {
        if (swapped) { position += 5; ++jumps; }
        PhysicsDrive::restoreGains();
    }
};
int main() {
    BoundaryJumpDrive d; d.friction = 0.2; d.noise = 0;
    AutonTuneConfig c; c.mode = Mode::ANGULAR;
    auto r = SelfTuningAuton(d,c).run();
    std::cout << "angular-boundary: status=" << statusName(r.status)
              << " trials=" << r.trials.size() << " final-heading=" << d.position
              << " allowed-return-error=" << c.returnToleranceInches << " jumps=" << d.jumps << '\n';
    for (const auto& t:r.trials)
        std::cout << "  measured=" << t.distanceInches << " returned=" << t.returnedToStart << '\n';
}
'''

def main():
    with tempfile.TemporaryDirectory(prefix="guantune-audit-") as directory:
        work = Path(directory)
        for fixture, test in (("fake_drive.hpp", "self_tuning_auton_test.cpp"),
                              ("physics_drive.hpp", "quality_test.cpp")):
            helper = "scripted_drive.hpp" if test == "self_tuning_auton_test.cpp" else "physics_drive.hpp"
            text = (LIBRARY / "test/support" / helper).read_text()
            (work / fixture).write_text(text)
        for name, source in (("decisions", DECISION_PROBES), ("angular", ANGULAR_PROBE)):
            path = work / (name + ".cpp")
            executable = work / name
            path.write_text(source)
            subprocess.run([
                os.environ.get("CXX", "clang++"),
                "-std=" + os.environ.get("CXXSTD", "c++17"),
                "-Wall", "-Wextra", "-Werror", "-pedantic",
                "-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-g",
                "-I", str(LIBRARY / "include"),
                *[str(LIBRARY / "src/autotune" / name) for name in (
                    "self_tuning_auton.cpp", "session_control.cpp", "battery_monitor.cpp",
                    "telemetry.cpp", "telemetry_output.cpp", "tuning_return.cpp", "tuning_config.cpp", "gain_search.cpp",
                    "motion_tracking.cpp", "tuning_motion.cpp", "output.cpp")],
                str(path), "-o", str(executable)
            ], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
