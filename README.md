# autotune — PROS / LemLib autonomous PID tuning

`SelfTuningAuton` performs repeated **24-inch forward trials**, starting with
**kP = 1.1, kI = 0, kD = 1.1**. It measures the complete motion and adjusts the
next trial's gains using the supplied tuning diagrams as guidance.

The self-tuning autonomous runner is implemented. The older `RelayTuner`,
`Tuner`, `Result::print`, and relay-result settings conversion remain declarations
only. Use the new API below; those older interfaces are not callable yet.

## Quick start

Install PROS and LemLib separately, calibrate your existing chassis and odometry,
and add the autotune headers and sources to your project. No wiring is assumed
or changed. See `examples/self_tuning_autonomous.cpp` for a PROS function that
prints each trial and waits for a fresh controller A press before each run.

```cpp
#include "autotune/lemlib_tuning_drive.hpp"

autotune::LemLibTuningDrive drive(chassis);
autotune::AutonTuneConfig config;
config.range = 0.1; // Acceptable distance error: +/-0.1 inch.
config.maxTrials = 15;
config.allowIntegral = false; // Default: keep kI at zero.
autotune::AutonTuner tuner(drive, config);

auto result = tuner.run([&](unsigned trial, const autotune::Gains& gains) {
    // REQUIRED: reposition or confirm clear room for another 24-inch run.
    // Finish any repositioning motion before returning true.
    // Return false to abort. See the complete example for a controller gate.
    return readyForNextTrial(trial, gains); // Supply your own callback.
});

if (result.status == autotune::AutonTuneStatus::COMPLETE) {
    auto settings = autotune::toLemLib(result, existingLateralSettings);
    // Save/print settings->kP, kI, kD for your chassis configuration.
}
```

`AutonTuner` is an alias for `SelfTuningAuton`. With C++20, construction can also
use designated parameters:

```cpp
autotune::AutonTuner tuner(drive, {.range = 0.1, .maxTrials = 15});
```

`range` is the acceptable distance error in inches, not a PID gain increment.
For the 24-inch target, `range = 0.1` accepts 23.9 to 24.1 inches, subject to
the speed, overshoot, and confirmation checks. It overrides the existing
`toleranceInches` parameter; omitting it preserves the default 0.5-inch tolerance.
`maxTrials` caps all trials, including confirmation runs. It must be at least
`confirmationTrials` (two by default). Invalid ranges (zero, negative, nonfinite,
or at least the target distance) and invalid trial limits return `INVALID_CONFIG`
before motor output. The drive argument supplies the robot hardware connection.

`run()` blocks and does not launch tasks. Only one run per tuner is permitted.
Another task can call `tuner.abort()` during motion; motion checks it each loop.
The caller must keep both objects alive and grant exclusive drivetrain access:
no driver control, LemLib motions, or other tuners may command it concurrently.
The adapter rejects active LemLib motions and a disabled competition state.
A callback that waits or repositions must handle its own cancellation; `abort()`
cannot interrupt arbitrary callback code. The example supports B while waiting;
disable the robot to stop during a trial.

Every trial starts at the current pose and goes forward. The library neither
resets odometry nor drives backward automatically. Blindly returning true may
produce up to 20 consecutive 24-inch runs; use the callback deliberately.

## Adjustment rules

- A stationary result within the configured range of target, with no excessive overshoot or
  oscillation, records the gains. Two consecutive successful trials are required.
- Overshoot greater than the configured range, or at least three meaningful crossings of the
  target, increases kD. Crossings use a +/-0.5-inch deadband to reject noise.
- An unsuccessful run without excessive overshoot or oscillation increases kP.
  This includes undershoot after the five-second trial timeout.
- P/D increments are +3 below a gain of 6, +1 from 6 to below 12, and +0.1 from
  12 onward. Switching between increasing P and increasing D also permanently
  selects +0.1 refinement for the rest of that run.
- kI is zero by default. With `allowIntegral = true`, it is eligible only after
  eight P/D adjustments, three consecutive undershoot trials, kP >= 12, no
  meaningful crossings, a stationary final reading, and a residual error within
  the 3-inch integral range.
  It increases by 0.001, capped at 0.05. Overshoot/oscillation first decreases
  any existing kI by 0.001 before increasing D. It is never enabled merely
  because a zero-I trial did not overshoot.

These are bounded heuristic rules, not relay feedback or a mathematical optimum.
A large kP, friction, bad odometry, or insufficient timeout can prevent convergence.
A failed run does not yield approved gains: `result.gains` always contains the
last **tested** gains, and only `COMPLETE` indicates confirmation passed. Inspect
`result.trials` for each tested gain set, final distance/error, peak overshoot,
crossings, settlement, timeout, and elapsed time.

## Measurements and safeguards

Lateral distance is the displacement from each starting pose projected onto its
heading, in inches. Heading follows LemLib's clockwise degrees with zero along
+y. This is odometry distance, not an independent ground-truth measurement;
wheel slip and odometry calibration directly affect tuning accuracy.

The runner samples every 10 ms, holds the initial heading using a separate
proportional correction, and limits both motor commands to 60 by default
(configurable up to 127). The lateral PID uses discrete error differences and
integral sums, as in [LemLib 0.5.6 PID](https://github.com/LemLib/LemLib/blob/v0.5.6/src/lemlib/pid.cpp),
with integral reset on sign changes and outside the windup range. The first
sample suppresses derivative kick. Gains should be used with the same 10 ms
interval and integral policy; they are a starting point for LemLib motion tuning,
whose heading, slew, and exit behavior differ from this isolated straight test.

Settlement requires 500 ms within tolerance and below 1 inch/second. After
settling or timing out, motors stop and measurement continues for 300 ms to
include coasting in the final distance and overshoot. Timeout is per trial;
post-stop observation adds up to 300 ms. Callback time is outside that timeout.

A trial stops for cancellation, disable, unavailable drive, nonfinite pose,
sensor jumps above 120 inches/second, displacement beyond 36 inches, lateral
wander beyond 4 inches, or heading deviation beyond 20 degrees. Gain limits and
20 trials bound the search. Stops command zero; braking behavior depends on your
robot's configured brake mode. Calibrate odometry and use a clear field with an
immediate stop mechanism.

`toLemLib(AutonTuneResult, baseSettings)` returns no settings for unsuccessful
runs and replaces only the gains on success. Use the returned settings when
constructing a chassis. Keep integral windup/sign-reset behavior consistent if
using nonzero kI. Other configuration is preserved. No existing chassis or
LemLib dependency is modified.

## Layout and tests

- `include/autotune/self_tuning_auton.hpp`: hardware-free drive interface,
  trial reports, configuration, and public class.
- `src/autotune/self_tuning_auton.cpp`: trial controller and gain search.
- `include/autotune/lemlib_tuning_drive.hpp`: thin PROS/LemLib adapter and
  successful-result conversion.
- `examples/self_tuning_autonomous.cpp`: controller-gated robot usage.
- `test/self_tuning_auton_test.cpp`: scripted and simulated host tests.
- `relay_tuner.*`, `lemlib_adapter.*`, and `output.cpp`: original relay TODOs.

Run the host checks from this folder (C++17 or later):

```sh
c++ -std=c++17 -Wall -Wextra -Werror -pedantic -I include \
    src/autotune/self_tuning_auton.cpp test/self_tuning_auton_test.cpp \
    -o /tmp/autotune-self-test
/tmp/autotune-self-test
```

These tests validate decisions and runner safeguards, not real-robot tuning.
The reference dependency versions are PROS kernel 4.2.2 and LemLib 0.5.6.
No ARM build, template installation, or hardware experiment has been validated.
The robot Makefile selects C++20 because the local PROS headers use C++20 APIs.

## Build and distribute as a PROS template

No PROS or LemLib files are bundled. With the PROS CLI and ARM toolchain
installed, create a disposable project outside this folder:

```sh
pros conduct new-project /tmp/autotune-build v5 4.2.2
cd /tmp/autotune-build
pros conduct add-depot LemLib https://raw.githubusercontent.com/LemLib/LemLib/depot/stable.json
pros conduct apply LemLib@0.5.6
```

Remove its generated `src/main.cpp`. Copy this folder's Makefile,
`include/autotune/`, `src/autotune/`, README, LICENSE, and CHANGELOG into that
project, preserving their paths and keeping the kernel-provided `common.mk`.

```sh
pros make library
pros make template
```

The Makefile configures `IS_LIBRARY=1`, `LIBNAME=autotune`, and version 0.1.0,
following the [PROS template workflow](https://pros.cs.purdue.edu/v5/cli/conductor.html#creating-templates).
Inspect the generated zip and `template.pros` before release: distribute autotune
headers, its static archive, and documentation, with no PROS/LemLib dependency
files or application sources. The relay interfaces are still unimplemented and
must be labeled accordingly in any release.

After validation, publish the zip as a GitHub Release asset. Consumers run these
as separate commands from their PROS project, using the actual downloaded path:

```sh
pros conduct fetch /path/to/autotune@0.1.0.zip
pros conduct apply autotune@0.1.0
```

Use semantic versioning and record tested CLI/kernel/LemLib versions and robot
configuration in the changelog. This library uses the MIT license in `LICENSE`.
LemLib is a separate MIT-licensed dependency; PROS retains its upstream license.
