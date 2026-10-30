# Guantune code audit — 2026-10-29

Four defects were reproduced in the current tuning core: one P1, two P2s,
and one P3. The most serious allows an angular session to report success outside
its configured return tolerance. Existing sanitizer suites pass despite these
cases. No production code was changed during this audit.

## Follow-up during the readability refactor

The four findings below are historical observations against the audited
implementation. The current working tree fixes all four and adds permanent
regression coverage in the scripted and physics suites. Source locations below
refer to the audited layout; see `docs/architecture.md` for the refactored code.
The reproduction script now compiles the split core and prints corrected results.

## Scope and evidence

Reviewed the working tree based on Guantune commit
`911c16ee4d1ed683f9e4f3266220b37d02eaa276`, including the pending automatic
opcontrol example changes. Covered the tuning algorithm, configuration checks,
LemLib adapter, PID restoration, telemetry, persistence, packaging scripts,
CI configuration, examples, and host tests. Also inspected qwerty's caller to
check how the library is currently used.

Reproductions execute the actual `SelfTuningAuton` implementation with fixtures
extracted from the existing tests. They establish software behavior under the
stated inputs; they are not physical robot results.

## Findings

### 1. [P1] Angular return discards heading changes at the trial/return boundary

Location: [self_tuning_auton.cpp:255](/Users/ethanguan/Documents/coding-workspace/VexRobotics/libraries/guantune/src/autotune/self_tuning_auton.cpp:255),
with accumulation at line 273 and acceptance at line 324.

`returnToStart()` receives displacement from the trial's last sample, then takes
a fresh pose as its `previous` baseline. Any rotation or odometry change between
those two samples disappears: only changes after the fresh baseline are added
to the old displacement. The return therefore stops at a biased heading and
can set `returnedToStart=true`. Unlike lateral return, it does not check its
final angular position directly against the saved origin.

Reproduction: extend `PhysicsDrive` so restoring a trial's PID injects a
5-degree heading change between phases. Use angular mode, zero measurement
noise, friction 0.2, and otherwise default configuration. The actual result is:

```text
angular-boundary: status=COMPLETE trials=3 final-heading=5.22879 allowed-return-error=0.5 jumps=3
```

All three trials report a successful return. The session starts at zero degrees,
so its reported success is outside the configured tolerance by more than tenfold.
An independent displacement probe may detect this discrepancy, but that probe
is optional and is absent from the default configuration.

Fix direction: preserve the last pose and timestamp across the phase boundary,
account for the intervening heading delta, and apply sensor checks across that
boundary. Before marking an angular return complete, also verify heading error
against the fixed session origin. Retain unwrapped tracking for travel limits
and 180-degree tests.

### 2. [P2] A return offset can cause queued trials to be cancelled before starting

Location: [self_tuning_auton.cpp:226](/Users/ethanguan/Documents/coding-workspace/VexRobotics/libraries/guantune/src/autotune/self_tuning_auton.cpp:226).

The fallback evidence that a motion started is absolute displacement from the
session origin. A successful return is allowed to leave a residual offset within
`returnToleranceInches`. If that offset exceeds the forward tolerance/deadband,
the next trial immediately sets `motionObserved=true`, even if the asynchronous
command is still queued. The following `!moving && motionObserved` condition
then cancels it as a completed motion. Subsequent gain adjustments consume the
trial budget without testing those gains.

Reproduction: use the existing `DelayedStartDrive` with a 40 ms startup delay,
`range=0.1`, `crossingDeadband=0.1`, `maxTrials=4`, and the default 0.5-inch
return tolerance. The first return leaves a valid 0.48-inch offset:

```text
residual-start: status=TRIAL_LIMIT trials=4 actual-starts=1 pending-cancellations=3
  measured=24 settled=1
  measured=0.48 settled=0
  measured=0.48 settled=0
  measured=0.48 settled=0
```

Fix direction: detect startup using movement since the trial's initial pose,
or an explicit motion lifecycle signal, while continuing to measure target error
from the fixed session origin. Add a repeated delayed-start test with a nonzero
valid return offset; the existing delayed-start test does not cover this case.

### 3. [P2] Configuration validation accepts return timing that cannot succeed

Location: [self_tuning_auton.cpp:54](/Users/ethanguan/Documents/coding-workspace/VexRobotics/libraries/guantune/src/autotune/self_tuning_auton.cpp:54),
with the incompatible requirements at lines 324–325.

Validation requires `returnTimeoutMs > settleMs`, but the return must remain
settled for `max(settleMs, postStopMs)`. It does not compare the timeout with
`postStopMs`. A configuration with `postStopMs=6000` and the default
`returnTimeoutMs=5000` passes validation even though no return can satisfy it.
The robot performs a forward trial and reverse movement before returning
`RETURN_FAILED` for what is actually an invalid configuration.

```text
impossible-return: status=RETURN_FAILED trials=1 writes=3
```

Fix direction: reject return deadlines that cannot accommodate the required
settling period before issuing motor commands. Test both configurations where
`postStopMs` dominates and the timeout/settling equality boundary, taking the
10 ms sampling interval into account.

### 4. [P3] An unused next-candidate adjustment overrides trial-limit reporting

Location: [self_tuning_auton.cpp:425](/Users/ethanguan/Documents/coding-workspace/VexRobotics/libraries/guantune/src/autotune/self_tuning_auton.cpp:425),
before the trial-limit handling at lines 440–442.

After the last allowed failed trial has safely returned, the tuner still calls
`adjustmentPolicy` and validates the next candidate. There is no remaining trial
in which that candidate could be used. An out-of-bounds candidate or an unchanged
candidate at a gain cap causes `GAIN_LIMIT` instead of `TRIAL_LIMIT`; a throwing
policy prevents a result from being returned at all. These exits also bypass
the documented best-trial fallback.

Reproduction: allow three trials; make the first good and the next two short.
The policy selects P=4.1 after the second trial, and P=999 after the third:

```text
budget-exhaustion: status=GAIN_LIMIT trials=3 policy-called-after-last=1 best-P=1.1 returned-P=4.1
```

Fix direction: after evaluating the final trial for completion, exit to the
trial-limit result before requesting or validating another candidate. Preserve
the best measured candidate for diagnostics. Do not increase the trial budget.

## Verification performed

- Clang C++17 and C++20: both complete host suites passed under ASan/UBSan with
  warnings treated as errors.
- The Python preparation tests and real ARM template-optimization regression
  passed in both runs; the ARM regression was enabled by adding the installed
  ARM toolchain to PATH.
- All four findings reproduced with sanitizer-enabled C++17 probes; the angular
  finding also reproduced with C++20.
- No robot session, upload, or new remote CI run was performed. The qwerty PROS
  compile/link passed earlier in this chat after the opcontrol integration
  changes; this audit did not repeat that unchanged build.

## Reproduce the observations

The accompanying [reproduce.py](/Users/ethanguan/Documents/coding-workspace/VexRobotics/libraries/guantune/docs/audits/2026-10-29/reproduce.py)
extracts the existing simulated drive fixtures, compiles the current tuning
core into temporary executables, and prints the observations above:

```sh
cd libraries/guantune
python3 docs/audits/2026-10-29/reproduce.py
CXXSTD=c++20 python3 docs/audits/2026-10-29/reproduce.py
```

The script prints behavior rather than asserting these defects must remain;
rerun it after fixes and compare the outputs with each finding's expected behavior.

## Coverage and validation gaps

- **Integral behavior is not modeled by the physics test.**
  [quality_test.cpp:55](/Users/ethanguan/Documents/coding-workspace/VexRobotics/libraries/guantune/test/quality_test.cpp:55)
  computes P and D only. The scripted suite checks that I is selected, but does
  not demonstrate that integral accumulation, windup cutoff, and reset-on-sign
  change produce acceptable motion or stable convergence.
- **Motion lifecycle behavior needs adapter-level evidence.** Host fixtures
  implement their own `isMoving()` and `stop()`. A compile/link pass cannot
  establish scheduling, cancellation, or task completion on PROS. The adapter
  uses `cancelAllMotions()` and `waitUntilDone()`; the pinned
  [LemLib 0.5.6 source](https://github.com/LemLib/LemLib/blob/v0.5.6/src/lemlib/chassis/chassis.cpp)
  implements these using shared motion flags, delays, and a completion sentinel.
  Exercise delayed startup and cancellation on the real runtime before treating
  that path as hardware-validated. No separate adapter race is counted as a
  confirmed defect in this report.
- **Physical repeatability remains unrecorded.** The
  [hardware log](/Users/ethanguan/Documents/coding-workspace/VexRobotics/libraries/guantune/docs/hardware-validation.md:3)
  contains no sessions. Surface, traction, coasting, battery variation, and
  independently measured final error remain unvalidated.
- GCC was not available on this host; `/usr/bin/g++` is Apple Clang. This audit
  does not claim a local GCC matrix or successful remote CI execution.

## Deliberate limitations, not counted as defects

The relay API and older asynchronous `Tuner` remain deleted scaffolds. The
implemented API is `SelfTuningAuton` with `LemLibTuningDrive`. Accepted gains can
be saved and converted to settings, but there is no built-in saved-gain loader
or automatic permanent installation. The current qwerty opcontrol caller runs
lateral tuning only. These are feature gaps, separate from the reproduced bugs.

Recommended repair order: angular return tracking, repeated-motion startup,
configuration timing validation, then trial-limit result handling.
