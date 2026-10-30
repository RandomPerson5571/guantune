# Reading Guantune

Start with `src/autotune/self_tuning_auton.cpp`. Its `run()` method coordinates
one session; the other files implement the individual responsibilities below.
The supported runner is `SelfTuningAuton` (also named `AutonTuner`). The older
relay interfaces remain unfinished proposals, clearly marked in their headers.

## Public interfaces

- `tuning_drive.hpp` defines the hardware boundary and pose coordinates.
  Tests implement `TuningDrive`; the real adapter implements the same operations.
- `tuning_config.hpp` contains target, tolerance, timing, search and safety
  settings, plus the optional callbacks. Configuration field order is retained
  for existing C++20 designated initializers.
- `tuning_result.hpp` contains session status and forward trial measurements.
  Only `COMPLETE` means all required confirmations and the final return passed.
- `self_tuning_auton.hpp` includes those three headers and declares the runner.
  Existing includes and public session entry points continue to work.
- `lemlib_tuning_drive.hpp` declares the hardware adapter and convenience helpers.
  The small, pure `toLemLib()` conversion remains inline for native header tests.
- `telemetry.hpp` defines recording values without hardware dependencies.
- `output.hpp` declares acceptance checks, JSON persistence, settings snippets,
  and CSV export. Serialization and filesystem implementation stays in sources.

## Implementation responsibilities

- `self_tuning_auton.cpp`: session orchestration, trial budget, target cycling,
  confirmations and best-candidate fallback. It does not calculate sensor deltas
  or decide which gain to increase.
- `tuning_config.cpp`: validation before motion. Target lists, deadlines, trial
  budgets, gain representability and output bounds have named validation steps.
- `motion_tracking.cpp`: projections, heading unwrapping, elapsed-time deltas,
  speed calculations and movement/sensor limits. One `MotionTracker` spans the
  whole session, including the boundaries between forward movement and return.
- `session_control.cpp`: cancellation, readiness, bounded idle waiting and clock
  watchdogs. Latched aborts are consumed only by the owning session.
- `battery_monitor.cpp`: strict resting checks and time-filtered low readings
  under load. It has no motor control responsibilities.
- `tuning_motion.cpp`: one forward trial with coast observation.
- `tuning_return.cpp`: predictive return stopping, coast observation and bounded
  creep retries. Motion methods do not select gains or write files.
- `telemetry.cpp`: sampling cadence, memory bounds and every-fifth-record motor
  I/O. Measurement tracking is separate and works with recording disabled.
- `gain_search.cpp`: the deterministic P/D/I adjustment schedule and optional
  policy. Named adjustments replace signed integer action codes. Search history
  is local to a session, so reusing a tuner starts with fresh search state.
- `lemlib_tuning_drive.cpp`: the actual PROS/LemLib calls, motion ownership and
  temporary installation/restoration of PID objects.
- `output.cpp` and `telemetry_output.cpp`: JSON/settings formatting and CSV rows.
  `file_output_internal.hpp` shares temporary-file/rename handling.
- `session_internal.hpp`: private types shared by these implementations. It is
  a source implementation detail, not another public API.

## Session sequence

1. Refuse a concurrent run or invalid configuration. Check the idle drive,
   calibration and finite pose before movement.
2. Run the optional once-per-session confirmation callback. The supplied
   opcontrol example does not require a button press.
3. Save the origin and start with P=1.1, I=0, D=1.1.
4. Select a target without changing the stored configuration. Install candidate
   gains, request movement and wait for evidence that this trial started.
5. Measure error, overshoot, target crossings and settling. Stop movement and
   observe coast before deciding whether the measurements passed.
6. Restore the original PID and return to the fixed origin with low power.
7. Confirm the same gains on the next target, or reset confirmations and select
   a new candidate. Never select an unused candidate after the final trial.
8. Return accepted gains on success, or diagnostic measurements and a status.
   Every unsuccessful result uses the best safely returned trial as a diagnostic
   recommendation, or starting gains when none completed.

## Rules worth preserving

**The origin does not move.** Lateral position is projected onto the initial
heading. Returns use the same coordinate system. A valid return can stop within
tolerance rather than at exactly zero, so startup detection measures movement
since the current trial began, not displacement from the session origin.

**Angular displacement does not wrap.** Compass targets sent to LemLib are
normalized to [0, 360), while the motion tracker accumulates signed heading
changes. It retains the last pose and time between phases so a heading change
during cleanup is still checked. Return acceptance also checks heading error
against the saved origin.

**Units follow the mode.** Legacy fields named `distanceInches`, `range`,
`returnToleranceInches`, and similar values describe degrees in angular mode.
`maxCrossTrackInches` and `maxTranslationalSensorSpeed` always describe translation
in inches and inches/second. Motor output and return power use -127 to 127 units.

**Cleanup owns the ordering.** `DriveCleanup` stops owned motion before restoring
PID state on ordinary return, abort, faults and callback exceptions. The adapter
also cleans up on destruction. Cleanup operations must not throw. The cleanup guard attempts both operations
without letting exceptions escape its destructor; this cannot guarantee a stop
if a hardware operation itself fails. Forced task
deletion and power loss do not run C++ destructors.

**No other task writes the drivetrain during a run.** The runner is blocking,
but a second task can call `abort()`. The atomic run guard protects one tuner
instance; it does not coordinate separate tuners or unrelated chassis owners.
Concurrent runs return `ALREADY_RUNNING`. An idle abort cancels the next run;
it is not discarded at startup. Callbacks must be prompt and must not start
motions or reposition the robot.

**Limits are real exits.** The trial budget includes confirmations and every
target. Sensor/travel faults stop without automatically reversing. Failed returns
stop the session. The tuner does not silently retry failed sessions or increase
the configured budget. Same-gain unsettled retries are bounded and each consumes
a trial; predictive creep retries share the existing return deadline.

## Making and validating changes

Keep hardware calls in the adapter. Put new search behavior in `gain_search.cpp`
and add tests through `TuningDrive`; do not mock upstream PROS/LemLib namespaces.
Motion regressions should check final status, whether movement actually started,
final return position, stopped output and original PID restoration.

Run `bash scripts/test_host.sh` with C++17/C++20 host compilers. The script links
all core sources, with ASan/UBSan and warnings as errors. The recorded audit's
four defects now have regression cases in the scripted/physics suites.

Copy or build the entire source directory, not only `self_tuning_auton.cpp`.
`scripts/prepare_build.py` already copies the complete tree. After adapter or
public-header changes, build/install the real PROS template and link a consumer;
the host suite alone cannot verify the hardware interface. See `compatibility.md`
and `test/README.md`. Robot repeatability still needs physical validation.

## Refactor verification — 2026-10-29

Clang C++17 and C++20 host suites passed with ASan/UBSan, including all four
audit regressions and the ARM template-optimization test. A clean PROS 4.2.2 /
LemLib 0.5.6 library/template build passed. Installing that template into a fresh
consumer compiled the automatic opcontrol example and linked both monolithic
and hot/cold binaries. The native settings-conversion test also passed against
real headers with the documented Apple Clang `FMT_CONSTEVAL` workaround.

The published base commit's latest-stable CI job was
[successful](https://github.com/RandomPerson5571/guantune/actions/runs/40933146371/job/123656590220).
That run covers the published base, not this uncommitted refactor. No upload or
physical robot validation was performed.
