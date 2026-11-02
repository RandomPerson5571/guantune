# LemLib chassis wrapper

`SelfTuningAuton` accepts a `lemlib::Chassis&` and owns a concrete
`LemLibTuningDrive` wrapper referring to that chassis. The caller keeps the
chassis alive and grants exclusive drivetrain access until `run()` returns.
There is no generic hardware interface, alternative drivetrain implementation,
relay tuner, or asynchronous background runner.

## Responsibilities

- **LemLib** owns odometry, motion controllers, PID updates, motion scheduling,
  and the underlying drivetrain. Guantune never resets odometry or writes motor
  outputs directly.
- **The chassis wrapper** calls `getPose()`, `moveToPoint()`, `turnToHeading()`,
  `tank()`, `isInMotion()`, `cancelAllMotions()` and `waitUntilDone()`. PROS provides
  timing, competition state and optional battery/motor telemetry. It temporarily
  installs a candidate LemLib PID only while motion is idle.
- **The session runner** selects targets, sequences trials/returns, handles
  cancellation, counts confirmations and reports accepted or diagnostic gains.
- **Position analysis** projects successive `getPose()` snapshots onto the fixed
  starting heading, unwraps angular changes and checks settling/travel/sensor
  limits. These are measurements of LemLib odometry, not a replacement estimator.
- **Gain search** adjusts P/D/I between trials. It never generates live motor
  commands. LemLib computes every forward trial's PID output.
- **Results/telemetry** returns an owning `AutonTuneResult` object with gains,
  status, trials and optional samples. Motor telemetry is read at a reduced
  cadence. The caller chooses the display or transport; Guantune has no file,
  console, JSON, CSV or SD-card output code.

## Files

All headers live in `include/autotune/`:

- `api.hpp`: supported entry points.
- `self_tuning_auton.hpp`: the chassis-based session runner and abort control.
- `lemlib_tuning_drive.hpp`: concrete chassis wrapper, helpers and settings conversion.
- `tuning_config.hpp`, `tuning_result.hpp`, `types.hpp`: configuration and results.
- `position.hpp`: a snapshot of a LemLib position reading.
- `telemetry.hpp`: optional in-memory recording values.
- `angle.hpp`, `idle_pid_guard.hpp`, `pid_validation.hpp`: shared utilities.
- `detail/`: internal declarations for gain search, position analysis, telemetry,
  configuration, cleanup and constants. These ship with the
  template but are not supported extension points.

`src/autotune/` contains only `.cpp` implementations, grouped into `core/`,
`motion/`, `adapters/` and `telemetry/`. The PROS build recursively
compiles these folders. Template packaging includes nested headers.

## Session sequence

1. Acquire the run guard; reject invalid configuration, disabled/active chassis,
   incomplete calibration and nonfinite position before movement.
2. Ask for optional clearance confirmation and save the initial `getPose()`.
3. Install candidate gains into the selected idle LemLib PID.
4. Request a LemLib move or turn and wait for evidence it started. Measure
   movement relative to this trial's initial reading for startup detection.
5. Continue sampling `getPose()` through movement and post-stop coast. Measure
   error from the fixed session origin, including any residual return offset.
6. Restore the original PID and return with a fixed low-power chassis tank
   command. Stop early using measured speed, observe coast and allow bounded
   creep within the same deadline. Never reset pose to make a return pass.
7. Confirm the candidate across targets, or choose another candidate if trials
   remain. Only accepted confirmations plus a successful return yield `COMPLETE`.
8. Return results with the original PID restored and owned chassis commands stopped.

One position tracker spans all phases, retaining heading changes at boundaries.
Angular targets sent to LemLib are normalized to [0, 360); angular measurements
remain unwrapped. Return acceptance also checks heading against the fixed origin.

Aborts are latched and consumed only by the owning session. An idle abort cancels
the next run. Concurrent calls on one tuner return `ALREADY_RUNNING`; separate
objects still require the caller to coordinate chassis ownership. Poll watchdogs
bound stalled-clock loops. Forced task deletion and power loss cannot run cleanup.

## Migration

Construct `SelfTuningAuton(chassis, config, readinessCallback)` directly.
The former `TuningDrive` constructor, fake-drive interface, relay proposals and
legacy asynchronous APIs have been removed. `tuneLateral`, `tuneAngular`,
`AutonTuner`, returned result objects and settings conversion remain available.
`distanceInches`/`testDistances` are lateral-only;
`angleDegrees`/`testAngles` are angular-only. Other legacy tolerance/travel/speed
field names still use mode-dependent units, as described in `tuning.md`.

## Verification boundary

Host tests exercise pure policy, configuration, analysis of position readings,
telemetry recording. They do not emulate a LemLib chassis.
The PROS library/template and consumer checks compile/link the actual motion
wrapper against upstream dependencies. They do not exercise scheduling,
cancellation, traction or settling at runtime. Those need physical sessions;
see `hardware-validation.md`. Historical generic-drive audits describe an older
architecture and are not current wrapper coverage.
