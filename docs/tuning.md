# Tuning reference

## Configuration

- `range`: acceptable forward distance error in inches. `0.1` accepts a final
  distance of 23.9–24.1 inches for a 24-inch target, subject to settling,
  overshoot, and confirmation checks. It overrides `toleranceInches`; the
  default tolerance when omitted is 0.5 inch.
- `maxTrials`: maximum forward trials, including confirmation trials; default
  20. It must be at least `confirmationTrials`, which defaults to two. Backward
  returns do not consume additional trials.
- `distanceInches`: forward target distance; default 24.
- `returnPower`: fixed backward command magnitude, 1–40; default 20. This is
  motor power, not a speed in inches per second.
- `returnToleranceInches`: radius around the saved start required for a return;
  default 0.5 inch. This is separate from the forward `range`.
- `returnTimeoutMs`: return deadline including settling; default 5000 ms. It
  must exceed both `settleMs` and `postStopMs`; otherwise the config is rejected
  before movement.
- `trialTimeoutMs`: forward trial deadline before stopping and observing coast;
  default 5000 ms.
- `allowIntegral`: enable last-resort integral tuning; default false.
- `maxOutput`: forward LemLib motion limit, 60–127; default 60. Lower values are
  rejected because [LemLib 0.5.6 moveToPoint](https://github.com/LemLib/LemLib/blob/v0.5.6/src/lemlib/chassis/motions/moveToPoint.cpp)
  can raise its limit to at least 60 near the target.

Additional limits, gain caps, and settling settings are declared in
[`AutonTuneConfig`](../include/autotune/tuning_config.hpp). Invalid configuration
returns `INVALID_CONFIG` before movement. The default travel limit is 36 inches;
each target must leave `max(2 * toleranceInches, 0.1 * target)` of overshoot
headroom below that limit. For example, a 33-inch target is invalid with a
36-inch limit. Angular targets must be strictly less than 180 degrees.

## Trial and return sequence

1. Save the initial `chassis.getPose()` as a fixed origin for the session.
2. Temporarily apply the candidate gains to LemLib's lateral PID and use
   `chassis.moveToPoint()` for the forward target along the starting heading.
3. Sample `chassis.getPose()` every 10 ms. Record projected distance, peak
   overshoot, meaningful target crossings, and settling behavior.
4. Stop the motion and observe coasting. Restore the original lateral PID.
5. Back up slowly with `chassis.tank(-returnPower, -returnPower, true)`, continuing
   to read chassis pose. Stop early by two sample periods of measured travel,
   observe coast, and creep again if it stops short. Accept only after the robot
   is stationary inside the actual return tolerance.
6. Adjust gains or confirm the current gains, then start the next forward trial.

If a trial moves only 20 inches, the robot backs toward its actual saved start;
it does not command a fixed 24-inch reverse move. The return does not use
`moveToPoint()` or a custom PID loop. With a starting pose of (0, 0), it stops
near (0, 0). With another starting pose, it uses that saved coordinate. It never
resets odometry to make a return appear complete.

A return that times out, coasts past the start outside tolerance, or cannot
reach the origin because of sideways error produces
`RETURN_FAILED`. No subsequent trial starts. Returns also follow the final
confirmation or trial-limit run. Aborted, disabled, or unsafe trials stop without
automatically reversing.

## Gain adjustments

- Start each session at P=1.1, I=0, D=1.1.
- Increase D for overshoot beyond the configured forward range, or at least
  three target crossings. Crossings use a default +/-0.5-inch deadband.
- Increase P for stationary undershoot without excessive overshoot or oscillation.
- Coarse P/D steps are 3 below a gain of 6; above that they are 20% of the
  current gain, with a minimum step of 1. After a reversal, fine steps are 2%
  of the current gain, with a minimum step of 0.1.
- If D is capped and overshoot remains, reduce P toward the starting value.
- A trial within tolerance that is not settled retries the same gains up to
  five times. A sixth such failure stops with `TRIAL_LIMIT`. These retries
  consume the ordinary trial budget; the budget is never increased.
- Require two consecutive successful trials by default before accepting gains.
- Keep I at zero unless `allowIntegral` is enabled. Even then, require eight P/D
  adjustments, three consecutive undershoots, P >= min(12, maxKp), no meaningful crossings,
  and a stationary final error within the 3-inch integral range. Increase I by
  0.001 up to 0.05. If overshoot or oscillation appears, reduce existing I before
  increasing D.

These are bounded heuristics. Friction, battery condition, odometry error, and
motion exit settings can prevent convergence. Treat successful gains as a
starting point for real-robot verification and manual refinement.

## Results and cancellation

Only `AutonTuneStatus::COMPLETE` means the gains passed the required confirmation
trials and the robot successfully returned. On other outcomes, `result.gains`
contains the best safely returned trial's gains, or the starting gains if none
completed. This recommendation is diagnostic and must not be treated as accepted.
The gains actually attempted remain in each `TrialReport`.

`result.trials` records each trial's gains, forward distance, final error/speed,
peak overshoot, crossings, elapsed time, settlement, timeout, and
`returnedToStart`. Return movement does not overwrite the forward measurements.

`toLemLib(result, baseSettings)` returns optional `lemlib::ControllerSettings`.
It returns no settings for an unsuccessful run and replaces the gains and windup
range on success. `result.integralWindupRange` records the range used in the
confirmed trials, so nonzero I retains the same integration cutoff when applied.
Exit thresholds and slew remain those of baseSettings. Use the returned settings
when constructing your chassis. Tuning temporarily
changes the chassis's lateral PID and restores it afterward; it does not
permanently apply the result.

`run()` blocks. Another task can call `tuner.abort()` during movement. Abort is
latched: an idle abort cancels exactly the next run, and the owning run clears it
on exit, including exception unwinding. A concurrent `run()` returns
`ALREADY_RUNNING` without consuming the active session's abort flag.
Between trials, an idle wait allows asynchronous motion bookkeeping to catch
up for at most max(postStopMs, 100 ms). Trial, return, and idle loops also have a
1000-poll watchdog so a stalled clock cannot make a deadline loop run forever.
An optional `BeforeTrial` callback can print progress or return false to abort. It must not
reposition the robot, and a blocking callback must handle its own cancellation.
Keep the tuner and adapter alive throughout the run. Do not run driver control,
other tuners, or unrelated LemLib motions concurrently.

Calling from enabled `opcontrol()` is supported. Call tuning synchronously at
the start of opcontrol, before the joystick loop, as shown in
`examples/opcontrol.cpp`. The session starts without button presses, adjusts
gains and returns automatically between trials, and stops on completion, the
configured trial limit, or a fault. Normal driver control resumes afterward.
Do not put the tuning call inside the joystick loop or retry it on every tick.
After dispatch, the runner allows up to 250 ms to observe asynchronous motion
startup before returning `DRIVE_UNAVAILABLE`. An initial idle poll alone does
not mean the trial has completed. Abort, disable and sensor checks still apply
while waiting. Use `statusName(result.status)` to report every outcome.

## Measurement limits and stopping

Distance comes from chassis odometry projected onto the fixed starting heading,
not an independent measurement of ground travel. LemLib headings use clockwise
degrees with zero along +y. Calibrate odometry before tuning; wheel slip affects
both gain selection and the return position.

Default settling requires 500 ms within the forward range and below 1 inch per
second. After a forward motion ends or is cancelled, observation continues for
at least the larger of `postStopMs` (300) and `settleMs` (500). Existing LemLib
exit conditions and slew settings remain in effect and can end a motion before
the requested tuning range is reached.

Both directions check abort, disable, invalid poses, sensor jumps above 120 inches
per second, displacement beyond 36 inches, sideways deviation beyond 4 inches,
and heading deviation beyond 20 degrees. Forward stopping cancels and waits for
the owned LemLib motion; return stopping commands zero tank output. Braking and
coasting depend on the robot's brake mode. Choose return power that overcomes
friction while allowing the robot to stop inside the return tolerance. Keep the
forward and backward path clear and have an immediate stop mechanism available.

The adapter uses LemLib's exposed `lateralPID`. Since its gains are const in the
referenced API, the adapter copies the original PID and reconstructs the idle
object with the candidate gains. It waits for motion completion before restoring
the original. No PROS or LemLib source files are modified.


## Multiple targets and angular tuning

Set `testDistances = {6, 24, 72}` and increase `maxTravelInches` beyond 72.
Increase returnTimeoutMs for longer travel: fixed low-power returns can take
considerably longer than forward trials. Each candidate must pass every target
for confirmationTrials complete cycles. Failure resets confirmations and starts
the next candidate at the first target. maxTrials counts all individual targets;
allow at least targets.size() * confirmationTrials plus search attempts.

For turns use tuneAngular(chassis, config), distanceInches=90,
testDistances={90,170}, maxTravelInches=210, maxSensorSpeed=400 and generous
trial/return timeouts. Angular distance, tolerance, deadband, travel and speed
settings use degrees and degrees/second despite their legacy inch field names.
Absolute headings are normalized to [0, 360) degrees before the core requests
an angular motion and again at LemLib's float precision. Thus 720 becomes 0,
-260 becomes 100, and a rounded 360 becomes 0. Negative or multi-revolution
sensor headings do not change the requested turn distance. Turns are clockwise; the
return uses fixed counterclockwise tank power. Angular
progress is unwrapped across 0/360. A 180-degree target is rejected; use a
smaller angle such as 170 degrees.
In angular mode, maxCrossTrackInches bounds total translational drift in inches. Do not reuse a
lateral independent-distance callback for angular trials.
`maxSensorSpeed` is the angular jump limit in degrees/second;
`maxTranslationalSensorSpeed` separately bounds x/y jumps in inches/second
(default 120), during both trials and returns. Lateral tests use `maxSensorSpeed`
for translation as before. Explicit `testDistances` replace `distanceInches`;
only the targets actually used are validated against travel and tolerance limits.

## Search policy

On an unsuccessful session, gains refer to the best safely returned trial,
ranked by (absolute final error + peak overshoot + crossings * tolerance) / target.
`bestTrial` identifies it. Normalization makes different target lengths comparable.
This is a diagnostic fallback, not accepted gains; a candidate's success on one
target does not imply success on every target. Only COMPLETE enables conversion
and persistence. The default policy retains the original deterministic schedule.

bisectOnReversal optionally tests the midpoint of the two preceding candidates
when the adjustment direction reverses between undershoot and overshoot. This is
bracket refinement, not a guarantee of monotonic behavior or convergence.
adjustmentPolicy overrides the next candidate after a failed trial that requires
a gain adjustment, and receives tested gains, final error, overshoot and trial
index. It is not called for unsettled same-gain retries or after the last allowed
trial. Its actual gain changes determine reversal and integral-gating history. All returned gains must be
finite and within configured caps; otherwise GAIN_LIMIT stops the run.
Gain caps, policy candidates, and the windup range must also fit LemLib's float
parameters. Oversized caps produce INVALID_CONFIG before movement; out-of-range
policy candidates produce GAIN_LIMIT before they can be installed.

## Readiness, clearance and independent measurement

The adapter checks finite pose, idle motion and competition availability
before moving.

LemLib exposes no public calibrated flag. Pass a readiness callback to the adapter
or tuneLateral/tuneAngular, e.g. `[&] { return calibrationComplete && !imu.is_calibrating(); }`.
A false result produces NOT_CALIBRATED. The default assumes the caller completed
calibration; a finite pose alone cannot establish calibration. Set readiness only
after your calibration routine completes successfully.

requireConfirmation is a once-per-session boolean callback before motion. Use it
to wait for a deliberate controller button press with your own timeout/cancel
handling. False returns ABORTED. It cannot detect obstacles or certify clearance.

independentDisplacement returns signed displacement from the saved session origin,
in inches (or degrees for angular). For a forward-facing wall sensor use
(startingRangeMm - currentRangeMm) / 25.4 after validating the sensor's range and
geometry. The callback runs during forward movement, coast observation, and the
entire return, including a check before reverse power starts. It must keep
reporting signed displacement from the same session origin as the robot reverses;
do not reset its baseline between trials. Sample every 10ms; callbacks must be prompt. NaN produces SENSOR_FAULT;
disagreement above maxMeasurementDisagreement produces MEASUREMENT_DISAGREEMENT.
The robot stops without reversing on these faults. Trials retain the last
independent reading. Odometry still determines the normal return; independent
measurement is a fault check, not sensor fusion.

## Saving results

Include autotune/output.hpp. `saveAcceptedGains(result, "/usd/guantune.json")`
writes schema 2 JSON containing mode, accepted gains, and `windupRange` to the SD
card via a temporary file and rename. External readers of schema 1 must be updated
to accept schema 2 and apply its windup range; schema 1 did not record that value.
Check its boolean result: an absent card, full filesystem or failed rename returns
false. A failed run does not overwrite accepted gains. Use one writer per path;
power-loss durability depends on the filesystem. `controllerSettingsSnippet(result)`
returns a copy-pasteable ControllerSettings copy and gain assignments preserving
your baseSettings exit thresholds and slew, while applying the tested gains and
windup range. It is empty on failed results or unrepresentable PID parameters.

The `guantune` namespace alias is available wherever types.hpp is included.
The older relay constructors, conversion and Result::print are deleted, yielding
compile-time diagnostics instead of unresolved symbols. They are not part of the
implemented autonomous tuning API.

## Battery monitoring

`minimumBatteryMillivolts` defaults to 7000. Before a session or trial moves,
readings below the threshold return `LOW_BATTERY` immediately. During trial,
coast, and return sampling, a brief dip is tolerated: stop after 100 ms of
consecutive low readings, or immediately below 85% of the threshold. A healthy
reading resets that timer. NaN, infinity, and nonpositive readings produce
`SENSOR_FAULT`. `TrialReport` retains starting and minimum forward-trial voltage.
Pick a threshold suitable for your robot; these checks do not estimate charge.

## Optional telemetry

Set `enableTelemetry=true` to collect phase, target, position, signed velocity,
battery voltage and optional motor readings in `result.telemetry`. Recording
is disabled by default. `telemetryPeriodMs` defaults to 20 and must be at least
10 ms. New trials and phase transitions are recorded even within that period.
`maxTelemetrySamples` defaults to 4096 and must be in [1, 100000] when recording
is enabled. Initial reservation is capped at 4096; the vector may grow to the
configured limit. At capacity, recording stops and `telemetryTruncated` is set,
but tuning continues. Large caps consume robot RAM.

`telemetryMotorPorts` contains signed PROS ports in [-21, -1] or [1, 21], with
no repeated physical motor. Motor I/O runs on the first record and every fifth
record thereafter; records in between have empty motor arrays. Missing/invalid
motor readings are absent values. They are not copied forward as fresh data.
An empty port list still records chassis and battery information.

LemLib does not expose its live PID command. Supply a prompt, thread-safe
`telemetryCommandProbe` if your own instrumentation has it. Otherwise forward
command fields remain absent; coast/settle commands are zero, and return power
is known. The nominal `12000 * power / 127` return voltages are labeled with
`voltageIsPowerEquivalent=true`; they are not measured or battery-compensated
voltage commands. Signed velocity uses inches/s laterally or degrees/s angularly;
position is displacement from the fixed origin. No elapsed time means no velocity.

Call `saveTelemetryCsv(result, "/usd/guantune-telemetry.csv")` after the run to
avoid SD writes in the control loop. Failed sessions can also be exported. CSV
has one row per motor on motor-sampled records and one chassis-only row otherwise;
absent values become empty fields. No samples returns false without opening a
file. Both JSON and CSV exports use temporary-file/rename and report write failure.
See `examples/telemetry_autonomous.cpp` for the recording setup.
