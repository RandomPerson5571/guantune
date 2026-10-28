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
- `returnTimeoutMs`: return deadline including settling; default 5000 ms.
- `trialTimeoutMs`: forward trial deadline before stopping and observing coast;
  default 5000 ms.
- `allowIntegral`: enable last-resort integral tuning; default false.
- `maxOutput`: forward LemLib motion limit, 60–127; default 60. Lower values are
  rejected because [LemLib 0.5.6 moveToPoint](https://github.com/LemLib/LemLib/blob/v0.5.6/src/lemlib/chassis/motions/moveToPoint.cpp)
  can raise its limit to at least 60 near the target.

Additional limits, gain caps, and settling settings are declared in
[`AutonTuneConfig`](include/autotune/self_tuning_auton.hpp). Invalid configuration
returns `INVALID_CONFIG` before movement. The default travel limit is 36 inches;
raise it appropriately if increasing `distanceInches` beyond that limit.

## Trial and return sequence

1. Save the initial `chassis.getPose()` as a fixed origin for the session.
2. Temporarily apply the candidate gains to LemLib's lateral PID and use
   `chassis.moveToPoint()` for the forward target along the starting heading.
3. Sample `chassis.getPose()` every 10 ms. Record projected distance, peak
   overshoot, meaningful target crossings, and settling behavior.
4. Stop the motion and observe coasting. Restore the original lateral PID.
5. Back up slowly with `chassis.tank(-returnPower, -returnPower, true)`, continuing
   to read chassis pose. Stop when within the return tolerance and wait for the
   robot to settle there.
6. Adjust gains or confirm the current gains, then start the next forward trial.

If a trial moves only 20 inches, the robot backs toward its actual saved start;
it does not command a fixed 24-inch reverse move. The return does not use
`moveToPoint()` or a custom PID loop. With a starting pose of (0, 0), it stops
near (0, 0). With another starting pose, it uses that saved coordinate. It never
resets odometry to make a return appear complete.

A return that times out, coasts outside tolerance, passes the start without
entering tolerance, or cannot reach it because of sideways error produces
`RETURN_FAILED`. No subsequent trial starts. Returns also follow the final
confirmation or trial-limit run. Aborted, disabled, or unsafe trials stop without
automatically reversing.

## Gain adjustments

- Start each session at P=1.1, I=0, D=1.1.
- Increase D for overshoot beyond the configured forward range, or at least
  three target crossings. Crossings use a default +/-0.5-inch deadband.
- Increase P for unsuccessful trials without excessive overshoot or oscillation.
- Increase P/D by 3 below a gain of 6, by 1 from 6 to below 12, and by 0.1 from
  12 onward. Switching between P and D adjustments selects 0.1 refinement for
  the rest of the session.
- Require two consecutive successful trials by default before accepting gains.
- Keep I at zero unless `allowIntegral` is enabled. Even then, require eight P/D
  adjustments, three consecutive undershoots, P >= 12, no meaningful crossings,
  and a stationary final error within the 3-inch integral range. Increase I by
  0.001 up to 0.05. If overshoot or oscillation appears, reduce existing I before
  increasing D.

These are bounded heuristics. Friction, battery condition, odometry error, and
motion exit settings can prevent convergence. Treat successful gains as a
starting point for real-robot verification and manual refinement.

## Results and cancellation

Only `AutonTuneStatus::COMPLETE` means the gains passed the required confirmation
trials and the robot successfully returned. On other outcomes, `result.gains`
contains the last tested gains (or the best measured candidate at TRIAL_LIMIT) and should not be treated as an accepted result.

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

`run()` blocks. Another task can call `tuner.abort()` during movement. An optional
`BeforeTrial` callback can print progress or return false to abort. It must not
reposition the robot, and a blocking callback must handle its own cancellation.
Keep the tuner and adapter alive throughout the run. Do not run driver control,
other tuners, or unrelated LemLib motions concurrently.

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
testDistances={90,180}, maxTravelInches=210, maxSensorSpeed=400 and generous
trial/return timeouts. Angular distance, tolerance, deadband, travel and speed
settings use degrees and degrees/second despite their legacy inch field names.
Turns are clockwise; the return uses fixed counterclockwise tank power. Angular
progress is unwrapped across 0/360, including the return from 180 degrees.
In angular mode, maxCrossTrackInches bounds total translational drift in inches. Do not reuse a
lateral independent-distance callback for angular trials.
`maxSensorSpeed` is the angular jump limit in degrees/second;
`maxTranslationalSensorSpeed` separately bounds x/y jumps in inches/second
(default 120), during both trials and returns. Lateral tests use `maxSensorSpeed`
for translation as before. Explicit `testDistances` replace `distanceInches`;
only the targets actually used are validated against travel and tolerance limits.

## Search policy

At TRIAL_LIMIT, gains refer to the best safely returned trial, ranked by absolute
final error + peak overshoot + crossings * tolerance. bestTrial identifies it.
This is a diagnostic fallback, not accepted gains; a candidate's success on one
target does not imply success on every target. Only COMPLETE enables conversion
and persistence. The default policy retains the original deterministic schedule.

bisectOnReversal optionally tests the midpoint of the two preceding candidates
when the adjustment direction reverses between undershoot and overshoot. This is
bracket refinement, not a guarantee of monotonic behavior or convergence.
adjustmentPolicy overrides the next candidate after a failed trial and receives
tested gains, final error, overshoot and trial index. All returned gains must be
finite and within configured caps; otherwise GAIN_LIMIT stops the run.
Gain caps, policy candidates, and the windup range must also fit LemLib's float
parameters. Oversized caps produce INVALID_CONFIG before movement; out-of-range
policy candidates produce GAIN_LIMIT before they can be installed.

## Readiness, clearance, battery and independent measurement

The adapter checks finite pose, idle motion, competition availability and battery
before moving. minimumBatteryMillivolts defaults to 7000; battery values are mV.
Trials record starting/minimum voltage; both trial and return stop on LOW_BATTERY.
Invalid voltage readings produce SENSOR_FAULT. Pick a floor suitable for your robot.

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

## Optional telemetry

Telemetry is disabled by default. Enable it explicitly and provide the signed
PROS drivetrain motor ports (negative for reversed motors):

```cpp
guantune::AutonTuneConfig config;
config.enableTelemetry = true;
config.telemetryMotorPorts = {-1, -2, 3, 4}; // Replace with your chassis ports.
config.telemetryPeriodMs = 20;
config.maxTelemetrySamples = 4096;
auto result = guantune::tuneLateral(chassis, config);
bool saved = guantune::saveTelemetryCsv(result, "/usd/guantune-telemetry.csv");
```

`result.telemetry` contains timestamps, session elapsed time, zero-based trial
index, mode, phase, target/actual position, signed velocity, battery voltage,
commanded left/right voltage, controller output, and per-motor measured voltage,
current, and temperature. Positions are displacement from the fixed session
origin in inches (lateral) or unwrapped degrees (angular). Returns target zero.
Velocity is the signed finite difference of control samples, in inches/second or
degrees/second; it is absent when no time elapsed. Timestamps use the raw drive
clock; elapsed time handles its 32-bit wrap. Sampling starts with the first trial.

The LemLib adapter reads motor voltage with `pros::c::motor_get_voltage`, current
with `motor_get_current_draw`, temperature with `motor_get_temperature`, and
battery voltage with `pros::battery::get_voltage`. Current uses mA, voltage mV,
and temperature degrees Celsius. Each motor is recorded separately. An API error
produces an absent reading, exported as an empty CSV field. Empty motor ports
still permit chassis/battery logging. Duplicate physical ports and ports outside
1–21 are rejected before movement when telemetry is enabled.
See the [PROS motor telemetry reference](https://pros.cs.purdue.edu/v5/pros-4/group__cpp-motors.html)
for the underlying measurement units and error behavior.

**Command visibility:** LemLib 0.5.6 exposes neither its live PID output nor the
last voltage/power commands from asynchronous motions. Forward trial command
fields remain absent unless `config.telemetryCommandProbe` supplies them from
your own controller instrumentation. This callback returns `TelemetryCommand`
with optional `controllerOutput`, `leftCommandedMillivolts`, and
`rightCommandedMillivolts`. Use a prompt, thread-safe snapshot. Measured motor
voltage is never used to invent a requested voltage or PID output.

Without a probe, coast/return-settle samples have known zero commands; moving
return samples record the known negative return power. Their voltage fields are
the nominal `12000 * power / 127` equivalents of LemLib tank power, with
`voltageIsPowerEquivalent=true`. These are labeled equivalents, not measured
voltage or exact battery-compensated voltage requests. A supplied probe replaces
these defaults on every recorded sample. Controller output uses signed motor-power
units along the tuned axis; a custom probe must use that same convention.

Samples cover trials, coasting, returns, and return settling. The default period
is 20 ms and must be at least 10 ms. Phase/trial transitions are recorded even
when the usual period has not elapsed. Recording stops at `maxTelemetrySamples`
and sets `result.telemetryTruncated`; the tuning run continues. Increasing the
limit uses more RAM, especially with many motors. With telemetry disabled, no
motor telemetry reads or command-probe calls occur and the result buffer stays
empty. With telemetry enabled, reads are sequential snapshots, not atomic across
all motors. Telemetry work adds time to the control loop; probes must not block.

`saveTelemetryCsv` writes one row per motor per sample, repeating chassis values;
samples without motor readings still produce one chassis-only row. It includes
units, final session status, truncation and voltage-equivalent flags. It exports
failed sessions as well as accepted ones. No samples returns false without
opening a file. Write after `run()` returns so SD latency does not affect motion.
Check the return value for absent SD cards, write failures, and rename failures.
Invalid poses or aborts can end a run before another sample is recorded; the
final session status is retained in the CSV. See `examples/telemetry_autonomous.cpp`.

The `guantune` namespace alias is available wherever types.hpp is included.
The older relay constructors, conversion and Result::print are deleted, yielding
compile-time diagnostics instead of unresolved symbols. They are not part of the
implemented autonomous tuning API.
