# Troubleshooting

## No movement from opcontrol

Enabled driver control is supported. Run tuning synchronously at the start of
opcontrol and pause all other motor-writing tasks for its duration; see
[`examples/opcontrol.cpp`](../examples/opcontrol.cpp). Do not launch tuning in a
background task while the joystick loop continues sending zero commands.

Print `guantune::statusName(result.status)` immediately after the call, including
failed runs. `ALREADY_RUNNING` means another call owns this tuner.
`DRIVE_UNAVAILABLE` means the robot is disabled, another motion is active, an
idle wait expired, or no motion was observed within the 250 ms startup window.
`INVALID_CONFIG` means configuration was rejected before movement: for example,
`maxOutput` below 60, insufficient overshoot headroom, a 180-degree angular
target, invalid telemetry limits/ports, or insufficient trials
for confirmation. `NOT_CALIBRATED` means the readiness callback returned false;
`SENSOR_FAULT` requires inspecting odometry and independent sensor readings.

The runner now waits for asynchronous motion startup instead of cancelling on
the first idle poll. A delayed-start regression reproduced that premature
cancellation; this does not establish the cause of every hardware no-movement
report. If no starting message appears in the example, check that opcontrol
calls the tuning function and that the correct program is installed. If the
call never returns, capture the last message and
check the motion task/odometry rather than treating a compile/link pass as a
successful robot test.

## RETURN_FAILED and the final robot state

The runner cancels/joins owned motion, sends zero manual drive output and restores
the original PID. It does not reset pose, force a brake mode, or guarantee the
robot reached home. The robot remains at its last physical position; residual
coasting depends on brake mode and surface. Inspect and manually reposition only
after the session stops. No further trials run after return failure.

Check sideways/heading drift, return tolerance, brake mode, return power and the
return timeout. Long targets need longer return deadlines. A tolerance narrower
than one sample of travel may be skipped; reduce return power or widen tolerance.
A stalled return needs enough power to overcome friction. Do not simply raise
power when failure is caused by overshooting home.

## Gains oscillate or the run reaches its limit

Check odometry calibration, wheel slip and battery consistency first. Use multiple
targets, more trials, bisectOnReversal or a custom policy. Inspect bestTrial and
all reports; the fallback gains at TRIAL_LIMIT remain unaccepted. Integral is an
explicit last resort and can amplify windup. Fixed heuristic steps cannot promise
convergence for every drivetrain or every LemLib motion exit configuration.

## Choosing returnPower and brake mode

Start at a low power that moves reliably under the robot's actual payload; the
allowed range is 1–40. Test clearance and stopping distance before an unattended
session. Higher gearing, traction and friction may need more power. More momentum
requires more stopping room. Use a consistent brake mode (typically brake rather
than coast), set by your robot's existing motor/chassis configuration before
running. The tuner never changes brake mode. Hold mode may create different
settling behavior; compare it explicitly across sessions. Verify real stopping
behavior instead of assuming zero output stops instantly.

## LOW_BATTERY and battery readings

Resting checks are strict. During movement, a low reading must persist for
100 ms, unless it is below 85% of minimumBatteryMillivolts. Inspect recorded
starting/minimum voltage, battery condition, connections and load. Invalid
voltage readings produce SENSOR_FAULT; brief tolerated sag can still appear
as the recorded minimum in an accepted trial.

## NOT_CALIBRATED and measurement faults

Set the readiness callback from completed calibration and sensor health, not a timer.
Check units and wall geometry for independent sensors; invalid readings should
return NaN. MEASUREMENT_DISAGREEMENT stops without a return because ground travel
is uncertain. SENSOR_FAULT includes nonfinite pose, implausible speed, invalid battery reads,
and a clock that remains unchanged for 1000 polls. Do not loosen limits until the sensor fault is understood.

## Build errors or unfinished interfaces

Use the supported dependency pair and compile the actual adapter, not just the
host core. See compatibility.md for version-guard overrides used only in audits.
RelayTuner/Tuner and relay Result::print are deliberately deleted until implemented;
use SelfTuningAuton, tuneLateral or tuneAngular instead.
