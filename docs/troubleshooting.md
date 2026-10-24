# Troubleshooting

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

## LOW_BATTERY, NOT_CALIBRATED and measurement faults

Charge/change the battery and inspect trial start/minimum voltages. Set the
readiness callback from completed calibration and sensor health, not a timer.
Check units and wall geometry for independent sensors; invalid readings should
return NaN. MEASUREMENT_DISAGREEMENT stops without a return because ground travel
is uncertain. SENSOR_FAULT includes nonfinite pose, bad battery and implausible
speed. Do not loosen limits until the sensor fault is understood.

## Build errors or unfinished interfaces

Use the supported dependency pair and compile the actual adapter, not just the
host core. See compatibility.md for version-guard overrides used only in audits.
RelayTuner/Tuner and relay Result::print are deliberately deleted until implemented;
use SelfTuningAuton, tuneLateral or tuneAngular instead.
