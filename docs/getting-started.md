# Getting started with Guantune

See the [code structure and flow diagrams](code-map.md) for a visual overview.

Guantune tries PID gains on your existing LemLib chassis, measures each motion,
and returns the robot to its starting position between trials. Use it to tune
lateral driving or angular turning. Your project still provides the motors,
sensors, odometry, and chassis configuration.

## Install

Use PROS 4.2.2 and LemLib 0.5.6. Copy the complete `include/autotune/` and
`src/autotune/` directories into the matching directories in your robot project.
For this source-copy setup, add `-DGUANTUNE_LEMLIB_VERSION=506` to your project's
`EXTRA_CXXFLAGS`. Build the robot project with `pros make`.

For template builds and dependency details, see [compatibility](compatibility.md).
The installed template is named `autotune`; the public C++ namespace can be
written as `guantune`.

## Run a lateral session

After calibrating the robot, call this from a dedicated test mode using your
existing `chassis`:

```cpp
#include "autotune/api.hpp"

// Inside your test mode, after calibration:
guantune::AutonTuneConfig config;
config.distanceInches = 24;
config.maxTravelInches = 36;
const auto result = guantune::tuneLateral(chassis, config);

if (result.succeeded()) {
    const auto gains = result.gains; // kP, kI, kD
    const auto windup = result.integralWindupRange;
    // Save these through your own display/logging code, then update
    // your LemLib controller configuration for the next build.
    (void)gains;
    (void)windup;
}
```

The call blocks until the session ends. Give it exclusive chassis access and
enough clear space for both trials and returns. Joystick loops and other tasks
must wait until it returns. See the [opcontrol example](../examples/opcontrol.cpp)
for placement before a driver loop.

## Run an angular session

Use a separate session with degree-based targets and limits:

```cpp
guantune::AutonTuneConfig config;
config.angleDegrees = 90;
config.maxTravelInches = 210; // This field uses degrees in angular mode.
config.maxSensorSpeed = 400; // Degrees per second in angular mode.
const auto result = guantune::tuneAngular(chassis, config);
```

Several field names retain `Inches` even in angular mode. Check the
[configuration reference](tuning.md) before changing tolerances or return limits.

## Use the result

Only `result.succeeded()` (status `COMPLETE`) means all confirmations and the
final return passed. The tuner restores the original PID when it exits; returned
gains are not applied permanently. Save successful gains and the windup range
in your LemLib configuration, then check normal robot motions with that setup.

For unsuccessful runs, inspect `result.status` and `result.trials` and follow
[troubleshooting](troubleshooting.md). For progress callbacks or cancellation,
see the runner example in the [README](../README.md). Physical validation is
still pending; see the [hardware validation guide](hardware-validation.md).
