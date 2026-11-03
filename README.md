# Guantune

New here? Read the [Getting started guide](docs/getting-started.md) for setup and a first run.
See the [code structure and flow diagrams](docs/code-map.md) for a visual overview.

Guantune is an automatic PID-tuning wrapper for an existing **LemLib chassis**.
It installs candidate gains into LemLib's lateral or angular PID, requests
LemLib motions, reads `chassis.getPose()` to measure the result, and returns
to the saved starting position between trials. Your project owns the chassis,
motors, sensors, calibration, odometry, brake mode and motion settings.

Guantune does not implement a drivetrain, odometry system or motor PID loop.
LemLib 0.5.6 calls its position getter `getPose()`; there is no chassis
`getPosition()` method in that version. The tuner uses `moveToPoint()` for
lateral trials, `turnToHeading()` for angular trials and low-power `tank()`
commands for returns. All drivetrain commands go through the supplied chassis.

## Use

Use PROS 4.2.2 and LemLib 0.5.6. Copy the complete `include/autotune/` and
`src/autotune/` trees into your robot project, or build/install the PROS template.
All headers, including internal declarations, are under `include/`; `src/`
contains only implementation `.cpp` files.

After calibration, with exclusive chassis access and a clear trial/return path:

```cpp
#include "autotune/api.hpp"

guantune::AutonTuneConfig config;
config.distanceInches = 24;
auto result = guantune::tuneLateral(chassis, config);
if (result.succeeded()) {
    const auto& pid = result.gains; // pid.kP, pid.kI, pid.kD
    // Display pid or send it through your own telemetry connection.
}
```

For angular tuning, configure degrees and allow enough travel/speed for a turn:

```cpp
config.angleDegrees = 90;
config.maxTravelInches = 210; // Angular travel limit, in degrees.
config.maxSensorSpeed = 400; // Angular jump limit, degrees/second.
auto result = guantune::tuneAngular(chassis, config);
```

Multiple-target runs use `testDistances`
for inches or `testAngles` for degrees. See [the tuning reference](docs/tuning.md)
for travel limits and angular configuration.

For progress callbacks or cancellation, construct the runner directly:

```cpp
guantune::SelfTuningAuton tuner(chassis, config, [&] {
    return calibrationComplete && !imu.is_calibrating();
});
auto result = tuner.run([](unsigned trial, const guantune::Gains& gains) {
    // Optionally report trial/gains through your own display or telemetry code.
    (void)trial;
    (void)gains;
    return true;
});
// Another task may call tuner.abort(); keep tuner alive until run() returns.
```

`run()` blocks. In driver control, call it once before the joystick loop, as in
[the opcontrol example](examples/opcontrol.cpp). Other tasks must also yield
chassis ownership. The original PID is restored when tuning exits; accepted
gains are returned in an `AutonTuneResult` object for deliberate application to
your chassis configuration. Read `result.gains.kP`, `.kI`, `.kD`,
`result.integralWindupRange`, and `result.status`. Optional recorded samples are
available in `result.telemetry`. Guantune does not write files, format output,
print results, or select a screen or telemetry transport.
Only `COMPLETE` means all confirmations and the final return succeeded.

## Layout and validation

- [Architecture and ownership](docs/architecture.md)
- [Configuration, gain search and measurement limits](docs/tuning.md)
- [Build compatibility](docs/compatibility.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Tests and their limits](test/README.md)
- [Physical validation protocol](docs/hardware-validation.md)

Run `bash scripts/test_host.sh` for policy, configuration, position-analysis,
telemetry checks under ASan/UBSan. Motion execution is compiled
against real PROS/LemLib dependencies and linked through the template consumer
check. The removed generic-drive simulations do not validate the current
chassis wrapper. Physical sessions have not been recorded, and compile/link
checks do not establish safe motion or convergence on a robot.

Guantune is MIT licensed; upstream dependencies retain their own licenses.
