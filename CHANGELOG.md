# Changelog

## Unreleased

- Implement `SelfTuningAuton` with 24-inch trials starting at P=1.1, I=0,
  D=1.1, coarse-to-fine P/D adjustments, repeated confirmation, and opt-in
  last-resort integral tuning.
- Add projected odometry measurement, overshoot/crossing reports, post-stop
  observation, aborts, output/travel/sensor limits, and a LemLib drive adapter.
- Add a controller-gated robot example and host tests of decisions and safeguards.
- Add the proposed autotune API, hardware-free core boundary, LemLib adapter
  boundary, PROS template configuration, and usage documentation.
- The original relay tuning, task, and printing methods remain unimplemented.

The Makefile reserves version 0.1.0; no hardware-validated release exists yet.
