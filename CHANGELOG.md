# Changelog

## Unreleased

- Add opt-in bounded telemetry for trials/coast/return/settle, per-motor PROS
  voltage/current/temperature reads, optional command instrumentation, and CSV
  export of successful or failed sessions. Keep telemetry off by default.
- Check independent displacement throughout returns, including before reverse
  power, and detect translational sensor jumps during angular returns.
- Preserve the tested integral windup range in results, settings conversion,
  snippets, and schema 2 saved JSON; external schema 1 readers need updating.
- Validate effective target lists without constraining unused distanceInches,
  and reject PID parameters that cannot be represented by LemLib floats.
- Add return-fault, target-override, numeric-boundary and export regressions.

- Add GCC/Clang C++17/C++20 sanitizer CI and pinned/latest real PROS builds.
- Add multi-target confirmation, angular trials, best-trial fallback, optional
  bracket refinement and a bounded custom gain policy.
- Add calibration/clearance hooks, battery floors and per-trial voltage records,
  plus independent displacement disagreement checks.
- Restore const-gain PIDs with an idle RAII guard; exercise exception cleanup,
  safety paths and seeded motor/friction/noise simulations.
- Add accepted-gain SD persistence, ControllerSettings snippets, simple tuning
  helpers and the guantune namespace alias. Delete unfinished relay entry points.
- Split reference/troubleshooting/compatibility docs from the README; add hardware
  log protocol, contribution guidance, issue templates and formatting config.

- Import real PROS/LemLib dependencies through external headers and Conductor;
  remove test-only redefinitions and add configurable upstream build setup.
- Return with configurable low-power LemLib tank commands, checking chassis
  pose until near the saved start; remove moveToPoint from the return phase.
- Delegate forward and backward motion/PID control to LemLib and read chassis
  poses; remove the manual PID and tank-output implementation.
- Return to the saved starting position after every valid trial using original
  gains. Abort the search if return positioning, safety, or timeout checks fail.
- Make the between-trial callback optional and add return status to trial reports.
- Add `range` (acceptable distance error in inches) and the `AutonTuner` alias;
  document `.range = 0.1, .maxTrials = 15` construction and trial-limit behavior.
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
