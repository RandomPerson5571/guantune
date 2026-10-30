# Changelog

## Unreleased

- Separate session orchestration, configuration validation, motion tracking,
  gain search, hardware operations, and persistence. Split public types into
  focused headers while retaining the existing session entry points.
- Keep configuration immutable during trials; use named gain adjustments and
  explain coordinate, timing, ownership, and PID restoration decisions.
- Fix angular phase-boundary tracking, repeated queued motion startup, invalid
  return timing, and unused gain adjustments after the trial limit. Add
  regression cases and a code map for maintainers.

- Normalize angular targets in the core and LemLib adapter to [0, 360),
  including float rounding at 360; retain unwrapped displacement for tuning
  and returns. Cover negative/multiple-turn starts in complete sessions.

- Integrate the supplied replacement algorithm: proportional gain steps, bounded
  unsettled retries, P backoff at the D cap, normalized best-trial ranking,
  latched aborts, ALREADY_RUNNING, idle waits, and clock-stall watchdogs.
- Add predictive return stopping with bounded creep retries. Restore strict
  resting-battery checks, brief-sag tolerance under load, bounded telemetry,
  throttled motor sampling, and CSV export in separate implementation modules.
- Require overshoot headroom for targets and angular targets below 180 degrees.

- Start the opcontrol example automatically without controller button presses;
  retain the trial limit and stop/fault behavior, then resume driver control.

- Wait for asynchronous motion startup before treating idle as completion;
  cover delayed/missing startup, abort and clock wrap. Add readable result
  statuses and a blocking opcontrol example that pauses joystick writes.
- Test PROS kernel 4.2.2 / ARM GCC 15.2.1 / LemLib 0.5.6 locally and pin it in
  CI, retaining the 4.2.1 job. Check both monolithic and hot/cold consumer links.
- Strip debug information only from the distributed library and compress the
  template ZIP; retain the local debug archive. Document measured size costs.
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
- Add calibration/clearance hooks and independent displacement disagreement checks.
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
