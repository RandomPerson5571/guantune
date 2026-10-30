# Supplied implementation integration

The replacement supplied on 2026-10-29 is the behavioral baseline for the tuner.
Its logic is integrated into the existing modules rather than copied into one
large source file. The original public session names remain available.

## Behavior adopted

- **Lifecycle:** `ALREADY_RUNNING` distinguishes a second call from unavailable
  hardware. An abort request is latched, honored by the current or next run,
  and consumed only when the owning run exits.
- **Timing:** bounded waits tolerate delayed idle bookkeeping. Poll-count
  watchdogs prevent stalled clocks from defeating trial/return deadlines.
- **Gain selection:** coarse steps grow by 20% above small gains; fine steps
  grow by 2%, with their supplied minimums. Overshoot at the D cap reduces P.
  Integral gating can use a configured P cap below 12. Custom-policy history
  follows the actual returned gains.
- **Retries:** within-tolerance but unsettled motion retries the same gains up
  to five times, within the existing trial budget. The sixth failure stops.
- **Return:** stop with two sample periods of measured lead, observe coast,
  and creep again if the robot comes to rest short of the tolerance window.
  Creep never gets a fresh return deadline.
- **Recommendations:** scores are normalized by target length. Every unsuccessful
  run recommends the best safely returned trial, or the initial gains when no
  trial completed. Those gains are diagnostics, not accepted results.
- **Validation:** targets need overshoot headroom, angular targets must be below
  180 degrees, telemetry has a 100000-sample cap, and ports must be valid/unique.
- **Battery:** resting checks are strict; under-load readings tolerate dips
  shorter than 100 ms unless below 85% of the configured threshold.
- **Telemetry:** initial reservation is at most 4096 samples. Motor I/O runs
  every fifth recorded sample, while chassis/command recording follows its
  configured cadence. Recording stops at capacity while tuning continues.

## Structure and retained corrections

`self_tuning_auton.cpp` coordinates the session. `session_control.cpp` handles
readiness and waits; `battery_monitor.cpp` owns voltage decisions;
`gain_search.cpp` owns search history; `tuning_motion.cpp` and `tuning_return.cpp`
own the two movement phases. `telemetry.cpp` owns recording, while JSON and CSV
serialization run in separate output sources after movement. See
[architecture.md](architecture.md) for the complete code map.

Several integration corrections preserve the existing audited behavior:

- Targets are passed into trials without temporarily mutating configuration.
- One motion tracker spans the whole session. Pose/time handoff is independent
  of telemetry, and angular return acceptance checks the fixed origin.
- Startup detection measures movement since this trial began, so a residual
  return offset cannot make a queued motion appear complete.
- Return timeouts must accommodate both settling and coast-observation periods.
- The final failed trial does not call a policy for an unused next candidate.
- Named search/return states replace signed action codes and paired flags.
- Policy history also recognizes decreased D as a P/D change, so its direction
  and integral gate reflect the actual adjustment. A policy-created reversal
  enables fine steps even when the overridden heuristic did not reverse.
- Cleanup destructors make a best-effort stop and restore without masking an
  original exception. Hardware cleanup still has a nonthrowing contract.

The supplied battery and telemetry behavior requires those fields and operations
in `TuningDrive`, configuration, results and the real LemLib adapter; they are
included in this integration. `saveTelemetryCsv` is available again. The angular
example uses 90/170-degree targets under the new validation rule.

## Tests

The host suite includes lifecycle/concurrency, strict and sagging battery reads,
frozen clocks in both motion phases, bounded/cancellable idle waits, cleanup
exceptions, policy history, damping-cap backoff, retry limits, relative ranking,
recording limits, motor cadence, clock wrap, command visibility and CSV export.
Existing delayed-start, return-fault and angular-boundary regressions remain.
Shared simulated drive fixtures live in `test/support/`; they implement the
actual `TuningDrive` interface without recreating upstream namespaces.

Simulation and compile/link checks do not establish physical robot convergence.

## Verification — 2026-10-29

Clang C++17 and C++20 sanitizer suites passed, including the restored telemetry
regressions, new lifecycle/search tests, existing seeded physics models, and the
real ARM template-optimization regression. The earlier four audit probes still
show corrected outcomes. Native settings conversion passed against actual
dependency headers using the documented Apple Clang fmt workaround.

With PROS 4.2.2, LemLib 0.5.6 and the installed ARM GCC 14.3.1 toolchain, the
library and optimized template built successfully. A fresh consumer installed
the template, compiled all four examples, and linked both monolithic and
hot/cold binaries. This local compiler differs from CI's pinned ARM GCC 15.2.1;
the published base's latest-stable CI job is successful, but these uncommitted
changes have not run in remote CI. No robot upload or physical test was performed.
