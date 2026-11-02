# Validation

Run `bash scripts/test_host.sh` with CXX=g++/clang++ and CXXSTD=c++17/c++20.
The native suite uses ASan/UBSan and warnings-as-errors:

- `host/angle_test.cpp`: heading normalization and float rounding at 360.
- `host/config_test.cpp`: mode-specific target selection, inactive-field isolation,
  invalid targets, confirmation budgets, return timing, gain limits and ports.
- `host/gain_search_test.cpp`: policy history, reversal steps, D-cap P backoff,
  integral gates, bounded retries and normalized scores.
- `host/position_test.cpp`: fixed-origin projection, angular wrap, phase continuity,
  timestamp wrap, same-tick jumps and independent translational sensor limits.
- `host/telemetry_test.cpp`: bounded recording, phase/cadence handling, clock wrap,
  reduced motor sampling and invalid motor readings.
- `packaging/`: complete source/header preparation, metadata, destination protection,
  and ARM archive stripping/linking when the toolchain is available.

The old scripted and physics suites used the removed generic `TuningDrive` API.
They were replaced rather than presented as coverage of the real LemLib chassis.
The native suite does not test motion scheduling, concurrent aborts, cancellation,
PID restoration while tasks are running, return convergence or physical stopping.
These require the real PROS runtime and recorded robot sessions.

Build and install the real PROS template using `scripts/prepare_build.py` and
`scripts/test_template.py`, as described in `docs/compatibility.md`. The consumer
check compiles every public example and links monolithic and hot/cold binaries.
No movement is requested by the consumer's inert smoke program.

`compatibility/lemlib_settings_check.cpp` compiles against actual dependency
headers. It checks chassis-based runner construction, concrete wrapper type and
returned-result acceptance and settings conversion without creating hardware objects:

```sh
c++ -std=c++20 -fsanitize=address,undefined -fno-sanitize-recover=all \
  -Iinclude -isystem /tmp/guantune-build/include \
  test/compatibility/lemlib_settings_check.cpp -o /tmp/lemlib-settings-check
/tmp/lemlib-settings-check
```

Apple Clang may need `-DFMT_CONSTEVAL=` for the upstream bundled fmt headers;
this workaround is only for native checks. The ARM build uses upstream defaults.
