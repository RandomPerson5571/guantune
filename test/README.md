# Validation

Run `bash scripts/test_host.sh` from the Guantune checkout. Set CXX=g++ or clang++
and CXXSTD=c++17 or c++20. ASan and UBSan are enabled; warnings are errors.
CI runs all four compiler/standard combinations.

self_tuning_auton_test.cpp covers scripted decision/return/fault/abort/time-wrap
cases. quality_test.cpp uses an independent first-order motor model with friction,
output saturation, brake response and measurement noise. Twelve fixed random seeds
must converge and confirm 6/24/72-inch targets. A separate run tests 90/180-degree
turns. Tests also cover exception restoration of const PIDs, failed preflight,
independent measurement, bounded policies, best-trial fallback and persistence.
Return regressions cover invalid independent readings in both modes, disagreement
before reversing, frozen odometry, signed origin-relative measurements and angular
x/y jumps. Target-list overrides, float parameter bounds and schema 2 windup
exports are checked too.
Telemetry regressions cover disabled overhead, phases and sampling intervals,
signed velocity, angular/lateral returns, time wrap, bounded recording, command
probes, invalid readings, failed-session CSV export and cleanup after callbacks throw.
prepare_build_test.py checks actual jsonpickle metadata shape, dependency version
recording, adapter packaging and existing-destination protection.

These tests cannot establish hardware safety or universal heuristic convergence.
The adapter must also compile against actual dependency headers with the PROS
compiler. See docs/compatibility.md and docs/hardware-validation.md. Never replace
PROS/LemLib implementations with test stubs to claim adapter compatibility.

`lemlib_settings_check.cpp` tests settings conversion against real dependency
headers without creating hardware objects. It runs in the PROS CI job after
dependency installation. From this checkout, with a prepared PROS build project:

```sh
c++ -std=c++20 -fsanitize=address,undefined -fno-sanitize-recover=all \
  -Iinclude -isystem /tmp/guantune-build/include \
  test/lemlib_settings_check.cpp -o /tmp/lemlib-settings-check
/tmp/lemlib-settings-check
```

Apple Clang may additionally need `-DFMT_CONSTEVAL=` for the bundled upstream
fmt headers. This only disables upstream format-string compile-time checking in
the host conversion test; the normal ARM adapter build retains its standard flags.
