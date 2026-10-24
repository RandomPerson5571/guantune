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
prepare_build_test.py checks actual jsonpickle metadata shape, dependency version
recording, adapter packaging and existing-destination protection.

These tests cannot establish hardware safety or universal heuristic convergence.
The adapter must also compile against actual dependency headers with the PROS
compiler. See docs/compatibility.md and docs/hardware-validation.md. Never replace
PROS/LemLib implementations with test stubs to claim adapter compatibility.
