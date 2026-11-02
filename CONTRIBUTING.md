# Contributing

Guantune is a wrapper for the caller's existing LemLib chassis. Keep drivetrain
commands in `src/autotune/adapters/` and use the chassis's public API. Read
position through `chassis.getPose()`. Do not add a generic drive backend, custom
odometry, direct motor control or a replacement PID loop.

Use C++17 in library code, conventional commits and the included `.clang-format`.
Preserve existing source line endings. Put every header under `include/autotune/`
and internal declarations under `include/autotune/detail/`. Keep `src/` limited
to `.cpp` implementations. Session/search logic belongs in `core/`, motion
measurement/execution in `motion/` and in-memory recording in `telemetry/`.
Return data through `AutonTuneResult`; presentation and transport belong to the
caller. Do not add file/SD-card writers or automatic console output. See `docs/architecture.md` for ownership and cleanup rules.

Run `bash scripts/test_host.sh` with Clang/GCC and C++17/C++20. ASan/UBSan and
warnings-as-errors are enabled. Host tests belong in `test/host/`; test pure
calculations and in-memory recording without recreating upstream PROS/LemLib classes.
Packaging tests live in `test/packaging/`, real-header checks in `test/compatibility/`.

For runner, adapter or public-header changes, build the real PROS library and
template, then compile all examples and link a consumer against the template.
Never replace upstream namespaces with mocks to claim compatibility. Never install
candidate gains while motion owns the PID. Stop/restore must not throw.

Physical results belong in `docs/hardware-validation.md` with configuration,
failures, independently measured error, battery conditions and robot details.
Host tests and compile/link checks cannot establish motion safety or convergence.
