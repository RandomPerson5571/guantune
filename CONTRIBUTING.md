# Contributing

Use C++17 in library code. Keep PROS/LemLib dependencies in the adapter; host tests
implement TuningDrive, never replacement upstream namespaces. Use conventional
commits and the included .clang-format. Preserve existing source line endings.

Run `bash scripts/test_host.sh` with Clang and GCC and CXXSTD=c++17/c++20.
CI runs all four combinations under ASan and UBSan. Add regression cases for
changed decisions, stop behavior and restoration. Simulated success is not
hardware validation. Report seeds and model parameters for simulation failures.

For adapter changes, run the pinned PROS build in docs/compatibility.md and check
the latest-stable CI job. Never install candidate gains while motion owns a PID.
Cleanup callbacks must not throw. Callbacks must not start unrelated motions.

Hardware results belong in docs/hardware-validation.md with the full environment,
all failed attempts and trial battery readings. Remove identifying information
from attachments. Use the tuning-result issue template. Do not label a version
validated until its linked CI evidence and physical test evidence exist.
