# Dependency compatibility

The pinned CI combination is **PROS kernel 4.2.2 / ARM GCC 15.2.1 / LemLib 0.5.6**.
A legacy job retains kernel 4.2.1 / ARM GCC 13.3.rel1. LemLib's
[stable depot](https://raw.githubusercontent.com/LemLib/LemLib/depot/stable.json)
requires ^4.2.1 for 0.5.6; 4.1.0 is not a compatible pin.

The weekly CI matrix also installs the latest kernel and compatible stable LemLib.
Each job compiles the public adapter, builds and installs a library/template,
links a consumer smoke program, and uploads
`project.pros` and `dependency-versions.json` as compatibility evidence. The pinned
job also uploads the template. Check the job result and resolved versions together:
a floating job passing today does not certify every version. A local ARM GNU 13.3.rel1 library/template build, template installation,
and consumer compile/link passed on 2026-10-24.
No completed remote CI run is claimed by this document.

On 2026-10-28, local PROS CLI 3.5.6 / kernel 4.2.2 / ARM GNU 15.2.Rel1
(GCC 15.2.1) / LemLib 0.5.6 passed the library build, optimized template creation,
Conductor installation into a fresh consumer, and both monolithic and default
hot/cold consumer links, using the kernel's default `gnu++26`. Clang sanitizer
host suites passed in C++17 and C++20, and the real ARM packaging regression passed.
The native settings-conversion check under Apple Clang 21 / C++20 failed inside
LemLib's bundled fmt headers with consteval diagnostics. This check has not been
validated locally for the new combination; the ARM consumer compilation passes.
Switching that native check to C++17 also fails because kernel 4.2.2 headers use
`std::is_constant_evaluated`. These limitations do not establish a CI failure.

Reproduce from the Guantune repository:

```sh
python3 scripts/prepare_build.py --kernel-version 4.2.2 --lemlib-version 0.5.6 /tmp/guantune-build
cd /tmp/guantune-build
pros make library
pros make template
cd /path/to/guantune
python3 scripts/test_template.py /tmp/guantune-consumer /tmp/guantune-build/autotune@0.1.0.zip
```

The adapter targets the public lateralPID/angularPID and five-argument PID
constructor in 0.5.6. Gains are const; assignment is invalid. IdlePidGuard saves
the original PID including its internal state, constructs a candidate before any
replacement, stops/joins owned motion, then reconstructs in place. Destruction
restores on ordinary return, abort, timeout and exception unwinding. This cannot
recover from power loss or forced task deletion. Stop/restore must not throw.

prepare_build.py generates a numeric version guard. Other resolved versions need
`GUANTUNE_ALLOW_UNTESTED_LEMLIB` defined to compile; the floating CI job sets it
explicitly to probe API compatibility. When copying source into an existing robot
project, define `GUANTUNE_LEMLIB_VERSION=506` to identify your installed version;
LemLib 0.5.6 does not expose a reliable public version macro for automatic detection.
A guard cannot certify ABI compatibility or hardware behavior.

CI compatibility ledger (resolved versions are uploaded by each matrix job):

| Kernel | LemLib | ARM toolchain | Evidence |
| --- | --- | --- | --- |
| 4.2.1 | 0.5.6 | 13.3.rel1 | Local build, install and consumer link passed 2026-10-24; remote CI pending |
| 4.2.2 | 0.5.6 | 15.2.rel1 | Local build, optimized install and monolithic/hot-cold links passed 2026-10-28; native Apple Clang settings check fails in upstream fmt; remote CI pending |
| Latest | Compatible stable | 15.2.rel1 | Weekly CI probe; inspect resolved-version artifact and job status |

Physical validation is pending; see hardware-validation.md.

## Replacement integration — 2026-10-29

The supplied replacement was integrated and tested locally with **PROS 4.2.2 /
ARM GCC 14.3.1 / LemLib 0.5.6**, using the installed desktop toolchain. This is
a different compiler from the CI pin above. The library/template build, fresh
template installation, all four example compilations, and monolithic/hot-cold
consumer links passed. Clang C++17/C++20 sanitizer suites and the native settings
conversion test (with the documented Apple Clang fmt workaround) also passed.

The published base's
[latest-stable job](https://github.com/RandomPerson5571/guantune/actions/runs/40933146371/job/123656590220)
is successful. It does not validate the current uncommitted integration. See
`pasted-implementation.md` for the behavior changes and local evidence. Physical
validation remains pending.
