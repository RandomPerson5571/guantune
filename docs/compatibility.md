# Dependency compatibility

The pinned CI combination is **PROS kernel 4.2.1 / LemLib 0.5.6**. LemLib's
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

Reproduce from the Guantune repository:

```sh
python3 scripts/prepare_build.py --kernel-version 4.2.1 --lemlib-version 0.5.6 /tmp/guantune-build
cd /tmp/guantune-build
pros make library
pros make template
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
| Latest | Compatible stable | 15.2.rel1 | Weekly CI probe; inspect resolved-version artifact and job status |

Physical validation for both combinations is pending; see hardware-validation.md.
