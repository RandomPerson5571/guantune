# Template and robot binary size

Measured locally on 2026-10-28 with PROS CLI 3.5.6, kernel 4.2.2,
ARM GNU 15.2.Rel1 (GCC 15.2.1), default gnu++26 and LemLib 0.5.6.
These are compile/link measurements, not robot RAM or hardware tests.

## What made installation large

The original template was 830,223 bytes. Its `firmware/autotune.a` occupied
768,474 bytes, of which 739,092 bytes (96.2%) disappeared with `--strip-debug`.
PROS common.mk builds with `-g`, including debug records for upstream headers
included by guantune's translation units. Those records help debugging but are
not executable code. No PROS, LemLib or fmt headers/archives were bundled in the
guantune template; the install smoke test checks dependency headers separately.

PROS CLI also stored every ZIP member without compression. Compressing the
original template alone reduced it to 265,442 bytes. Removing library debug
sections and compressing it reduced it to 38,193 bytes (95.4% smaller). Its
uncompressed payload became 88,471 bytes, including the 29,382-byte library,
public headers, documentation and template metadata. Documentation changes can
slightly change the final ZIP size.

`pros make template` now runs `scripts/optimize_template.py` after the standard
PROS template recipe. It strips only the library copy inside the ZIP and
compresses all members. `bin/autotune.a` retains full debug information. Link
symbols and relocations are preserved; `--strip-all` must not be used.
The optimized template was installed and linked in a fresh consumer under both
monolithic and hot/cold builds. Code/data section sizes matched the original.

The separate LemLib dependency is about 18 MiB on disk, and kernel archives add
several more MiB. Creating a fresh build project installs those dependencies;
that directory's size is not guantune's incremental installation footprint.

## Robot upload cost

The following monolithic `.bin` sizes came from the same disposable consumer,
with the same dependencies installed and `USE_PACKAGE=0`, `-Os`, and
`EXTRA_CXXFLAGS=-Iinclude`. Each variant was rebuilt with `pros make all`.
All included `autotune/api.hpp` and used a global nullable chassis pointer to
retain compile/link coverage without permitting movement on accidental upload.

- Headers included, no calls: 429,356 bytes.
- LemLib `moveToPoint(0, 24, 1000)` only: 488,788 bytes.
- `tuneLateral` with telemetry enabled and ports `{1, -2}`: 512,900 bytes.
- Tuning plus JSON saving: 526,620 bytes.
- Tuning plus settings snippet: 516,988 bytes.
- Tuning plus CSV saving: 529,252 bytes.
- Tuning plus all three export helpers: 533,020 bytes.

Tuning therefore added 24,112 bytes over this LemLib movement baseline;
all export helpers together added another 20,120 bytes. This is a fixture-specific
comparison: an existing robot program may already link some shared dependencies.
Export costs overlap and must not be added independently. `output.hpp` uses
`std::ofstream`, `std::ostringstream`, locales and floating-point formatting;
calling these helpers brings in that support. Merely including the header does
not produce the same cost. Telemetry also allocates at runtime; these file sizes
do not measure its heap consumption.

For the default hot/cold mode, an empty consumer's cold binary measured
1,468,288 bytes without `firmware/autotune.a` and 1,477,648 bytes with the unused
library installed: a 9,360-byte increase. The hot binary stayed 392 bytes.
PROS's cold link uses `--whole-archive --gc-keep-exported`; unlike the monolithic
link, this retains exported library code even when the user program does not
call it. Stripping debug information does not remove this executable code.

The roughly 49 MB BSS displayed by `pros make` includes the linker script's
`0x02E00000` reserved heap in a `NOLOAD` section. It is not an upload payload or
a measurement of guantune's live allocations. Likewise, the multi-megabyte
unstripped `.elf` is a debugging artifact; compare `.bin` sizes for upload cost.

## Reproduce

Follow [compatibility.md](compatibility.md) to prepare, build and install the
template using the explicit kernel/compiler versions above. The consumer script
links all export helpers and then builds both package modes. For incremental
cost comparisons, use fresh disposable consumers with the same dependency
versions, replace only `src/main.cpp` with the variants above, and compare
`bin/monolith.bin`. For the unused hot/cold comparison, use empty competition
callbacks and compare with/without the guantune archive in that disposable
consumer. Do not delete archives from a robot project to run this experiment.

Run the packaging regression with ARM tools on PATH:

```sh
python3 -m unittest discover -s test -p template_optimization_test.py
```

It compiles an actual ARM object with debug information, optimizes a template,
checks that metadata and the original archive survive, and links the stripped
archive into a consumer executable.
