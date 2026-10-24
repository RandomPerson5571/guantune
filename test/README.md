# Host-side tests

`self_tuning_auton_test.cpp` exercises the implemented autonomous tuner using a
scripted drive and a simple simulated plant. See the main README for the command.
Checks include successful confirmation, P/D step selection, fine refinement,
oscillation, optional integral gating, faults, aborts, and clock wraparound.
It requires only C++17, not PROS or LemLib.

## Future relay tests

The separate relay core is intentionally unimplemented.
Host tests should include only autotune/relay_tuner.hpp or autotune/types.hpp;
they must not need PROS or LemLib. Keep robot builds on the PROS toolchain.

When implementing the core, cover hysteresis under sensor noise, elapsed-time
handling, known simulated oscillations, settling-cycle exclusion, period/amplitude
variance rejection, timeout and invalid-input status, and each rule's gain units.
Validate failure results contain no gains. Hardware checks must separately verify
motor clamps, abort response, sensor failures, task lifetime, and stopping on every
exit path. Do not treat simulated tuning as robot validation.
