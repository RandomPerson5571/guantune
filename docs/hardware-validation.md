# Hardware validation log

No physical sessions have been recorded. Host and physics tests do not validate
robot safety, traction, repeatability or the quality of gains on a real chassis.
Do not substitute generated or simulated measurements for this log.

Copy this record for each session; retain failures. Run at least three sessions
with the same robot and targets, across battery conditions, before describing
results as repeatable. Compare gains and independently measured final error.

- Date, operator/team, Guantune commit:
- PROS kernel and LemLib versions, linked CI build:
- Drive motors, cartridge, gearing, track width, payload:
- Wheel type/diameter and odometry configuration:
- Surface and available forward/backward or rotational clearance:
- Brake mode and calibration/readiness method:
- Battery charge, initial voltage and temperature:
- Configuration, distances/angles and independent sensor method:
- Per trial: target, P/I/D, measured/independent
  displacement, error, overshoot, crossings, elapsed time, settlement and return:
- Final status, accepted gains (only COMPLETE), actual final position:
- Failures, manual interventions and differences from previous sessions:
- Attached raw reports, video and independent tape-measure checks:

Use the repository's tuning-result issue form to submit a record. A maintainable
log requires actual robot evidence; this checkout provides the protocol only.
