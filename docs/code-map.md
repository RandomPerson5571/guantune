# Code structure and flow

These Mermaid diagrams show the chassis-based tuner. Arrows in the structure
diagram mean “uses.” All public and internal headers live in `include/autotune/`;
implementations live in `src/autotune/`.

## Code structure

```mermaid
flowchart TD
    App["Robot application / examples"] --> API["api.hpp<br/>tuneLateral / tuneAngular / SelfTuningAuton"]
    API --> Core["core/self_tuning_auton.cpp<br/>Session sequencing + confirmations"]
    Core --> Config["core/tuning_config.cpp<br/>Config validation"]
    Core --> Search["core/gain_search.cpp<br/>Choose next candidate gains"]
    Core --> Session["core/session_control.cpp<br/>Readiness, cancellation, idle checks"]
    Core --> Motion["motion/tuning_motion.cpp<br/>Run and measure a trial"]
    Core --> Return["motion/tuning_return.cpp<br/>Return to saved origin"]
    Motion --> Tracker["motion/motion_tracking.cpp<br/>Displacement, speed, limits"]
    Return --> Tracker
    Motion --> Adapter["adapters/lemlib_tuning_drive.cpp<br/>Chassis wrapper + PID guard"]
    Return --> Adapter
    Session --> Adapter
    Adapter --> LemLib["Caller-owned LemLib chassis<br/>Odometry, motion PID, drivetrain"]
    Core --> Telemetry["telemetry/telemetry.cpp<br/>Optional in-memory samples"]
    Core --> Result["tuning_result.hpp<br/>Status, gains, trials, telemetry"]
```

`tuning_config.hpp` and `types.hpp` define session settings and shared values.
The `detail/` headers declare internal helpers. `idle_pid_guard.hpp` handles
temporary PID replacement and restoration. PROS supplies timing, competition
state, and optional motor/battery readings through the adapter. The caller owns
the chassis and decides how to display or save results.

## Session flow

```mermaid
flowchart TD
    Start["tuneLateral / tuneAngular<br/>or SelfTuningAuton.run"] --> Guard{"Session already running?"}
    Guard -- Yes --> Busy["Return ALREADY_RUNNING<br/>Leave active session alone"]
    Guard -- No --> Checks["Validate config and cancellation<br/>Reject unrelated active motion<br/>Check readiness and optional confirmation"]
    Checks --> Ready{"Checks pass?"}
    Ready -- No --> Exit["Return failure status"]
    Ready -- Yes --> Candidate["Start with configured gains + targets"]
    Candidate --> Before["Check abort / beforeTrial callback<br/>Wait for idle; save origin on first trial"]
    Before --> Trial["Install candidate PID<br/>Request LemLib move or turn"]
    Trial --> Measure["Sample getPose through motion + coast<br/>Check settling, travel, sensors, cancellation"]
    Measure --> Usable{"Usable trial report?"}
    Usable -- No --> Cleanup["Stop owned commands<br/>Restore original PID"]
    Usable -- Yes --> Return["Restore PID<br/>Return to origin with low-power tank commands"]
    Return --> Returned{"Return succeeded?"}
    Returned -- No --> Cleanup
    Returned -- Yes --> Passed{"Trial meets acceptance limits?"}
    Passed -- Yes --> Confirm{"All target confirmations complete?"}
    Confirm -- Yes --> Complete["Set COMPLETE + accepted gains"]
    Complete --> Cleanup
    Confirm -- No --> Budget
    Passed -- No --> Reset["Reset confirmations and target index"]
    Reset --> Budget{"Trials remain?"}
    Budget -- No --> Limit["Set TRIAL_LIMIT"]
    Limit --> Cleanup
    Budget -- Yes --> Next["Keep passing candidate<br/>or ask GainSearch for next gains"]
    Next --> Before
    Before -. Abort / error .-> Cleanup
    Next -. Gain / retry limit .-> Cleanup
    Cleanup --> Result["Return status, gains, trials<br/>and optional telemetry"]
```

For early failures, cleanup only touches commands and PID state owned by this
session; it does not cancel unrelated motion rejected at entry. A failed session
returns diagnostic gains, not accepted gains. Only `COMPLETE` means every
required confirmation and the final return succeeded. The original PID is
restored on exit; the caller applies successful gains deliberately.

The flow runs synchronously on the calling task. LemLib owns motion execution
and odometry; Guantune measures and adjusts gains between trials. See
[getting started](getting-started.md), [architecture](architecture.md), and
[configuration](tuning.md) for details.
