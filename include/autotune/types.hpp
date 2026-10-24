#pragma once

#include <cstdint>
#include <optional>

namespace autotune {

enum class Mode { LATERAL, ANGULAR };
enum class Rule { TYREUS_LUYBEN, ZIEGLER_NICHOLS };
enum class Status {
    NOT_IMPLEMENTED,
    RUNNING,
    COMPLETE,
    ABORTED,
    TIMED_OUT,
    INVALID_CONFIG,
    SENSOR_FAULT,
    TRAVEL_LIMIT,
    UNSTABLE_OSCILLATION
};

struct Gains {
    double kP;
    double kI;
    double kD;
};

// Lateral measurements use inches; angular measurements use continuous degrees.
// Time is elapsed milliseconds from the beginning of the experiment.
struct Sample {
    std::uint32_t timeMs;
    double setpoint;
    double measurement;
};

struct Step {
    double output = 0; // Signed motor command, -127 to 127.
    Status status = Status::NOT_IMPLEMENTED;
    bool done = true;
};

struct Config {
    Mode mode = Mode::ANGULAR;
    Rule rule = Rule::TYREUS_LUYBEN;
    double testMagnitude = 90; // Relative to the starting pose, in mode units.
    double maxOutput = 60;     // Relay amplitude and absolute output clamp, 0-127.
    double hysteresis = 1;     // Half-width of the relay dead band, in mode units.
    double maxTravel = 180;    // Maximum displacement from the start, in mode units.
    std::uint32_t timeoutMs = 15000;
    std::uint32_t samplePeriodMs = 10;
    unsigned settlingCycles = 2;
    unsigned measuredCycles = 6;
    double maxRelativeStdDev = 0.15; // Planned amplitude/period stability threshold.
};

struct Result {
    Status status = Status::NOT_IMPLEMENTED;
    Mode mode = Mode::ANGULAR;
    // Empty until a completed, validated experiment produces real estimates.
    std::optional<Gains> gains;
    std::optional<double> Ku;
    std::optional<double> TuSeconds;
    unsigned cycles = 0;

    // Declaration only: terminal output belongs to the output layer.
    void print() const;
};

// Planned streaming log record; an SD/terminal sink can consume these later.
struct LogRecord {
    Sample sample;
    double output;
    Status status;
};

} // namespace autotune
