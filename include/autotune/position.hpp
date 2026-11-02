#pragma once

namespace autotune {
// A snapshot of chassis.getPose(): inches and clockwise degrees, zero along +y.
// This stores a LemLib reading; Guantune does not own or update odometry.
struct ChassisPosition {
    double x;
    double y;
    double headingDegrees;
};
} // namespace autotune
