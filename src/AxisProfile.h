#pragma once

#include <stdint.h>

namespace axis {

struct MotionState {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double jerk = 0.0;
};

// Analytic, constant-jerk segments. No numerical integration over control ticks,
// no allocation, and no dependency on a board or control-loop frequency.
class SCurveProfile {
 public:
    bool planPosition(double distance, double speed, double acceleration, double deceleration, double jerk);
    bool planVelocity(double velocity, double acceleration, double targetVelocity,
                      double accelerationLimit, double decelerationLimit, double jerk);
    // Replanning from nonzero acceleration without first neutralizing it.
    // Used by a continuously changing, acceleration-limited external master.
    bool planTrackingVelocity(double velocity, double acceleration, double targetVelocity,
                              double accelerationLimit, double jerk);
    MotionState sample(double seconds) const;
    double duration() const { return _duration; }

 private:
    // Position: tj_acc, ta_acc, cruise, tj_dec, ta_dec.
    // Velocity: neutralize_initial_acceleration, tj, ta, unused, unused.
    double _times[5] = {};
    double _jerk = 0.0, _initialVelocity = 0.0, _initialAcceleration = 0.0;
    double _endVelocity = 0.0, _distance = 0.0, _duration = 0.0;
    int8_t _direction = 1, _neutralDirection = 0;
    bool _velocityMode = false;

    double phaseDuration(uint8_t phase) const;
    double phaseJerk(uint8_t phase) const;
    MotionState integrate(double seconds) const;
};

}  // namespace axis
