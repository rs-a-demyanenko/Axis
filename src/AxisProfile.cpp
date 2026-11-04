#include "AxisProfile.h"

#include <math.h>

namespace axis {
namespace {
bool positiveFinite(double x) { return isfinite(x) && x > 0.0; }
int8_t directionOf(double x) { return x < 0.0 ? -1 : 1; }
void rampTimes(double velocityChange, double acceleration, double jerk, double& tj, double& ta) {
    tj = fmin(acceleration / jerk, sqrt(velocityChange / jerk));
    ta = fmax(0.0, velocityChange / acceleration - tj);
}
double rampDuration(double velocityChange, double acceleration, double jerk) {
    double tj, ta;
    rampTimes(velocityChange, acceleration, jerk, tj, ta);
    return 2.0 * tj + ta;
}
double rampDistance(double velocity, double acceleration, double deceleration, double jerk) {
    return 0.5 * velocity * (rampDuration(velocity, acceleration, jerk) +
                             rampDuration(velocity, deceleration, jerk));
}
}  // namespace

bool SCurveProfile::planPosition(double distance, double speed, double acceleration,
                                double deceleration, double jerk) {
    if (!isfinite(distance) || !positiveFinite(speed) || !positiveFinite(acceleration) ||
        !positiveFinite(deceleration) || !positiveFinite(jerk)) return false;
    *this = SCurveProfile();
    _jerk = jerk;
    _distance = distance;
    _direction = directionOf(distance);
    const double length = fabs(distance);
    if (length == 0.0) return true;
    double peak = speed;
    double ramps = rampDistance(peak, acceleration, deceleration, jerk);
    if (!isfinite(ramps)) return false;
    if (ramps > length) {
        // Monotone distance equation also handles asymmetric accel/decel and
        // triangular acceleration ramps on short moves. Computed only on plan.
        double lower = 0.0, upper = speed;
        for (uint8_t i = 0; i < 40; ++i) {
            double middle = 0.5 * (lower + upper);
            if (rampDistance(middle, acceleration, deceleration, jerk) > length) upper = middle;
            else lower = middle;
        }
        peak = lower;
        ramps = rampDistance(peak, acceleration, deceleration, jerk);
    }
    if (!positiveFinite(peak)) return false;
    rampTimes(peak, acceleration, jerk, _times[0], _times[1]);
    rampTimes(peak, deceleration, jerk, _times[3], _times[4]);
    _times[2] = fmax(0.0, (length - ramps) / peak);
    for (uint8_t i = 0; i < 7; ++i) _duration += phaseDuration(i);
    return positiveFinite(_duration);
}

bool SCurveProfile::planVelocity(double velocity, double acceleration, double targetVelocity,
                                double accelerationLimit, double decelerationLimit, double jerk) {
    if (!isfinite(velocity) || !isfinite(acceleration) || !isfinite(targetVelocity) ||
        !positiveFinite(accelerationLimit) || !positiveFinite(decelerationLimit) || !positiveFinite(jerk))
        return false;
    *this = SCurveProfile();
    _velocityMode = true;
    _jerk = jerk;
    _initialVelocity = velocity;
    _initialAcceleration = acceleration;
    _endVelocity = targetVelocity;
    // First bring existing acceleration continuously to zero. An interrupting
    // stop can initially increase velocity; its stopping distance includes this.
    _times[0] = fabs(acceleration) / jerk;
    _neutralDirection = acceleration == 0.0 ? 0 : -directionOf(acceleration);
    const double neutralVelocity = velocity + 0.5 * acceleration * _times[0];
    const double change = targetVelocity - neutralVelocity;
    _direction = directionOf(change);
    double limit = fabs(targetVelocity) > fabs(neutralVelocity) ? accelerationLimit : decelerationLimit;
    if (targetVelocity * neutralVelocity < 0.0) limit = fmin(accelerationLimit, decelerationLimit);
    rampTimes(fabs(change), limit, jerk, _times[1], _times[2]);
    _times[3] = _times[1];
    for (uint8_t i = 0; i < 7; ++i) _duration += phaseDuration(i);
    if (!isfinite(_duration)) return false;
    _distance = integrate(_duration).position;
    return isfinite(_distance);
}

bool SCurveProfile::planTrackingVelocity(double velocity, double acceleration, double targetVelocity,
                                        double limit, double jerk) {
    if (!isfinite(velocity) || !isfinite(acceleration) || !isfinite(targetVelocity) ||
        !positiveFinite(limit) || !positiveFinite(jerk) || fabs(acceleration) > limit * 1.000001) return false;
    *this = SCurveProfile();
    _velocityMode = true; _jerk = jerk;
    _initialVelocity = velocity; _initialAcceleration = acceleration; _endVelocity = targetVelocity;
    double change = targetVelocity - velocity;
    double neutralChange = acceleration * fabs(acceleration) / (2.0 * jerk);
    _direction = directionOf(change - neutralChange);
    double signedChange = _direction * change;
    double peak = sqrt(fmax(0.0, jerk * signedChange + 0.5 * acceleration * acceleration));
    peak = fmin(peak, limit);
    _times[1] = fmax(0.0, (peak - _direction * acceleration) / jerk);
    _times[3] = peak / jerk;
    double rampChange = (peak * peak - 0.5 * acceleration * acceleration) / jerk;
    _times[2] = peak > 0.0 ? fmax(0.0, (signedChange - rampChange) / peak) : 0.0;
    for (uint8_t i = 0; i < 7; ++i) _duration += phaseDuration(i);
    if (!isfinite(_duration)) return false;
    _distance = integrate(_duration).position;
    return isfinite(_distance);
}

double SCurveProfile::phaseDuration(uint8_t phase) const {
    if (_velocityMode) {
        if (phase == 0) return _times[0];
        if (phase == 1) return _times[1];
        if (phase == 3) return _times[3];
        return phase == 2 ? _times[2] : 0.0;
    }
    switch (phase) {
        case 0: case 2: return _times[0];
        case 1: return _times[1];
        case 3: return _times[2];
        case 4: case 6: return _times[3];
        default: return _times[4];
    }
}

double SCurveProfile::phaseJerk(uint8_t phase) const {
    if (_velocityMode) {
        if (phase == 0) return _neutralDirection * _jerk;
        if (phase == 1) return _direction * _jerk;
        return phase == 3 ? -_direction * _jerk : 0.0;
    }
    if (phase == 0 || phase == 6) return _direction * _jerk;
    if (phase == 2 || phase == 4) return -_direction * _jerk;
    return 0.0;
}

MotionState SCurveProfile::integrate(double seconds) const {
    MotionState state;
    state.velocity = _initialVelocity;
    state.acceleration = _initialAcceleration;
    for (uint8_t i = 0; i < 7; ++i) {
        const double duration = phaseDuration(i);
        if (duration <= 0.0) continue;
        const double time = fmin(seconds, duration);
        const double jerk = phaseJerk(i);
        state.position += state.velocity * time + 0.5 * state.acceleration * time * time +
                          jerk * time * time * time / 6.0;
        state.velocity += state.acceleration * time + 0.5 * jerk * time * time;
        state.acceleration += jerk * time;
        if (seconds < duration) { state.jerk = jerk; return state; }
        seconds -= duration;
    }
    return state;
}

MotionState SCurveProfile::sample(double seconds) const {
    seconds = fmax(0.0, seconds);
    if (seconds < _duration) return integrate(seconds);
    MotionState state;
    state.position = _distance + _endVelocity * (seconds - _duration);
    state.velocity = _endVelocity;
    return state;
}

}  // namespace axis
