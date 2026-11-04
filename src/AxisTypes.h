#pragma once

#include <stdint.h>

namespace axis {

enum class Kind : uint8_t { LINEAR, ROTARY };
enum class EncoderKind : uint8_t { LINEAR, ROTARY };
enum class Frame : uint8_t { MACHINE, WORK };
enum class Direction : uint8_t { SHORTEST, POSITIVE, NEGATIVE };
enum class Mode : uint8_t { IDLE, POSITION, VELOCITY, STOPPING, HOMING, RECOVERY, FAULT, SYNCHRONIZED };
enum class HomePhase : uint8_t {
    NONE, RELEASE, RELEASE_STOP, SEARCH, SEARCH_STOP,
    BACKOFF, BACKOFF_STOP, LATCH, LATCH_STOP, PARK
};
enum class HomeSource : uint8_t { SEPARATE, MIN_LIMIT, MAX_LIMIT };
enum class EndAction : uint8_t { HOLD, COAST };
enum class FaultAction : uint8_t { COAST, BRAKE };
enum class OutputAction : uint8_t { COAST, DRIVE, BRAKE };
enum class Result : uint8_t {
    OK, NOT_INITIALIZED, INVALID_ARGUMENT, BUSY, NOT_HOMED,
    OUT_OF_RANGE, LIMIT_ACTIVE, FAULT_ACTIVE, UNSUPPORTED
};
enum class Fault : uint8_t {
    NONE, EMERGENCY_STOP, MIN_LIMIT, MAX_LIMIT, BOTH_LIMITS,
    SOFT_LIMIT, ENCODER_INVALID, ENCODER_RATE, CONTROL_TIMEOUT,
    FOLLOWING_ERROR, STALL, HOMING_TIMEOUT, HOMING_TRAVEL, NUMERIC_ERROR, SYNC_FAULT
};
enum class Completion : uint8_t { NONE, RUNNING, REACHED, STOPPED, CANCELLED, FAULTED };

struct EncoderScale {
    EncoderKind kind = EncoderKind::LINEAR;
    double countsPerUnit = 100.0;
    bool inverted = false;

    // Counts are AFTER quadrature decoding, not the encoder's nominal PPR.
    static EncoderScale linear(double countsPerMillimeter) {
        EncoderScale value;
        value.countsPerUnit = countsPerMillimeter;
        return value;
    }
    // unitsPerRevolution: mm/rev for a screw, degrees/rev for a rotary axis.
    // For a motor encoder behind a reduction G: 360/G degrees per motor rev.
    static EncoderScale rotary(double countsPerRevolution, double unitsPerRevolution = 360.0) {
        EncoderScale value;
        value.kind = EncoderKind::ROTARY;
        value.countsPerUnit = countsPerRevolution > 0.0 && unitsPerRevolution > 0.0 ?
                             countsPerRevolution / unitsPerRevolution : 0.0;
        return value;
    }
};

struct HomeConfig {
    HomeSource source = HomeSource::SEPARATE;
    int8_t direction = -1;
    double searchSpeed = 2.0;
    double latchSpeed = 0.5;
    double backoff = 1.0;
    double switchPosition = -1.0;
    double parkPosition = 0.0;
    uint32_t debounceUs = 2000;
    uint32_t timeoutUs = 30000000;
    double maxTravel = 1000.0;
};

struct Config {
    Kind kind = Kind::LINEAR;
    bool continuous = false;           // Only valid for a rotary axis.
    double rotaryPeriod = 360.0;
    EncoderScale encoder;
    double maxSpeed = 20.0;
    double acceleration = 100.0;
    double deceleration = 100.0;
    double maxJerk = 1000.0;             // unit/s^3; zero selects legacy trapezoid.
    double positionKp = 8.0;
    double velocityKp = 0.08;           // Normalized duty / (unit/s).
    double velocityKi = 0.4;
    double velocityKd = 0.0;            // Derivative on measured velocity.
    double velocitySetpointWeight = 0.8; // 2-DOF proportional setpoint weight beta.
    double velocityDerivativeWeight = 0.0; // Derivative setpoint weight gamma.
    double velocityFeedforward = 0.0;   // Normalized duty / (unit/s).
    double velocityFilterTime = 0.02;   // Seconds; zero disables filtering.
    double derivativeFilterTime = 0.02;
    double maxOutput = 0.8;             // Normalized duty, (0, 1].
    double positionTolerance = 0.05;
    double velocityTolerance = 0.1;
    double positionHysteresis = 0.05;
    uint32_t settleTimeUs = 100000;
    uint32_t controlPeriodUs = 5000;
    uint32_t maxTickGapUs = 50000;
    bool softLimitsEnabled = true;
    double softMin = 0.0;
    double softMax = 100.0;
    uint32_t limitReleaseUs = 2000;     // Assert immediately, debounce release.
    bool allowUnhomedRelative = true;
    double unhomedMaxDistance = 10.0;
    double unhomedMaxSpeed = 2.0;
    double recoveryMaxDistance = 10.0;
    double recoveryMaxSpeed = 1.0;
    double maxFollowingError = 5.0;     // Zero disables this diagnostic.
    uint32_t followingErrorTimeUs = 100000;
    double stallOutput = 0.6;           // Zero disables this diagnostic.
    double stallVelocity = 0.02;
    uint32_t stallTimeUs = 500000;
    double maxEncoderRate = 0.0;        // Counts/s; zero disables plausibility check.
    EndAction endAction = EndAction::HOLD;
    FaultAction faultAction = FaultAction::COAST;
    HomeConfig home;
};

// Logical active states; pin polarity belongs to the hardware adapter.
struct Inputs {
    bool minLimit = false;
    bool maxLimit = false;
    bool home = false;
    bool emergency = false;
    bool encoderValid = true;
};

struct Output {
    double duty = 0.0;
    OutputAction action = OutputAction::COAST;
};

struct Snapshot {
    Mode mode;
    HomePhase homePhase;
    Fault fault;
    Completion completion;
    bool homed;
    bool inPosition;
    int64_t counts;
    double position;
    double workPosition;
    double velocity;
    double target;
    double referencePosition;
    double referenceVelocity;
    double followingError;
    double velocityCommand;
    double integral;
    bool saturated;
    Output output;
    Inputs inputs;
    int64_t absolutePositionCounts;
    int64_t relativePositionCounts;
    double referenceAcceleration;
    double referenceJerk;
};

const char* toString(Result value);
const char* toString(Fault value);
const char* toString(Mode value);
const char* toString(Completion value);

}  // namespace axis

