#pragma once

#include "AxisTypes.h"

namespace axis {

// Hardware-independent control core. Call from one task, never from an ISR.
// nowUs and encoderCount are modulo-2^32 counters; all inputs are one snapshot.
class Core {
 public:
    Result begin(Config config, uint32_t nowUs, uint32_t encoderCount,
                 const Inputs& inputs = Inputs());
    static bool validConfig(const Config& config);
    bool tick(uint32_t nowUs, uint32_t encoderCount, const Inputs& inputs = Inputs());

    Result moveTo(double position, double speed = 0.0, Frame frame = Frame::MACHINE);
    Result moveBy(double distance, double speed = 0.0);
    Result moveToAngle(double angle, Direction direction = Direction::SHORTEST, double speed = 0.0);
    Result jog(double velocity);
    Result stop();
    Result home();
    Result recoverFromLimit(int8_t direction, double distance, double speed = 0.0);
    Result clearFault();
    void emergencyStop();
    void disable();
    void brake();
    void invalidateReference();
    Result setWorkPosition(double position);
    // Work/relative frame: position = absolute position - relative origin.
    // A relative reading can be zeroed before homing; an absolute origin needs Home.
    Result setRelativePosition(double position = 0.0);
    Result setRelativeOrigin(double absolutePosition);
    Result setRelativeOriginCounts(int64_t absoluteCounts);
    Result setMaxSpeed(double speed);
    Result setAcceleration(double acceleration);
    Result setDeceleration(double deceleration);

    bool isInitialized() const { return _initialized; }
    bool isHomed() const { return _homed; }
    bool isBusy() const;
    bool inPosition() const { return _inPosition; }
    double getPosition() const { return _position; }
    double getWorkPosition() const { return _position - _workOffset; }
    // Physical units: millimeters on linear axes, degrees on rotary axes.
    // Before isHomed(), the absolute frame is provisional (relative to begin()).
    double getAbsolutePosition() const { return getPosition(); }
    double getRelativePosition() const { return getWorkPosition(); }
    // Signed encoder counts in these coordinate frames, not raw getCounts().
    int64_t getAbsolutePositionCounts() const;
    int64_t getRelativePositionCounts() const;
    double getVelocity() const { return _velocity; }
    double getTarget() const { return _target; }
    double getAngle() const;
    int64_t getCounts() const { return _counts; }
    Fault getFault() const { return _fault; }
    Mode getMode() const { return _mode; }
    HomePhase getHomePhase() const { return _homePhase; }
    Completion getCompletion() const { return _completion; }
    const Output& getOutput() const { return _output; }
    const Config& getConfig() const { return _config; }
    Snapshot snapshot() const;

 private:
    struct Debounce {
        bool raw = false;
        bool stable = false;
        uint32_t changedAt = 0;
        int64_t edgeCount = 0;
        void begin(bool state, uint32_t now, int64_t count);
        void update(bool state, uint32_t now, uint32_t duration, int64_t count);
    };

    Config _config;
    Inputs _inputs;
    Output _output;
    Debounce _homeInput, _minInput, _maxInput;
    bool _initialized = false, _homed = false, _referenced = false;
    bool _holding = false, _inPosition = false, _saturated = false;
    Mode _mode = Mode::IDLE;
    HomePhase _homePhase = HomePhase::NONE;
    Fault _fault = Fault::NONE;
    Completion _completion = Completion::NONE;
    uint32_t _lastRaw = 0, _lastSampleUs = 0, _nowUs = 0;
    uint32_t _settleUs = 0, _followingUs = 0, _stallUs = 0, _homeStartedUs = 0;
    int64_t _counts = 0, _originCounts = 0, _sampleCounts = 0;
    int64_t _workOffsetCounts = 0;
    double _originPosition = 0.0, _position = 0.0, _samplePosition = 0.0;
    double _workOffset = 0.0, _velocity = 0.0, _previousVelocity = 0.0;
    double _derivative = 0.0, _integral = 0.0, _velocityCommand = 0.0;
    double _target = 0.0, _referencePosition = 0.0, _referenceVelocity = 0.0;
    double _jogVelocity = 0.0, _moveSpeed = 0.0;
    double _profileStart = 0.0, _profileDistance = 0.0, _profileSign = 1.0;
    double _profilePeak = 0.0, _accelTime = 0.0, _cruiseTime = 0.0;
    double _decelTime = 0.0, _profileTime = 0.0;
    double _homeTravel = 0.0, _phasePosition = 0.0;
    int8_t _recoveryDirection = 0;

    Result ready() const;
    Result checkMove(double position, double speed) const;
    double signedScale() const;
    bool homeRaw(const Inputs& inputs) const;
    bool minActive() const { return _inputs.minLimit || _minInput.stable; }
    bool maxActive() const { return _inputs.maxLimit || _maxInput.stable; }
    bool softActive() const;
    bool sharedHomeException() const;
    void updatePosition();
    void resetController();
    void trip(Fault fault, bool loseReference = false);
    void idle(Completion completion, bool hold);
    void startProfile(double position, double speed);
    bool advanceProfile(double dt);
    double approachVelocity(double desired, double dt);
    double safeVelocity(double velocity) const;
    void homeStep(double dt);
    void control(double desiredVelocity, double dt);
    void checkSettled(uint32_t elapsedUs, bool profileDone);
    bool checkHardLimits();
    void guardOutput();
};

}  // namespace axis
