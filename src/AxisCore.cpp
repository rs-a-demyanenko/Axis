#include "AxisCore.h"

#include <limits.h>
#include <math.h>
#include <new>

namespace axis {
namespace {
double clampValue(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
double signOf(double x) { return x < 0.0 ? -1.0 : 1.0; }
bool finiteValue(double x) { return isfinite(x); }
bool positive(double x) { return finiteValue(x) && x > 0.0; }
bool nonnegative(double x) { return finiteValue(x) && x >= 0.0; }
bool duration(uint32_t x) { return x > 0 && x < 0x80000000UL; }
// Counts remain integer even where floating point no longer resolves one pulse.
bool countsFit(double value) {
    return finiteValue(value) && value >= -9223372036854775808.0 && value < 9223372036854775808.0;
}
int64_t roundedCounts(double value) {
    if (isnan(value)) return 0;
    if (value >= 9223372036854775808.0) return INT64_MAX;
    if (value <= -9223372036854775808.0) return INT64_MIN;
    value = round(value);
    if (value >= 9223372036854775808.0) return INT64_MAX;
    return static_cast<int64_t>(value);
}
bool subtractionFits(int64_t a, int64_t b) {
    return !((b > 0 && a < INT64_MIN + b) || (b < 0 && a > INT64_MAX + b));
}
int64_t subtractCounts(int64_t a, int64_t b) {
    if (!subtractionFits(a, b)) return b > 0 ? INT64_MIN : INT64_MAX;
    return a - b;
}
int64_t addCounts(int64_t a, int64_t b) {
    if (b > 0 && a > INT64_MAX - b) return INT64_MAX;
    if (b < 0 && a < INT64_MIN - b) return INT64_MIN;
    return a + b;
}
uint32_t accumulate(uint32_t value, uint32_t step, uint32_t limit) {
    return value >= limit || step >= limit - value ? limit : value + step;
}
double wrapped(double value, double period) {
    value = fmod(value, period);
    return value < 0.0 ? value + period : value;
}
}  // namespace

void Core::Debounce::begin(bool state, uint32_t now, int64_t count) {
    raw = stable = state;
    changedAt = now;
    edgeCount = count;
}

void Core::Debounce::update(bool state, uint32_t now, uint32_t wait, int64_t count) {
    if (raw != state) {
        raw = state;
        changedAt = now;
        edgeCount = count;
    }
    if (static_cast<uint32_t>(now - changedAt) >= wait) stable = raw;
}

bool Core::validConfig(const Config& c) {
    if (c.control != Control::DC && c.control != Control::STEPPER) return false;
    if (c.kind != Kind::LINEAR && c.kind != Kind::ROTARY) return false;
    if (c.encoder.kind != EncoderKind::LINEAR && c.encoder.kind != EncoderKind::ROTARY) return false;
    if (c.continuous && c.kind != Kind::ROTARY) return false;
    if (!positive(c.rotaryPeriod) || !positive(c.encoder.countsPerUnit)) return false;
    if (!positive(c.maxSpeed) || !positive(c.acceleration) || !positive(c.deceleration)) return false;
    if (!nonnegative(c.maxJerk) || !nonnegative(c.velocitySetpointWeight) || c.velocitySetpointWeight > 1.0 ||
        !nonnegative(c.velocityDerivativeWeight) || c.velocityDerivativeWeight > 1.0) return false;
    if (!positive(c.positionKp) || !nonnegative(c.velocityKp) || !nonnegative(c.velocityKi) ||
        !nonnegative(c.velocityKd) || !nonnegative(c.velocityFeedforward)) return false;
    if (c.control == Control::DC && c.velocityKp == 0.0 && c.velocityKi == 0.0 &&
        c.velocityFeedforward == 0.0) return false;
    if (!nonnegative(c.velocityFilterTime) || !nonnegative(c.derivativeFilterTime)) return false;
    if (!positive(c.maxOutput) || c.maxOutput > 1.0) return false;
    if (!positive(c.positionTolerance) || c.positionTolerance * c.encoder.countsPerUnit < 1.0 ||
        !positive(c.velocityTolerance) || !nonnegative(c.positionHysteresis)) return false;
    if (!duration(c.controlPeriodUs) || !duration(c.maxTickGapUs) || c.maxTickGapUs < c.controlPeriodUs ||
        !duration(c.settleTimeUs) || c.limitReleaseUs >= 0x80000000UL) return false;
    if (!finiteValue(c.softMin) || !finiteValue(c.softMax) || c.softMin >= c.softMax) return false;
    if (!positive(c.unhomedMaxDistance) || !positive(c.unhomedMaxSpeed) ||
        !positive(c.recoveryMaxDistance) || !positive(c.recoveryMaxSpeed)) return false;
    if (!nonnegative(c.maxFollowingError) || !duration(c.followingErrorTimeUs) ||
        !nonnegative(c.stallOutput) || c.stallOutput > c.maxOutput || !nonnegative(c.stallVelocity) ||
        !duration(c.stallTimeUs) || !nonnegative(c.maxEncoderRate)) return false;
    if (c.endAction != EndAction::HOLD && c.endAction != EndAction::COAST) return false;
    if (c.faultAction != FaultAction::COAST && c.faultAction != FaultAction::BRAKE) return false;
    const HomeConfig& h = c.home;
    if (h.source != HomeSource::SEPARATE && h.source != HomeSource::MIN_LIMIT && h.source != HomeSource::MAX_LIMIT)
        return false;
    if (h.direction != -1 && h.direction != 1) return false;
    if ((h.source == HomeSource::MIN_LIMIT && h.direction != -1) ||
        (h.source == HomeSource::MAX_LIMIT && h.direction != 1)) return false;
    if (!positive(h.searchSpeed) || !positive(h.latchSpeed) || h.searchSpeed > c.maxSpeed ||
        h.latchSpeed > h.searchSpeed || !positive(h.backoff) || !positive(h.maxTravel) ||
        !finiteValue(h.switchPosition) || !finiteValue(h.parkPosition) ||
        !duration(h.timeoutUs) || h.debounceUs >= 0x80000000UL) return false;
    if (c.softLimitsEnabled && (h.parkPosition < c.softMin || h.parkPosition > c.softMax)) return false;
    if (h.source != HomeSource::SEPARATE && (h.parkPosition - h.switchPosition) * h.direction >= 0.0)
        return false;  // Shared switch must be left in the direction away from the limit.
    return true;
}

Result Core::begin(Config config, uint32_t now, uint32_t raw, const Inputs& inputs) {
    if (isBusy()) return Result::BUSY;
    if (!validConfig(config)) return Result::INVALID_ARGUMENT;
    // Reconstruct in the existing storage: unlike assignment from Core(), this
    // does not require a second, full-size controller on the small AVR stack.
    // Placement construction allocates no memory. Config is already a value copy.
    ::new (static_cast<void*>(this)) Core();
    _config = config;
    _inputs = inputs;
    _lastRaw = raw;
    _nowUs = _lastSampleUs = now;
    _homeInput.begin(homeRaw(inputs), now, 0);
    _minInput.begin(inputs.minLimit, now, 0);
    _maxInput.begin(inputs.maxLimit, now, 0);
    _initialized = true;
    if (inputs.emergency) trip(Fault::EMERGENCY_STOP, true);
    else if (!inputs.encoderValid) trip(Fault::ENCODER_INVALID, true);
    else if (minActive() && maxActive()) trip(Fault::BOTH_LIMITS);
    return _fault == Fault::NONE ? Result::OK : Result::FAULT_ACTIVE;
}

bool Core::isBusy() const {
    return _mode != Mode::IDLE && _mode != Mode::FAULT;
}

double Core::signedScale() const {
    return (_config.encoder.inverted ? -1.0 : 1.0) / _config.encoder.countsPerUnit;
}

void Core::updatePosition() {
    _position = _originPosition + static_cast<double>(_counts - _originCounts) * signedScale();
}

int64_t Core::getAbsolutePositionCounts() const {
    int64_t delta = subtractCounts(_counts, _originCounts);
    if (_config.encoder.inverted) delta = subtractCounts(0, delta);
    return addCounts(delta, roundedCounts(_originPosition * _config.encoder.countsPerUnit));
}

int64_t Core::getRelativePositionCounts() const {
    return subtractCounts(getAbsolutePositionCounts(), _workOffsetCounts);
}

bool Core::homeRaw(const Inputs& inputs) const {
    if (_config.home.source == HomeSource::MIN_LIMIT) return inputs.minLimit;
    if (_config.home.source == HomeSource::MAX_LIMIT) return inputs.maxLimit;
    return inputs.home;
}

bool Core::softActive() const {
    return _config.softLimitsEnabled && _referenced && _mode != Mode::RECOVERY &&
           _mode != Mode::HOMING;
}

bool Core::sharedHomeException() const {
    return _mode == Mode::HOMING && _config.home.source != HomeSource::SEPARATE;
}

Result Core::ready() const {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (_fault != Fault::NONE) return Result::FAULT_ACTIVE;
    if (isBusy() || fabs(_velocity) > _config.velocityTolerance) return Result::BUSY;
    if (minActive() || maxActive()) return Result::LIMIT_ACTIVE;
    return Result::OK;
}

Result Core::checkMove(double position, double speed) const {
    if (!finiteValue(position) || !nonnegative(speed) || speed > _config.maxSpeed) return Result::INVALID_ARGUMENT;
    if (_homed && _config.softLimitsEnabled && (position < _config.softMin || position > _config.softMax))
        return Result::OUT_OF_RANGE;
    return Result::OK;
}

void Core::resetController() {
    _integral = _derivative = _velocityCommand = 0.0;
    _previousVelocity = _velocity;
    _previousVelocityCommand = 0.0;
    _referenceAcceleration = _referenceJerk = 0.0;
    _rampActive = _jogStopping = false;
    _followingUs = _stallUs = _settleUs = 0;
    _saturated = _inPosition = false;
}

bool Core::startProfile(double position, double speed) {
    _target = position;
    _profileStart = _referencePosition = _position;
    _profileDistance = fabs(position - _position);
    _profileSign = signOf(position - _position);
    _moveSpeed = speed;
    _profileTime = _referenceVelocity = 0.0;
    _holding = false;
    _completion = Completion::RUNNING;
    resetController();
    if (_config.maxJerk > 0.0) {
        if (!_smoothProfile.planPosition(position - _position, speed, _config.acceleration,
                                         _config.deceleration, _config.maxJerk)) {
            trip(Fault::NUMERIC_ERROR, true);
            return false;
        }
    } else {
        const double a = _config.acceleration, d = _config.deceleration;
        _profilePeak = fmin(speed, sqrt(2.0 * _profileDistance / (1.0 / a + 1.0 / d)));
        _accelTime = _profilePeak / a;
        _decelTime = _profilePeak / d;
        double rampDistance = 0.5 * _profilePeak * (_accelTime + _decelTime);
        _cruiseTime = _profilePeak > 0.0 ? fmax(0.0, (_profileDistance - rampDistance) / _profilePeak) : 0.0;
    }
    return true;
}

Result Core::moveTo(double position, double speed, Frame frame) {
    Result result = ready();
    if (result != Result::OK) return result;
    if (!_homed) return Result::NOT_HOMED;
    if (frame != Frame::MACHINE && frame != Frame::WORK) return Result::INVALID_ARGUMENT;
    if (frame == Frame::WORK) position += _workOffset;
    result = checkMove(position, speed);
    if (result != Result::OK) return result;
    if (!startProfile(position, speed == 0.0 ? _config.maxSpeed : speed)) return Result::FAULT_ACTIVE;
    _mode = Mode::POSITION;
    return Result::OK;
}

Result Core::moveBy(double distance, double speed) {
    Result result = ready();
    if (result != Result::OK) return result;
    if (!finiteValue(distance)) return Result::INVALID_ARGUMENT;
    if (!_homed && !_config.allowUnhomedRelative) return Result::NOT_HOMED;
    if (!_homed && fabs(distance) > _config.unhomedMaxDistance) return Result::OUT_OF_RANGE;
    result = checkMove(_position + distance, speed);
    if (result != Result::OK) return result;
    speed = speed == 0.0 ? _config.maxSpeed : speed;
    if (!_homed) speed = fmin(speed, _config.unhomedMaxSpeed);
    if (!startProfile(_position + distance, speed)) return Result::FAULT_ACTIVE;
    _mode = Mode::POSITION;
    return Result::OK;
}

double Core::getAngle() const { return wrapped(_position, _config.rotaryPeriod); }

Result Core::moveToAngle(double angle, Direction direction, double speed) {
    if (_config.kind != Kind::ROTARY || !_config.continuous) return Result::UNSUPPORTED;
    if (!finiteValue(angle)) return Result::INVALID_ARGUMENT;
    if (direction != Direction::SHORTEST && direction != Direction::POSITIVE && direction != Direction::NEGATIVE)
        return Result::INVALID_ARGUMENT;
    const double period = _config.rotaryPeriod;
    double delta = wrapped(wrapped(angle, period) - getAngle(), period);
    if (direction == Direction::NEGATIVE && delta > 0.0) delta -= period;
    if (direction == Direction::SHORTEST && delta > period * 0.5) delta -= period;
    return moveTo(_position + delta, speed);  // A half-turn tie goes positive.
}

Result Core::jog(double velocity) {
    Result result = ready();
    if (result != Result::OK) return result;
    if (!finiteValue(velocity) || fabs(velocity) > _config.maxSpeed) return Result::INVALID_ARGUMENT;
    // An unbounded command before homing cannot enforce an allowed travel distance.
    if (!_homed) return Result::NOT_HOMED;
    if (velocity == 0.0) return Result::OK;
    if (_config.softLimitsEnabled && ((_position <= _config.softMin && velocity < 0.0) ||
                                    (_position >= _config.softMax && velocity > 0.0))) return Result::OUT_OF_RANGE;
    resetController();
    _holding = false;
    _referencePosition = _position;
    _referenceVelocity = 0.0;
    _jogVelocity = velocity;
    _completion = Completion::RUNNING;
    _mode = Mode::VELOCITY;
    return Result::OK;
}

Result Core::stop() {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (_fault != Fault::NONE) return Result::FAULT_ACTIVE;
    if (_mode == Mode::SYNCHRONIZED) return Result::BUSY;  // Stop via its coordinator.
    if (_mode == Mode::STOPPING || _mode == Mode::IDLE) return Result::OK;
    if (_mode != Mode::RECOVERY) _recoveryDirection = 0;
    if (_mode == Mode::HOMING) {
        if (_config.home.source == HomeSource::MIN_LIMIT) _recoveryDirection = 1;
        if (_config.home.source == HomeSource::MAX_LIMIT) _recoveryDirection = -1;
        _homed = _referenced = false;
        _homePhase = HomePhase::NONE;
    }
    _mode = Mode::STOPPING;
    if (_config.maxJerk == 0.0) {
        _referencePosition = _position;
        _referenceVelocity = _velocity;
    }
    _rampActive = false;
    _target = _position;
    _settleUs = 0;
    _followingUs = 0;
    return Result::OK;
}

Result Core::home() {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (_fault != Fault::NONE) return Result::FAULT_ACTIVE;
    if (isBusy() || fabs(_velocity) > _config.velocityTolerance) return Result::BUSY;
    if ((_config.home.source != HomeSource::MIN_LIMIT && minActive()) ||
        (_config.home.source != HomeSource::MAX_LIMIT && maxActive())) return Result::LIMIT_ACTIVE;
    _homed = _referenced = _holding = false;
    _workOffset = 0.0;
    _workOffsetCounts = 0;
    resetController();
    _referencePosition = _position;
    _referenceVelocity = 0.0;
    _homeStartedUs = _nowUs;
    _homeTravel = 0.0;
    _phasePosition = _position;
    _homePhase = _homeInput.raw || _homeInput.stable ? HomePhase::RELEASE : HomePhase::SEARCH;
    _mode = Mode::HOMING;
    _completion = Completion::RUNNING;
    return Result::OK;
}

Result Core::recoverFromLimit(int8_t direction, double distance, double speed) {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (isBusy()) return Result::BUSY;
    if (_fault != Fault::NONE && _fault != Fault::MIN_LIMIT && _fault != Fault::MAX_LIMIT)
        return Result::FAULT_ACTIVE;
    if ((direction != -1 && direction != 1) || !positive(distance) ||
        distance > _config.recoveryMaxDistance || !nonnegative(speed) || speed > _config.recoveryMaxSpeed)
        return Result::INVALID_ARGUMENT;
    if ((direction == 1 && (!minActive() || maxActive())) ||
        (direction == -1 && (!maxActive() || minActive()))) return Result::LIMIT_ACTIVE;
    if (!_inputs.encoderValid || _inputs.emergency) return Result::FAULT_ACTIVE;
    if (fabs(_velocity) > _config.velocityTolerance) return Result::BUSY;
    const double target = _position + direction * distance;
    // Starting beyond a soft boundary is allowed; crossing the opposite one is not.
    if (_referenced && _config.softLimitsEnabled &&
        ((direction > 0 && target > _config.softMax) || (direction < 0 && target < _config.softMin)))
        return Result::OUT_OF_RANGE;
    _fault = Fault::NONE;
    _recoveryDirection = direction;
    speed = speed == 0.0 ? _config.recoveryMaxSpeed : speed;
    if (!startProfile(target, fmin(speed, _config.maxSpeed))) return Result::FAULT_ACTIVE;
    _mode = Mode::RECOVERY;
    return Result::OK;
}

void Core::trip(Fault fault, bool loseReference) {
    if (_fault == Fault::NONE) _fault = fault;
    if (loseReference || _mode == Mode::HOMING) _homed = _referenced = false;
    _mode = Mode::FAULT;
    _homePhase = HomePhase::NONE;
    _holding = false;
    resetController();
    _referenceVelocity = 0.0;
    _completion = Completion::FAULTED;
    _output.duty = 0.0;
    _output.velocity = 0.0;
    _output.action = _config.faultAction == FaultAction::BRAKE ? OutputAction::BRAKE : OutputAction::COAST;
}

Result Core::clearFault() {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (_inputs.emergency || !_inputs.encoderValid || minActive() || maxActive()) return Result::FAULT_ACTIVE;
    if (fabs(_velocity) > _config.velocityTolerance) return Result::BUSY;
    if (_fault == Fault::NONE) return Result::OK;
    _fault = Fault::NONE;
    _mode = Mode::IDLE;
    _homePhase = HomePhase::NONE;
    _holding = false;
    resetController();
    _referencePosition = _target = _position;
    _output.duty = 0.0;
    _output.velocity = 0.0;
    _output.action = OutputAction::COAST;
    return Result::OK;
}

void Core::emergencyStop() {
    if (_initialized) trip(Fault::EMERGENCY_STOP, true);
}

void Core::idle(Completion completion, bool hold) {
    _mode = Mode::IDLE;
    _homePhase = HomePhase::NONE;
    _completion = completion;
    _holding = hold;
    _recoveryDirection = 0;
    _referenceVelocity = 0.0;
    _referenceAcceleration = _referenceJerk = 0.0;
    _rampActive = _jogStopping = false;
    _referencePosition = _target;
    _settleUs = _followingUs = _stallUs = 0;
    if (_config.control == Control::STEPPER) _output.velocity = 0.0;
    if (!hold) {
        resetController();
        _inPosition = completion == Completion::REACHED;
        _output.duty = 0.0;
        _output.velocity = 0.0;
        _output.action = OutputAction::COAST;
    }
}

void Core::disable() {
    if (!_initialized) return;
    if (_mode == Mode::HOMING) _homed = _referenced = false;
    if (_fault == Fault::NONE) {
        _target = _position;
        idle(Completion::CANCELLED, false);
    }
    _output.duty = 0.0;
    _output.velocity = 0.0;
    _output.action = OutputAction::COAST;
}

void Core::brake() {
    disable();
    if (_initialized) _output.action = OutputAction::BRAKE;
}

void Core::invalidateReference() {
    disable();
    _homed = _referenced = false;
    _workOffset = 0.0;
    _workOffsetCounts = 0;
}

Result Core::setWorkPosition(double position) {
    Result result = ready();
    if (result != Result::OK) return result;
    if (!_homed) return Result::NOT_HOMED;
    return setRelativePosition(position);
}

Result Core::setRelativePosition(double position) {
    Result result = ready();
    if (result != Result::OK) return result;
    const double offset = _position - position;
    const double scaled = position * _config.encoder.countsPerUnit;
    if (!finiteValue(offset) || !countsFit(scaled)) return Result::INVALID_ARGUMENT;
    const int64_t absolute = getAbsolutePositionCounts();
    const int64_t relative = roundedCounts(scaled);
    if (!subtractionFits(absolute, relative)) return Result::INVALID_ARGUMENT;
    _workOffset = offset;
    _workOffsetCounts = absolute - relative;
    return Result::OK;
}

Result Core::setRelativeOrigin(double absolutePosition) {
    Result result = ready();
    if (result != Result::OK) return result;
    if (!_homed) return Result::NOT_HOMED;
    const double scaled = absolutePosition * _config.encoder.countsPerUnit;
    if (!finiteValue(_position - absolutePosition) || !countsFit(scaled)) return Result::INVALID_ARGUMENT;
    _workOffset = absolutePosition;
    _workOffsetCounts = roundedCounts(scaled);
    return Result::OK;
}

Result Core::setRelativeOriginCounts(int64_t absoluteCounts) {
    Result result = ready();
    if (result != Result::OK) return result;
    if (!_homed) return Result::NOT_HOMED;
    const double offset = static_cast<double>(absoluteCounts) / _config.encoder.countsPerUnit;
    if (!finiteValue(offset) || !finiteValue(_position - offset)) return Result::INVALID_ARGUMENT;
    _workOffset = offset;
    _workOffsetCounts = absoluteCounts;
    return Result::OK;
}

Result Core::setMaxSpeed(double value) {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (isBusy() || _holding) return Result::BUSY;
    Config candidate = _config;
    candidate.maxSpeed = value;
    if (!validConfig(candidate)) return Result::INVALID_ARGUMENT;
    _config = candidate;
    return Result::OK;
}

Result Core::setAcceleration(double value) {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (isBusy() || _holding) return Result::BUSY;
    if (!positive(value)) return Result::INVALID_ARGUMENT;
    _config.acceleration = value;
    return Result::OK;
}

Result Core::setDeceleration(double value) {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (isBusy() || _holding) return Result::BUSY;
    if (!positive(value)) return Result::INVALID_ARGUMENT;
    _config.deceleration = value;
    return Result::OK;
}

Result Core::setMaxJerk(double value) {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (isBusy() || _holding) return Result::BUSY;
    if (!nonnegative(value)) return Result::INVALID_ARGUMENT;
    _config.maxJerk = value;
    return Result::OK;
}

bool Core::advanceProfile(double dt) {
    _profileTime += dt;
    if (_config.maxJerk > 0.0) {
        MotionState state = _smoothProfile.sample(_profileTime);
        _referencePosition = _profileStart + state.position;
        _referenceVelocity = state.velocity;
        _referenceAcceleration = state.acceleration;
        _referenceJerk = state.jerk;
        return _profileTime >= _smoothProfile.duration();
    }
    double t = _profileTime, distance, velocity;
    if (t < _accelTime) {
        _referenceAcceleration = _profileSign * _config.acceleration;
        velocity = _config.acceleration * t;
        distance = 0.5 * _config.acceleration * t * t;
    } else if (t < _accelTime + _cruiseTime) {
        _referenceAcceleration = 0.0;
        velocity = _profilePeak;
        distance = 0.5 * _profilePeak * _accelTime + _profilePeak * (t - _accelTime);
    } else if (t < _accelTime + _cruiseTime + _decelTime) {
        _referenceAcceleration = -_profileSign * _config.deceleration;
        t -= _accelTime + _cruiseTime;
        velocity = _profilePeak - _config.deceleration * t;
        distance = 0.5 * _profilePeak * _accelTime + _profilePeak * _cruiseTime +
                   _profilePeak * t - 0.5 * _config.deceleration * t * t;
    } else {
        _referenceAcceleration = 0.0;
        _referencePosition = _target;
        _referenceVelocity = 0.0;
        return true;
    }
    _referencePosition = _profileStart + _profileSign * distance;
    _referenceVelocity = _profileSign * velocity;
    return false;
}

double Core::approachVelocity(double desired, double dt) {
    if (_config.maxJerk > 0.0) {
        if (!_rampActive || desired != _rampTarget) {
            if (!_smoothProfile.planVelocity(_referenceVelocity, _referenceAcceleration, desired,
                                              _config.acceleration, _config.deceleration, _config.maxJerk)) {
                trip(Fault::NUMERIC_ERROR, true);
                return 0.0;
            }
            _profileTime = _rampLastPosition = 0.0;
            _rampTarget = desired;
            _rampActive = true;
        }
        if (_profileTime >= _smoothProfile.duration()) {
            _referencePosition += desired * dt;
            _referenceVelocity = desired;
            _referenceAcceleration = _referenceJerk = 0.0;
        } else {
            _profileTime += dt;
            MotionState state = _smoothProfile.sample(_profileTime);
            _referencePosition += state.position - _rampLastPosition;
            _rampLastPosition = state.position;
            _referenceVelocity = state.velocity;
            _referenceAcceleration = state.acceleration;
            _referenceJerk = state.jerk;
        }
        return _referenceVelocity;
    }
    const double previous = _referenceVelocity;
    // Brake to zero first on reversal; this avoids crossing zero at an acceleration
    // that exceeds the configured deceleration.
    if (previous * desired < 0.0) desired = 0.0;
    const double rate = fabs(desired) > fabs(previous) ? _config.acceleration : _config.deceleration;
    _referenceVelocity += clampValue(desired - previous, -rate * dt, rate * dt);
    _referenceAcceleration = (_referenceVelocity - previous) / dt;
    _referencePosition += 0.5 * (previous + _referenceVelocity) * dt;
    return _referenceVelocity;
}

double Core::safeVelocity(double velocity) const {
    if (!softActive() || velocity == 0.0) return velocity;
    const double remaining = velocity > 0.0 ? _config.softMax - _position : _position - _config.softMin;
    // Include the maximum allowed update gap in the stopping-distance allowance.
    const double latency = _config.maxTickGapUs * 1e-6;
    double linear = latency, constant = 0.0;
    double brakingDeceleration = _config.deceleration;
    if (_config.maxJerk > 0.0) {
        const double a = fmax(0.0, _referenceAcceleration * signOf(velocity));
        const double neutralTime = a / _config.maxJerk;
        const double extraVelocity = a * latency + 0.5 * a * neutralTime;
        // Choose the triangular ramp's peak acceleration when it is lower
        // than the configured limit. This keeps the bound useful at low jerk.
        brakingDeceleration = fmin(brakingDeceleration,
                                    sqrt((fabs(velocity) + extraVelocity) * _config.maxJerk));
        // Conservative S-stop distance. For triangular acceleration ramps,
        // v^2/(2*d) + v*d/(2*j) is an upper bound, not an underestimate.
        linear += neutralTime + extraVelocity / brakingDeceleration +
                  0.5 * brakingDeceleration / _config.maxJerk;
        constant = 0.5 * a * latency * latency + a * latency * neutralTime +
                   a * neutralTime * neutralTime / 3.0 +
                   0.5 * extraVelocity * extraVelocity / brakingDeceleration +
                   0.5 * extraVelocity * brakingDeceleration / _config.maxJerk;
    }
    const double dv = brakingDeceleration * linear;
    const double distance = fmax(0.0, remaining - constant);
    const double allowed = 2.0 * brakingDeceleration * distance /
                           (sqrt(dv * dv + 2.0 * brakingDeceleration * distance) + dv);
    return signOf(velocity) * fmin(fabs(velocity), allowed);
}

void Core::homeStep(double dt) {
    const double direction = _config.home.direction;
    double desired = 0.0;
    const bool stopped = fabs(_referenceVelocity) < 1e-9 && fabs(_referenceAcceleration) < 1e-9 &&
                         fabs(_velocity) <= _config.velocityTolerance;
    switch (_homePhase) {
        case HomePhase::RELEASE:
            desired = -direction * _config.home.latchSpeed;
            if (!_homeInput.raw && !_homeInput.stable) {
                _homePhase = HomePhase::RELEASE_STOP;
                desired = 0.0;
            }
            break;
        case HomePhase::RELEASE_STOP:
            if (stopped) _homePhase = HomePhase::SEARCH;
            break;
        case HomePhase::SEARCH:
            desired = direction * _config.home.searchSpeed;
            if (_homeInput.stable) {
                _homePhase = HomePhase::SEARCH_STOP;
                desired = 0.0;
            }
            break;
        case HomePhase::SEARCH_STOP:
            if (stopped) {
                _phasePosition = _position;
                _homePhase = HomePhase::BACKOFF;
            }
            break;
        case HomePhase::BACKOFF:
            desired = -direction * _config.home.latchSpeed;
            if (!_homeInput.raw && !_homeInput.stable &&
                (_position - _phasePosition) * -direction >= _config.home.backoff) {
                _homePhase = HomePhase::BACKOFF_STOP;
                desired = 0.0;
            }
            break;
        case HomePhase::BACKOFF_STOP:
            if (stopped) _homePhase = HomePhase::LATCH;
            break;
        case HomePhase::LATCH:
            desired = direction * _config.home.latchSpeed;
            if (_homeInput.stable) {
                // Coordinate the first raw edge of the accepted stable activation,
                // rather than the later position after debounce and braking.
                _originCounts = _homeInput.edgeCount;
                _originPosition = _config.home.switchPosition;
                updatePosition();
                _samplePosition = _position;
                _referencePosition = _position;
                _referenced = true;
                _homePhase = HomePhase::LATCH_STOP;
                desired = 0.0;
            }
            break;
        case HomePhase::LATCH_STOP:
            if (stopped) {
                if (!startProfile(_config.home.parkPosition, _config.home.latchSpeed)) return;
                _homePhase = HomePhase::PARK;
            }
            break;
        default: break;
    }
    if (_homePhase != HomePhase::PARK) approachVelocity(desired, dt);
}

bool Core::checkHardLimits() {
    if (minActive() && maxActive()) { trip(Fault::BOTH_LIMITS); return false; }
    if (!isBusy() && !_holding) return true;
    bool allowMin = ((_mode == Mode::RECOVERY || _mode == Mode::STOPPING) && _recoveryDirection == 1) ||
                    (sharedHomeException() && _config.home.source == HomeSource::MIN_LIMIT);
    bool allowMax = ((_mode == Mode::RECOVERY || _mode == Mode::STOPPING) && _recoveryDirection == -1) ||
                    (sharedHomeException() && _config.home.source == HomeSource::MAX_LIMIT);
    if (minActive() && !allowMin) { trip(Fault::MIN_LIMIT); return false; }
    if (maxActive() && !allowMax) { trip(Fault::MAX_LIMIT); return false; }
    return true;
}

void Core::guardOutput() {
    if (_fault != Fault::NONE || _output.action != OutputAction::DRIVE) return;
    bool allowTowardHome = _mode == Mode::HOMING &&
        (_homePhase == HomePhase::SEARCH || _homePhase == HomePhase::LATCH) && !_homeInput.stable;
    double& command = _config.control == Control::STEPPER ? _output.velocity : _output.duty;
    if (minActive() && command < 0.0 &&
        !(allowTowardHome && _config.home.source == HomeSource::MIN_LIMIT)) command = 0.0;
    if (maxActive() && command > 0.0 &&
        !(allowTowardHome && _config.home.source == HomeSource::MAX_LIMIT)) command = 0.0;
    // Once the switch has released, reverse torque is needed to brake and
    // correct a small overshoot. An asserted limit still blocks it above.
    if (softActive()) {
        if ((_position <= _config.softMin && command < 0.0) ||
            (_position >= _config.softMax && command > 0.0)) command = 0.0;
    }
}

void Core::control(double desired, double dt) {
    if (!finiteValue(desired) || !positive(dt)) { trip(Fault::NUMERIC_ERROR, true); return; }
    _velocityCommand = safeVelocity(clampValue(desired, -_config.maxSpeed, _config.maxSpeed));
    if (_config.control == Control::STEPPER) {
        _output.duty = 0.0;
        _output.velocity = _velocityCommand;
        _output.action = OutputAction::DRIVE;
        _saturated = false;
        guardOutput();
        return;
    }
    const double error = _velocityCommand - _velocity;
    const double rawDerivative = (_config.velocityDerivativeWeight *
                                  (_velocityCommand - _previousVelocityCommand) -
                                  (_velocity - _previousVelocity)) / dt;
    _previousVelocity = _velocity;
    _previousVelocityCommand = _velocityCommand;
    double alpha = _config.derivativeFilterTime == 0.0 ? 1.0 : dt / (_config.derivativeFilterTime + dt);
    _derivative += alpha * (rawDerivative - _derivative);
    const double proportionalError = _config.velocitySetpointWeight * _velocityCommand - _velocity;
    const double base = _config.velocityKp * proportionalError + _config.velocityKd * _derivative +
                        _config.velocityFeedforward * _velocityCommand;
    const double limit = _config.maxOutput;
    const double candidate = clampValue(_integral + _config.velocityKi * error * dt, -limit, limit);
    double rawOutput = base + candidate;
    // Conditional integration: permit unwinding while the output is saturated.
    if ((rawOutput <= limit && rawOutput >= -limit) || (rawOutput > limit && error < 0.0) ||
        (rawOutput < -limit && error > 0.0)) _integral = candidate;
    rawOutput = base + _integral;
    if (!finiteValue(rawOutput)) { trip(Fault::NUMERIC_ERROR, true); return; }
    _output.duty = clampValue(rawOutput, -limit, limit);
    _saturated = fabs(rawOutput) > limit;
    _output.action = OutputAction::DRIVE;
    guardOutput();
    // A directional interlock also prevents integral accumulation against a stop.
    if (_output.duty == 0.0 && rawOutput != 0.0) _integral = 0.0;
}

void Core::checkSettled(uint32_t elapsed, bool profileDone) {
    const bool close = fabs(_target - _position) <= _config.positionTolerance &&
                       fabs(_velocity) <= _config.velocityTolerance;
    if (!profileDone || !close) { _settleUs = 0; return; }
    _settleUs = accumulate(_settleUs, elapsed, _config.settleTimeUs);
    if (_settleUs < _config.settleTimeUs) return;
    if (_mode == Mode::HOMING) {
        if (_config.home.source != HomeSource::SEPARATE && (minActive() || maxActive())) {
            trip(_config.home.source == HomeSource::MIN_LIMIT ? Fault::MIN_LIMIT : Fault::MAX_LIMIT);
            return;
        }
        _homed = _referenced = true;
    }
    if (_mode == Mode::RECOVERY && (minActive() || maxActive())) {
        trip(minActive() ? Fault::MIN_LIMIT : Fault::MAX_LIMIT);
        return;
    }
    _inPosition = true;
    idle(Completion::REACHED, _config.endAction == EndAction::HOLD);
}

bool Core::tick(uint32_t now, uint32_t raw, const Inputs& inputs) {
    return update<false>(now, raw, inputs, nullptr);
}

bool Core::tickSynchronized(uint32_t now, uint32_t raw, const Inputs& inputs, const ExternalReference& reference) {
    return update<true>(now, raw, inputs, &reference);
}

Result Core::canSynchronize() const {
    Result result = ready();
    if (result != Result::OK) return result;
    return _homed ? Result::OK : Result::NOT_HOMED;
}

Result Core::beginSynchronization() {
    Result result = canSynchronize();
    if (result != Result::OK) return result;
    resetController();
    _holding = false;
    _referencePosition = _target = _position;
    _referenceVelocity = 0.0;
    _lastSampleUs = _nowUs;
    _sampleCounts = _counts;
    _mode = Mode::SYNCHRONIZED;
    _completion = Completion::RUNNING;
    return Result::OK;
}

Result Core::validateReference(const ExternalReference& reference) const {
    if (!_initialized) return Result::NOT_INITIALIZED;
    if (!_homed) return Result::NOT_HOMED;
    const MotionState& s = reference.state;
    if (!reference.valid || !finiteValue(s.position) || !finiteValue(s.velocity) ||
        !finiteValue(s.acceleration) || !finiteValue(s.jerk)) return Result::INVALID_ARGUMENT;
    if (_config.softLimitsEnabled && (s.position < _config.softMin - _config.positionTolerance ||
                                      s.position > _config.softMax + _config.positionTolerance))
        return Result::OUT_OF_RANGE;
    double accelerationLimit = s.velocity * s.acceleration >= 0.0 ? _config.acceleration : _config.deceleration;
    if (fabs(s.velocity) > _config.maxSpeed * 1.000001 + 1e-9 ||
        fabs(s.acceleration) > accelerationLimit * 1.000001 + 1e-9 ||
        (_config.maxJerk > 0.0 && fabs(s.jerk) > _config.maxJerk * 1.000001 + 1e-9))
        return Result::OUT_OF_RANGE;
    return Result::OK;
}

Result Core::releaseSynchronization(bool decelerate) {
    if (_fault != Fault::NONE) return Result::FAULT_ACTIVE;
    if (_mode != Mode::SYNCHRONIZED) return Result::BUSY;
    if (decelerate) {
        _mode = Mode::VELOCITY;  // Enter the normal stop branch, retaining v/a.
        return stop();
    }
    disable();
    return Result::OK;
}

void Core::finishSynchronization(Completion completion) {
    if (_mode != Mode::SYNCHRONIZED || _fault != Fault::NONE) return;
    _inPosition = completion == Completion::REACHED;
    idle(completion, _config.endAction == EndAction::HOLD);
}

void Core::synchronizationFault() { if (_initialized) trip(Fault::SYNC_FAULT); }

MotionState Core::getReferenceState() const {
    MotionState state;
    state.position = _referencePosition; state.velocity = _referenceVelocity;
    state.acceleration = _referenceAcceleration; state.jerk = _referenceJerk;
    return state;
}

template <bool External>
bool Core::update(uint32_t now, uint32_t raw, const Inputs& inputs, const ExternalReference* reference) {
    if (!_initialized) return false;
    if (External && _mode != Mode::SYNCHRONIZED) return false;
    if (!External && _mode == Mode::SYNCHRONIZED) trip(Fault::SYNC_FAULT);
    _nowUs = now;
    _inputs = inputs;
    uint32_t deltaRaw = raw - _lastRaw;
    int64_t delta = deltaRaw <= INT32_MAX ? static_cast<int64_t>(deltaRaw) :
                    static_cast<int64_t>(deltaRaw) - 4294967296LL;
    _lastRaw = raw;
    if (inputs.encoderValid) {
        if ((delta > 0 && _counts > INT64_MAX - delta) || (delta < 0 && _counts < INT64_MIN - delta)) {
            trip(Fault::ENCODER_INVALID, true);
            return true;
        }
        _counts += delta;
        updatePosition();
        if (_mode == Mode::HOMING) _homeTravel += fabs(delta * signedScale());
    }
    _homeInput.update(homeRaw(inputs), now, _config.home.debounceUs, _counts);
    _minInput.update(inputs.minLimit, now, _config.limitReleaseUs, _counts);
    _maxInput.update(inputs.maxLimit, now, _config.limitReleaseUs, _counts);
    if (inputs.emergency) trip(Fault::EMERGENCY_STOP, true);
    if (!inputs.encoderValid) trip(Fault::ENCODER_INVALID, true);
    if (_fault == Fault::NONE) checkHardLimits();
    if (External && _fault == Fault::NONE &&
        (validateReference(*reference) != Result::OK ||
         static_cast<uint32_t>(now - reference->timestampUs) > _config.maxTickGapUs)) trip(Fault::SYNC_FAULT);
    const uint32_t elapsed = now - _lastSampleUs;
    if (External && !reference->updateControl) {
        if (elapsed > _config.maxTickGapUs) trip(Fault::CONTROL_TIMEOUT, true);
        guardOutput();
        return _fault != Fault::NONE;
    }
    if (elapsed < _config.controlPeriodUs) { guardOutput(); return _fault != Fault::NONE; }
    if (elapsed > _config.maxTickGapUs && (isBusy() || _holding)) trip(Fault::CONTROL_TIMEOUT, true);
    const double dt = elapsed * 1e-6;
    const double countDelta = static_cast<double>(_counts - _sampleCounts);
    if (_config.maxEncoderRate > 0.0 && fabs(countDelta) > _config.maxEncoderRate * dt + 2.0)
        trip(Fault::ENCODER_RATE, true);
    const double measured = countDelta * signedScale() / dt;
    const double alpha = _config.velocityFilterTime == 0.0 ? 1.0 : dt / (_config.velocityFilterTime + dt);
    _velocity += alpha * (measured - _velocity);
    _sampleCounts = _counts;
    _samplePosition = _position;
    _lastSampleUs = now;
    if (_fault != Fault::NONE) return true;
    if (_mode == Mode::HOMING) {
        if (static_cast<uint32_t>(now - _homeStartedUs) >= _config.home.timeoutUs) trip(Fault::HOMING_TIMEOUT, true);
        else if (_homeTravel > _config.home.maxTravel) trip(Fault::HOMING_TRAVEL, true);
        if (_fault != Fault::NONE) return true;
    }
    if (softActive() && (_position < _config.softMin - _config.positionTolerance ||
                         _position > _config.softMax + _config.positionTolerance)) {
        trip(Fault::SOFT_LIMIT);
        return true;
    }
    bool profileDone = false;
    double desired = 0.0;
    if (External) {
        _target = _referencePosition = reference->state.position;
        _referenceVelocity = reference->state.velocity;
        _referenceAcceleration = reference->state.acceleration;
        _referenceJerk = reference->state.jerk;
        desired = _referenceVelocity + _config.positionKp * (_referencePosition - _position);
    }
    if (!External && _mode == Mode::HOMING && _homePhase != HomePhase::PARK) homeStep(dt);
    if (_fault != Fault::NONE) return true;
    if (_mode == Mode::POSITION || _mode == Mode::RECOVERY ||
        (_mode == Mode::HOMING && _homePhase == HomePhase::PARK)) {
        profileDone = advanceProfile(dt);
        desired = _referenceVelocity + _config.positionKp * (_referencePosition - _position);
        desired = clampValue(desired, -_moveSpeed, _moveSpeed);
    } else if (_mode == Mode::VELOCITY) {
        if (_config.maxJerk > 0.0) {
            double cap = fabs(safeVelocity(_jogVelocity));
            double speedNow = fmax(fabs(_referenceVelocity), fabs(_velocity));
            if (softActive() && cap < fabs(_jogVelocity) &&
                cap <= speedNow + _config.acceleration * dt) _jogStopping = true;
            desired = approachVelocity(_jogStopping ? 0.0 : _jogVelocity, dt);
        } else desired = approachVelocity(safeVelocity(_jogVelocity), dt);
        // Automatically finish a jog as a controlled stop at a software boundary.
        const bool finishedJog = _config.maxJerk > 0.0 ?
            (_jogStopping && _referenceVelocity == 0.0 && _referenceAcceleration == 0.0) :
            (softActive() && fabs(safeVelocity(_jogVelocity)) <= _config.velocityTolerance);
        if (finishedJog &&
            fabs(_velocity) <= _config.velocityTolerance) {
            _target = _position;
            idle(Completion::STOPPED, _config.endAction == EndAction::HOLD);
        }
    } else if (_mode == Mode::STOPPING) {
        desired = approachVelocity(0.0, dt);
        if (_referenceVelocity == 0.0 && _referenceAcceleration == 0.0 &&
            fabs(_velocity) <= _config.velocityTolerance) {
            _settleUs = accumulate(_settleUs, elapsed, _config.settleTimeUs);
            if (_settleUs >= _config.settleTimeUs) {
                _target = _position;
                idle(Completion::STOPPED, _config.endAction == EndAction::HOLD);
            }
        } else _settleUs = 0;
    } else if (_mode == Mode::HOMING) desired = _referenceVelocity;
    if (_fault != Fault::NONE) return true;
    if (_mode == Mode::IDLE) {
        if (!_holding) {
            if (fabs(_target - _position) > _config.positionTolerance + _config.positionHysteresis ||
                fabs(_velocity) > _config.velocityTolerance) _inPosition = false;
            return true;
        }
        desired = _config.positionKp * (_target - _position);
        if (_config.control == Control::STEPPER &&
            fabs(_target - _position) <= _config.positionTolerance) desired = 0.0;
        if (fabs(_target - _position) > _config.positionTolerance + _config.positionHysteresis ||
            fabs(_velocity) > _config.velocityTolerance) {
            _inPosition = false;
            _settleUs = 0;
        } else if (!_inPosition && fabs(_target - _position) <= _config.positionTolerance) {
            _settleUs = accumulate(_settleUs, elapsed, _config.settleTimeUs);
            if (_settleUs >= _config.settleTimeUs) _inPosition = true;
        }
    }
    control(desired, dt);
    if (_fault != Fault::NONE) return true;
    bool positionControl = External || _mode == Mode::POSITION || _mode == Mode::RECOVERY || _holding ||
                           (_mode == Mode::HOMING && _homePhase == HomePhase::PARK);
    if (positionControl && _config.maxFollowingError > 0.0 &&
        fabs(_referencePosition - _position) > _config.maxFollowingError) {
        _followingUs = accumulate(_followingUs, elapsed, _config.followingErrorTimeUs);
        if (_followingUs >= _config.followingErrorTimeUs) trip(Fault::FOLLOWING_ERROR);
    } else _followingUs = 0;
    if (_config.stallOutput > 0.0 && fabs(_output.duty) >= _config.stallOutput &&
        fabs(_velocity) <= _config.stallVelocity && fabs(_velocityCommand) > _config.velocityTolerance) {
        _stallUs = accumulate(_stallUs, elapsed, _config.stallTimeUs);
        if (_stallUs >= _config.stallTimeUs) trip(Fault::STALL);
    } else _stallUs = 0;
    if (!External && _fault == Fault::NONE && positionControl && !_holding) checkSettled(elapsed, profileDone);
    // A stuck switch must not leave recovery waiting forever for a position
    // tolerance that cannot be reached while reverse output is interlocked.
    if (_fault == Fault::NONE && _mode == Mode::RECOVERY && profileDone && (minActive() || maxActive()))
        trip(minActive() ? Fault::MIN_LIMIT : Fault::MAX_LIMIT);
    return true;
}

Snapshot Core::snapshot() const {
    Snapshot s;
    s.mode = _mode; s.homePhase = _homePhase; s.fault = _fault; s.completion = _completion;
    s.homed = _homed; s.inPosition = _inPosition; s.counts = _counts;
    s.position = _position; s.workPosition = getWorkPosition(); s.velocity = _velocity;
    s.target = _target; s.referencePosition = _referencePosition; s.referenceVelocity = _referenceVelocity;
    s.followingError = _referencePosition - _position; s.velocityCommand = _velocityCommand;
    s.integral = _integral; s.saturated = _saturated; s.output = _output; s.inputs = _inputs;
    s.absolutePositionCounts = getAbsolutePositionCounts();
    s.relativePositionCounts = getRelativePositionCounts();
    s.referenceAcceleration = _referenceAcceleration;
    s.referenceJerk = _referenceJerk;
    return s;
}

const char* toString(Result v) {
    switch (v) {
        case Result::OK: return "OK";
        case Result::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case Result::INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case Result::BUSY: return "BUSY";
        case Result::NOT_HOMED: return "NOT_HOMED";
        case Result::OUT_OF_RANGE: return "OUT_OF_RANGE";
        case Result::LIMIT_ACTIVE: return "LIMIT_ACTIVE";
        case Result::FAULT_ACTIVE: return "FAULT_ACTIVE";
        case Result::UNSUPPORTED: return "UNSUPPORTED";
    }
    return "UNKNOWN";
}
const char* toString(Fault v) {
    switch (v) {
        case Fault::NONE: return "NONE";
        case Fault::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case Fault::MIN_LIMIT: return "MIN_LIMIT";
        case Fault::MAX_LIMIT: return "MAX_LIMIT";
        case Fault::BOTH_LIMITS: return "BOTH_LIMITS";
        case Fault::SOFT_LIMIT: return "SOFT_LIMIT";
        case Fault::ENCODER_INVALID: return "ENCODER_INVALID";
        case Fault::ENCODER_RATE: return "ENCODER_RATE";
        case Fault::CONTROL_TIMEOUT: return "CONTROL_TIMEOUT";
        case Fault::FOLLOWING_ERROR: return "FOLLOWING_ERROR";
        case Fault::STALL: return "STALL";
        case Fault::HOMING_TIMEOUT: return "HOMING_TIMEOUT";
        case Fault::HOMING_TRAVEL: return "HOMING_TRAVEL";
        case Fault::NUMERIC_ERROR: return "NUMERIC_ERROR";
        case Fault::SYNC_FAULT: return "SYNC_FAULT";
    }
    return "UNKNOWN";
}
const char* toString(Mode v) {
    switch (v) {
        case Mode::IDLE: return "IDLE";
        case Mode::POSITION: return "POSITION";
        case Mode::VELOCITY: return "VELOCITY";
        case Mode::STOPPING: return "STOPPING";
        case Mode::HOMING: return "HOMING";
        case Mode::RECOVERY: return "RECOVERY";
        case Mode::FAULT: return "FAULT";
        case Mode::SYNCHRONIZED: return "SYNCHRONIZED";
    }
    return "UNKNOWN";
}
const char* toString(Completion v) {
    switch (v) {
        case Completion::NONE: return "NONE";
        case Completion::RUNNING: return "RUNNING";
        case Completion::REACHED: return "REACHED";
        case Completion::STOPPED: return "STOPPED";
        case Completion::CANCELLED: return "CANCELLED";
        case Completion::FAULTED: return "FAULTED";
    }
    return "UNKNOWN";
}

}  // namespace axis
