#pragma once

#include <Arduino.h>
#include <GyverMotor2.h>
#include <math.h>

#include "AxisCore.h"
#include "AxisQuadrature.h"

namespace axis {

struct InputPin {
    uint8_t pin = 0xff;
    bool activeLow = true;
    bool pullup = true;
    InputPin() {}
    InputPin(uint8_t number, bool low = true, bool usePullup = true)
        : pin(number), activeLow(low), pullup(usePullup) {}
};

struct Pins {
    InputPin minLimit, maxLimit, home, emergency;
};

typedef uint32_t (*EncoderReader)(void* context);
typedef Inputs (*InputsReader)(void* context);

}  // namespace axis

// Only this adapter touches Arduino GPIO. Control math remains host-testable.
// All methods must be called from the same task; only the encoder counter's
// update belongs in an interrupt. Readers must return an atomic snapshot.
template <GM2 driver, uint8_t resolution = 8>
class Axis : protected GyverMotor2<driver, resolution> {
    typedef GyverMotor2<driver, resolution> Motor;

 public:
    Axis(uint8_t pinA, uint8_t pinB = 0xff, uint8_t pinC = 0xff) : Motor(pinA, pinB, pinC) {
        Motor::setAccel(0);
        Motor::setMinDuty(0);
    }
    Axis(const Axis&) = delete;
    Axis& operator=(const Axis&) = delete;

    axis::Result attachEncoder(axis::EncoderReader reader, void* context = nullptr) {
        if (!reader) return axis::Result::INVALID_ARGUMENT;
        if (_core.isBusy()) return axis::Result::BUSY;
        _core.invalidateReference();
        apply();
        _encoderReader = reader;
        _encoderContext = context;
        _needsBegin = true;
        return axis::Result::OK;
    }

    // Call before begin(). For custom hardware (I2C expander, driver fault, etc.)
    // attachInputs() may replace pin reading with a logical-input callback.
    axis::Result setPins(const axis::Pins& pins) {
        if (_core.isBusy()) return axis::Result::BUSY;
        _core.disable();
        apply();
        _pins = pins;
        _needsBegin = true;
        setupPin(pins.minLimit); setupPin(pins.maxLimit);
        setupPin(pins.home); setupPin(pins.emergency);
        return axis::Result::OK;
    }

    axis::Result attachInputs(axis::InputsReader reader, void* context = nullptr) {
        if (_core.isBusy()) return axis::Result::BUSY;
        _core.invalidateReference();
        apply();
        _inputsReader = reader;
        _inputsContext = context;
        _needsBegin = true;
        return axis::Result::OK;
    }

    axis::Result begin(const axis::Config& config) {
        // These two GyverMotor2 modes cannot provide bidirectional variable duty.
        if (driver == GM2::ONLY_PWM || driver == GM2::DIR_DIR) return axis::Result::UNSUPPORTED;
        if (!_encoderReader) return axis::Result::INVALID_ARGUMENT;
        axis::Inputs inputs = readInputs();
        uint32_t count = _encoderReader(_encoderContext);
        axis::Result result = _core.begin(config, static_cast<uint32_t>(micros()), count, inputs);
        if (result == axis::Result::OK || result == axis::Result::FAULT_ACTIVE) _needsBegin = false;
        apply();
        return result;
    }

    bool tick() {
        if (_needsBegin || !_encoderReader || !_core.isInitialized()) return false;
        axis::Inputs inputs = readInputs();
        uint32_t count = _encoderReader(_encoderContext);
        bool updated = _core.tick(static_cast<uint32_t>(micros()), count, inputs);
        apply();
        return updated;
    }

    axis::Result moveTo(double position, double speed = 0.0, axis::Frame frame = axis::Frame::MACHINE) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.moveTo(position, speed, frame);
    }
    axis::Result moveBy(double distance, double speed = 0.0) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.moveBy(distance, speed);
    }
    axis::Result moveToAngle(double angle, axis::Direction direction = axis::Direction::SHORTEST,
                             double speed = 0.0) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.moveToAngle(angle, direction, speed);
    }
    axis::Result jog(double velocity) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.jog(velocity);
    }
    axis::Result stop() {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.stop();
    }
    axis::Result home() {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.home();
    }
    axis::Result recoverFromLimit(int8_t direction, double distance, double speed = 0.0) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        axis::Result result = _core.recoverFromLimit(direction, distance, speed);
        apply();
        return result;
    }
    axis::Result clearFault() {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        axis::Result result = _core.clearFault();
        apply();
        return result;
    }
    void emergencyStop() { _core.emergencyStop(); apply(); }
    void disable() { _core.disable(); apply(); }
    void brake() { _core.brake(); apply(); }
    void invalidateReference() { _core.invalidateReference(); apply(); }

    axis::Result setWorkPosition(double position) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.setWorkPosition(position);
    }
    axis::Result setRelativePosition(double position = 0.0) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.setRelativePosition(position);
    }
    axis::Result setRelativeOrigin(double absolutePosition) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.setRelativeOrigin(absolutePosition);
    }
    axis::Result setRelativeOriginCounts(int64_t absoluteCounts) {
        if (!refresh()) return axis::Result::NOT_INITIALIZED;
        return _core.setRelativeOriginCounts(absoluteCounts);
    }
    axis::Result setMaxSpeed(double value) { return _core.setMaxSpeed(value); }
    axis::Result setAcceleration(double value) { return _core.setAcceleration(value); }
    axis::Result setDeceleration(double value) { return _core.setDeceleration(value); }
    axis::Result setMaxJerk(double value) { return _core.setMaxJerk(value); }

    axis::Result setMotorReverse(bool reverse) {
        if (_core.isBusy()) return axis::Result::BUSY;
        _core.invalidateReference();
        apply();
        Motor::setReverse(reverse);
        return axis::Result::OK;
    }
    // Deadtime is the only deliberate, bounded blocking operation inherited
    // from the motor driver; typically zero for integrated H bridges.
    axis::Result setDeadtime(uint8_t microseconds) {
        if (_core.isBusy()) return axis::Result::BUSY;
        Motor::setDeadtime(microseconds);
        return axis::Result::OK;
    }

    bool isInitialized() const { return !_needsBegin && _core.isInitialized(); }
    bool isHomed() const { return isInitialized() && _core.isHomed(); }
    bool isBusy() const { return _core.isBusy(); }
    bool inPosition() const { return _core.inPosition(); }
    double getPosition() const { return _core.getPosition(); }
    double getWorkPosition() const { return _core.getWorkPosition(); }
    double getAbsolutePosition() const { return _core.getAbsolutePosition(); }
    double getRelativePosition() const { return _core.getRelativePosition(); }
    int64_t getAbsolutePositionCounts() const { return _core.getAbsolutePositionCounts(); }
    int64_t getRelativePositionCounts() const { return _core.getRelativePositionCounts(); }
    double getVelocity() const { return _core.getVelocity(); }
    double getTarget() const { return _core.getTarget(); }
    double getAngle() const { return _core.getAngle(); }
    int64_t getCounts() const { return _core.getCounts(); }
    int16_t getDuty() const { return _appliedDuty; }
    axis::Fault getFault() const { return _core.getFault(); }
    axis::Mode getMode() const { return _core.getMode(); }
    axis::HomePhase getHomePhase() const { return _core.getHomePhase(); }
    axis::Completion getCompletion() const { return _core.getCompletion(); }
    const axis::Config& getConfig() const { return _core.getConfig(); }
    axis::Snapshot snapshot() const { return _core.snapshot(); }

 private:
    axis::Core _core;
    axis::Pins _pins;
    axis::EncoderReader _encoderReader = nullptr;
    axis::InputsReader _inputsReader = nullptr;
    void* _encoderContext = nullptr;
    void* _inputsContext = nullptr;
    bool _needsBegin = true, _applied = false;
    int16_t _appliedDuty = 0;
    axis::OutputAction _appliedAction = axis::OutputAction::COAST;

    static void setupPin(const axis::InputPin& input) {
        if (input.pin != 0xff) pinMode(input.pin, input.pullup ? INPUT_PULLUP : INPUT);
    }
    static bool readPin(const axis::InputPin& input) {
        return input.pin != 0xff && (digitalRead(input.pin) == (input.activeLow ? LOW : HIGH));
    }
    axis::Inputs readInputs() const {
        if (_inputsReader) return _inputsReader(_inputsContext);
        axis::Inputs inputs;
        inputs.minLimit = readPin(_pins.minLimit);
        inputs.maxLimit = readPin(_pins.maxLimit);
        inputs.home = readPin(_pins.home);
        inputs.emergency = readPin(_pins.emergency);
        return inputs;
    }
    bool refresh() {
        if (!isInitialized()) return false;
        tick();
        return true;
    }
    void apply() {
        axis::Output output = _core.getOutput();
        int16_t duty = static_cast<int16_t>(round(output.duty * Motor::getMaxDuty()));
        if (_applied && _appliedDuty == duty && _appliedAction == output.action) return;
        if (output.action == axis::OutputAction::BRAKE) Motor::brake();
        else if (output.action == axis::OutputAction::COAST) Motor::disable();
        else Motor::runSpeed(duty);
        _applied = true;
        _appliedDuty = duty;
        _appliedAction = output.action;
    }
};

