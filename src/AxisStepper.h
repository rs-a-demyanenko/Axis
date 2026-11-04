#pragma once

#include <StepperCore.h>
#include <math.h>
#include "AxisArduino.h"
#include "AxisCoordinators.h"
#include "AxisQuadrature.h"

// StepperCore provides only individual steps and driver power. Axis owns the
// trajectory, position correction, homing, limits and synchronization.
template <GS_driverType driver, GS_driverType interface = STEPPER_PINS>
class AxisStepper : protected Stepper<driver, interface>,
                    public axis::ArduinoAxis<AxisStepper<driver, interface>> {
    typedef Stepper<driver, interface> Motor;
    typedef axis::ArduinoAxis<AxisStepper<driver, interface>> Base;
    friend class axis::ArduinoAxis<AxisStepper<driver, interface>>;
    static_assert(driver == STEPPER2WIRE || driver == STEPPER4WIRE || driver == STEPPER4WIRE_HALF,
                  "Unsupported stepper driver");
    static_assert(interface == STEPPER_PINS || interface == STEPPER_VIRTUAL,
                  "Unsupported stepper interface");

 public:
    // STEP/DIR: step, dir, enable. Four-wire: phase1..phase4, enable.
    AxisStepper(uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255,
                uint8_t pin4 = 255, uint8_t pin5 = 255)
        : Motor(pin1, pin2, pin3, pin4, pin5) { Motor::disable(); }
    using Base::disable;

    axis::Result setStepsPerUnit(double steps) {
        if (!isfinite(steps) || steps <= 0.0) return axis::Result::INVALID_ARGUMENT;
        if (this->_core.isBusy()) return axis::Result::BUSY;
        this->_core.invalidateReference();
        this->apply();
        _stepsPerUnit = steps;
        this->_needsBegin = true;
        return axis::Result::OK;
    }
    double getStepsPerUnit() const { return _stepsPerUnit; }
    bool hasEncoder() const { return this->_encoderReader != nullptr; }
    axis::Result setMaxSpeed(double speed) {
        if (!isfinite(speed * _stepsPerUnit) || speed * _stepsPerUnit > 1000000.0)
            return axis::Result::INVALID_ARGUMENT;
        return this->_core.setMaxSpeed(speed);
    }
    // Signed logical steps, modulo 2^32, independent of encoder scale/origin.
    uint32_t getStepCount() const { return _stepCount; }

    axis::Result detachEncoder() {
        if (this->_core.isBusy()) return axis::Result::BUSY;
        this->_core.invalidateReference();
        this->apply();
        this->_encoderReader = nullptr;
        this->_encoderContext = nullptr;
        this->_needsBegin = true;
        return axis::Result::OK;
    }
    axis::Result setMotorReverse(bool reverse) {
        if (this->_core.isBusy()) return axis::Result::BUSY;
        this->_core.invalidateReference();
        this->apply();
        Motor::reverse(reverse);
        return axis::Result::OK;
    }
    axis::Result setEnableInverted(bool inverted) {
        if (this->_core.isBusy()) return axis::Result::BUSY;
        this->_core.invalidateReference();
        Motor::invertEn(inverted);
        _powered = true;  // Force the new inactive level to the driver.
        this->apply();
        return axis::Result::OK;
    }
    axis::Result attachStep(void (*handler)(uint8_t)) {
        if (interface != STEPPER_VIRTUAL) return axis::Result::UNSUPPORTED;
        if (!handler) return axis::Result::INVALID_ARGUMENT;
        if (this->_core.isBusy()) return axis::Result::BUSY;
        this->_core.invalidateReference();
        this->apply();
        Motor::attachStep(handler);
        _hasStepHandler = true;
        this->_needsBegin = true;
        return axis::Result::OK;
    }
    axis::Result attachPower(void (*handler)(bool)) {
        if (interface != STEPPER_VIRTUAL) return axis::Result::UNSUPPORTED;
        if (!handler) return axis::Result::INVALID_ARGUMENT;
        if (this->_core.isBusy()) return axis::Result::BUSY;
        this->_core.invalidateReference();
        this->apply();
        Motor::attachPower(handler);
        Motor::disable();
        return axis::Result::OK;
    }

 private:
    double _stepsPerUnit = 100.0, _stepVelocity = 0.0, _phase = 0.0;
    uint32_t _stepCount = 0, _lastStepUs = 0;
    bool _powered = false, _hasStepHandler = false;

    axis::Result prepare(axis::Config& config) {
        if (interface == STEPPER_VIRTUAL && !_hasStepHandler) return axis::Result::INVALID_ARGUMENT;
        if (!isfinite(config.maxSpeed * _stepsPerUnit) ||
            config.maxSpeed * _stepsPerUnit > 1000000.0) return axis::Result::INVALID_ARGUMENT;
        config.control = axis::Control::STEPPER;
        if (!hasEncoder()) config.encoder = axis::EncoderScale::linear(_stepsPerUnit);
        return axis::Result::OK;
    }
    uint32_t readFeedback() {
        return hasEncoder() ? this->_encoderReader(this->_encoderContext) : _stepCount;
    }
    void resetDriver() {
        _lastStepUs = static_cast<uint32_t>(micros());
        _stepVelocity = _phase = 0.0;
    }
    void applyOutput() {
        const axis::Output& output = this->_core.getOutput();
        const uint32_t now = static_cast<uint32_t>(micros());
        const uint32_t elapsed = now - _lastStepUs;
        _lastStepUs = now;
        const bool powered = output.action != axis::OutputAction::COAST;
        if (powered != _powered) { Motor::power(powered); _powered = powered; }
        const double speed = output.action == axis::OutputAction::DRIVE ?
                             output.velocity * _stepsPerUnit : 0.0;
        if (speed == 0.0 || (_stepVelocity < 0.0) != (speed < 0.0)) _phase = 0.0;
        else _phase += fabs(_stepVelocity) * (elapsed * 1e-6);
        _stepVelocity = speed;
        if (speed == 0.0 || _phase < 1.0) return;
        // At most one physical step per service call. Late calls never produce
        // a burst or pretend that unissued pulses have moved the shaft.
        _phase -= floor(_phase);
        Motor::dir = speed > 0.0 ? 1 : -1;
        // StepperCore's signed pos is private to this adapter. Keep it bounded;
        // our unsigned pulse counter and Core's 64-bit coordinate handle wrap.
        Motor::pos = 0;
        Motor::step();
        _stepCount += Motor::dir > 0 ? uint32_t(1) : uint32_t(-1);
    }
};
