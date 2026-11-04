#pragma once

#include <GyverMotor2.h>
#include <math.h>
#include "AxisArduino.h"
#include "AxisQuadrature.h"

template <GM2 driver, uint8_t resolution = 8>
class Axis : protected GyverMotor2<driver, resolution>, public axis::ArduinoAxis<Axis<driver, resolution>> {
    typedef GyverMotor2<driver, resolution> Motor;
    typedef axis::ArduinoAxis<Axis<driver, resolution>> Base;
    friend class axis::ArduinoAxis<Axis<driver, resolution>>;

 public:
    using Base::tick;
    using Base::stop;
    using Base::brake;
    using Base::disable;
    using Base::getTarget;
    Axis(uint8_t pinA, uint8_t pinB = 0xff, uint8_t pinC = 0xff) : Motor(pinA, pinB, pinC) {
        Motor::setAccel(0);
        Motor::setMinDuty(0);
    }
    int16_t getDuty() const { return _appliedDuty; }
    axis::Result setMotorReverse(bool reverse) {
        if (this->_core.isBusy()) return axis::Result::BUSY;
        this->_core.invalidateReference();
        this->apply();
        Motor::setReverse(reverse);
        return axis::Result::OK;
    }
    axis::Result setDeadtime(uint8_t microseconds) {
        if (this->_core.isBusy()) return axis::Result::BUSY;
        Motor::setDeadtime(microseconds);
        return axis::Result::OK;
    }

 private:
    bool _applied = false;
    int16_t _appliedDuty = 0;
    axis::OutputAction _appliedAction = axis::OutputAction::COAST;
    axis::Result prepare(axis::Config& config) {
        if (driver == GM2::ONLY_PWM || driver == GM2::DIR_DIR) return axis::Result::UNSUPPORTED;
        if (!this->_encoderReader) return axis::Result::INVALID_ARGUMENT;
        config.control = axis::Control::DC;
        return axis::Result::OK;
    }
    void resetDriver() {}
    uint32_t readFeedback() { return this->_encoderReader(this->_encoderContext); }
    void applyOutput() {
        axis::Output output = this->_core.getOutput();
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

#include "AxisCoordinators.h"
