#pragma once
#include <AxisStepper.h>

static uint32_t stepperPulses = 0, stepperFeedback = 0;
static int64_t stepperTravel = 0;
static bool stepperPower = false, stepperFrozen = false;
static bool stepperInvertEncoder = false;
static axis::Inputs stepperInputs;
static void stepperStep(uint8_t direction) {
    ++stepperPulses;
    stepperTravel += direction ? 1 : -1;
    // Encoder has twice the resolution of the motor step count.
    if (!stepperFrozen) {
        int delta = direction ? 2 : -2;
        if (stepperInvertEncoder) delta = -delta;
        stepperFeedback += static_cast<uint32_t>(delta);
    }
}
static void stepperSetPower(bool power) { stepperPower = power; }
static uint32_t stepperReadEncoder(void*) { return stepperFeedback; }
static axis::Inputs stepperReadInputs(void*) { return stepperInputs; }
typedef AxisStepper<STEPPER2WIRE, STEPPER_VIRTUAL> TestStepper;

static void prepareStepper(TestStepper& motor, bool encoder = false) {
    stepperPulses = stepperFeedback = 0;
    stepperTravel = 0;
    stepperPower = stepperFrozen = stepperInvertEncoder = false;
    stepperInputs = axis::Inputs();
    CHECK(motor.attachStep(stepperStep) == Result::OK);
    CHECK(motor.attachPower(stepperSetPower) == Result::OK);
    CHECK(motor.attachInputs(stepperReadInputs) == Result::OK);
    CHECK(motor.setStepsPerUnit(100) == Result::OK);
    if (encoder) CHECK(motor.attachEncoder(stepperReadEncoder) == Result::OK);
    axis::Config config = testConfig();
    config.encoder = axis::EncoderScale::linear(200);
    config.positionTolerance = 0.02;
    config.maxJerk = 200;
    CHECK(motor.begin(config) == Result::OK);
    CHECK(motor.hasEncoder() == encoder);
    CHECK(motor.getConfig().control == axis::Control::STEPPER);
    NEAR(motor.getConfig().encoder.countsPerUnit, encoder ? 200 : 100, 0);
}
static void runStepper(TestStepper& motor, unsigned us = 1000000) {
    for (unsigned t = 0; t < us; t += 100) {
        mockMicros += 100;
        motor.tick();
    }
}
static void homeStepper(TestStepper& motor) {
    CHECK(motor.home() == Result::OK);
    for (unsigned i = 0; i < 150000 && motor.isBusy(); ++i) {
        stepperInputs.home = stepperTravel <= -100;
        mockMicros += 100;
        motor.tick();
    }
    CHECK(motor.getFault() == Fault::NONE);
    CHECK(motor.isHomed());
}

static void stepperTests() {
    {
        TestStepper motor;
        CHECK(motor.begin(testConfig()) == Result::INVALID_ARGUMENT);
        prepareStepper(motor);
        CHECK(motor.setStepsPerUnit(0) == Result::INVALID_ARGUMENT);
        CHECK(motor.setStepsPerUnit(std::numeric_limits<double>::infinity()) == Result::INVALID_ARGUMENT);
        CHECK(motor.setMaxSpeed(1e30) == Result::INVALID_ARGUMENT);
        CHECK(motor.moveTo(1) == Result::NOT_HOMED);
        CHECK(motor.moveBy(1, 1) == Result::OK);
        CHECK(motor.setStepsPerUnit(200) == Result::BUSY);
        CHECK(motor.detachEncoder() == Result::BUSY);
        runStepper(motor, 2500000);
        CHECK(!motor.isBusy()); CHECK(motor.inPosition());
        CHECK(stepperPower);
        NEAR(motor.getPosition(), 1, 0.02);
        CHECK(motor.getStepCount() == stepperPulses);
        CHECK(motor.getCounts() == static_cast<int64_t>(motor.getStepCount()));
        CHECK(motor.setRelativePosition() == Result::OK);
        CHECK(motor.moveBy(-2, 1) == Result::OK);
        runStepper(motor, 3500000);
        CHECK(motor.getFault() == Fault::NONE); CHECK(!motor.isBusy());
        NEAR(motor.getPosition(), -1, 0.04);
        NEAR(motor.getRelativePosition(), -2, 0.02);
        CHECK(motor.getCounts() < 0);
        uint32_t pulses = stepperPulses;
        runStepper(motor, 500000);
        CHECK(stepperPulses == pulses);  // Holding does not dither by one step.
        motor.brake(); CHECK(stepperPower);
        motor.disable(); CHECK(!stepperPower);
        CHECK(motor.setMotorReverse(true) == Result::OK);
        CHECK(motor.moveBy(0.5, 1) == Result::OK);
        runStepper(motor, 2000000);
        CHECK(!motor.isBusy());
        CHECK(motor.getCounts() > -100);  // Reverse changes hardware, not coordinates.
    }
    {
        TestStepper motor;
        prepareStepper(motor, true);
        stepperInvertEncoder = true;
        axis::Config config = motor.getConfig();
        config.encoder.inverted = true;
        config.velocityKp = config.velocityKi = config.velocityFeedforward = 0;
        CHECK(motor.begin(config) == Result::OK);  // No DC speed PID required.
        CHECK(motor.moveBy(1, 1) == Result::OK);
        runStepper(motor, 2500000);
        CHECK(!motor.isBusy()); CHECK(motor.getFault() == Fault::NONE);
        NEAR(motor.getPosition(), 1, 0.02);
        CHECK(motor.getCounts() < 0);
        CHECK(motor.getCounts() == -2 * static_cast<int64_t>(motor.getStepCount()));
        // Measured position, not the commanded pulse count, controls the axis.
        stepperFeedback -= 40;
        runStepper(motor, 1500000);
        NEAR(motor.getPosition(), 1, 0.02);
        CHECK(motor.getStepCount() < 100);
        CHECK(motor.detachEncoder() == Result::OK);
        CHECK(!motor.isInitialized()); CHECK(!motor.hasEncoder());
        CHECK(motor.begin(testConfig()) == Result::OK);
    }
    {
        TestStepper motor;
        prepareStepper(motor, true);
        axis::Config config = motor.getConfig();
        config.maxFollowingError = 0.1;
        config.followingErrorTimeUs = 20000;
        CHECK(motor.begin(config) == Result::OK);
        stepperFrozen = true;
        CHECK(motor.moveBy(1, 1) == Result::OK);
        runStepper(motor, 1000000);
        CHECK(stepperPulses > 0); CHECK(motor.getCounts() == 0);
        CHECK(motor.getFault() == Fault::FOLLOWING_ERROR); CHECK(!stepperPower);
        uint32_t pulses = stepperPulses;
        runStepper(motor); CHECK(stepperPulses == pulses);
    }
    {
        TestStepper motor;
        prepareStepper(motor);
        CHECK(motor.moveBy(2, 1) == Result::OK);
        runStepper(motor, 300000);
        stepperInputs.minLimit = true;
        uint32_t pulses = stepperPulses;
        mockMicros += 100;
        motor.tick();
        CHECK(motor.getFault() == Fault::MIN_LIMIT);
        CHECK(stepperPulses == pulses); CHECK(!stepperPower);
        stepperInputs.minLimit = false;
        runStepper(motor, 500000);
        CHECK(motor.clearFault() == Result::OK);
        CHECK(motor.moveBy(1, 1) == Result::OK);
        runStepper(motor, 300000);
        pulses = stepperPulses;
        stepperInputs.emergency = true;
        CHECK(motor.moveBy(1) == Result::FAULT_ACTIVE);
        CHECK(motor.getFault() == Fault::EMERGENCY_STOP);
        CHECK(stepperPulses == pulses);
    }
    {
        TestStepper motor;
        mockMicros = 0xffff0000UL;
        prepareStepper(motor);
        CHECK(motor.moveBy(2, 1) == Result::OK);
        runStepper(motor, 500000);  // micros() wraps.
        CHECK(motor.getFault() == Fault::NONE); CHECK(stepperPulses > 0);
        CHECK(motor.stop() == Result::OK);
        runStepper(motor, 1500000);
        CHECK(!motor.isBusy()); CHECK(motor.getCompletion() == axis::Completion::STOPPED);
        CHECK(motor.getPosition() < 2);
        CHECK(motor.moveBy(1, 1) == Result::OK);
        runStepper(motor, 300000);
        uint32_t pulses = stepperPulses;
        mockMicros += 100000;
        motor.tick();
        CHECK(motor.getFault() == Fault::CONTROL_TIMEOUT);
        CHECK(stepperPulses == pulses);
    }
    {
        TestStepper motor;
        prepareStepper(motor);
        homeStepper(motor);
        CHECK(motor.moveTo(1, 1) == Result::OK);
        runStepper(motor, 2500000);
        CHECK(motor.inPosition()); NEAR(motor.getPosition(), 1, 0.02);
        axis::AxisPort port = motor.syncPort();
        CHECK(port.valid());
        CHECK(port.core->beginSynchronization() == Result::OK);
        CHECK(motor.moveBy(1) == Result::BUSY);
        axis::ExternalReference reference;
        reference.state.position = 1.5;
        for (unsigned i = 0; i < 20000; ++i) {
            mockMicros += 100;
            reference.timestampUs = mockMicros;
            port.update(mockMicros, &reference);
            port.commit();
        }
        CHECK(motor.getFault() == Fault::NONE);
        NEAR(motor.getPosition(), 1.5, 0.02);
        port.core->finishSynchronization(); port.commit();
        CHECK(motor.getMode() == axis::Mode::IDLE);
    }
    {
        // A physical stepper and a simulated DC axis use the same coordinator.
        DrivenPlant dc;
        dc.prepare();
        TestStepper motor;
        prepareStepper(motor);
        homeStepper(motor);
        dc.now = mockMicros;
        dc.port().update(mockMicros); dc.port().commit();
        const double dcStart = dc.core.getPosition(), start = motor.getPosition();
        AxisGroup<2> group;
        CHECK(group.add(motor.syncPort(), 1) == Result::OK);
        CHECK(group.add(dc.port(), 2) == Result::OK);
        CHECK(group.moveBy(1, 1) == Result::OK);
        for (unsigned i = 0; i < 60000 && group.isBusy(); ++i) {
            dc.advance(100); mockMicros += 100;
            group.tick();
        }
        CHECK(!group.isBusy()); CHECK(group.getError() == axis::SyncError::NONE);
        NEAR(motor.getPosition() - start, 1, 0.02);
        NEAR(dc.core.getPosition() - dcStart, 2, dc.config.positionTolerance);
        CHECK(group.moveBy(1, 1) == Result::OK);
        for (unsigned i = 0; i < 3000; ++i) {
            dc.advance(100); mockMicros += 100; group.tick();
        }
        // An input fault on the later DC member prevents another step on X.
        dc.switches = false; dc.inputs.emergency = true;
        uint32_t pulses = stepperPulses;
        dc.advance(100); mockMicros += 100; group.tick();
        CHECK(group.getError() == axis::SyncError::AXIS_FAULT);
        CHECK(stepperPulses == pulses); CHECK(!stepperPower);
        CHECK(motor.getFault() == Fault::SYNC_FAULT);
    }
    {
        TestStepper motor;
        prepareStepper(motor);
        axis::Config config = motor.getConfig();
        config.kind = axis::Kind::ROTARY;
        config.continuous = true;
        config.endAction = axis::EndAction::COAST;
        config.faultAction = axis::FaultAction::BRAKE;
        CHECK(motor.begin(config) == Result::OK);
        CHECK(motor.moveBy(1, 1) == Result::OK);
        runStepper(motor, 2500000);
        CHECK(!motor.isBusy()); CHECK(!stepperPower);
        NEAR(motor.getAngle(), 1, 0.02);
        CHECK(motor.moveBy(1, 1) == Result::OK);
        runStepper(motor, 300000);
        uint32_t pulses = stepperPulses;
        motor.emergencyStop();
        CHECK(stepperPower); CHECK(stepperPulses == pulses);
        runStepper(motor, 500000);
        CHECK(stepperPulses == pulses);
    }
    {
        TestStepper motor;
        prepareStepper(motor);
        CHECK(motor.moveBy(2, 2) == Result::OK);
        // Service too slowly for 200 steps/s; only actually emitted steps count.
        for (unsigned i = 0; i < 60; ++i) {
            uint32_t pulses = stepperPulses;
            mockMicros += 20000; motor.tick();
            CHECK(stepperPulses - pulses <= 1);
            // The sample precedes the output commit by at most one step.
            CHECK(static_cast<int64_t>(motor.getStepCount()) - motor.getCounts() <= 1);
        }
        CHECK(stepperPulses > 0);
        CHECK(motor.getPosition() < 1);
    }
    {
        // Real pin backend, including enable polarity and both phase modes.
        AxisStepper<STEPPER2WIRE> motor(40, 41, 42);
        CHECK(mockDigital[42] == HIGH);
        CHECK(motor.setEnableInverted(true) == Result::OK);
        CHECK(mockDigital[42] == LOW);
        CHECK(motor.begin(testConfig()) == Result::OK);
        CHECK(motor.moveBy(1, 1) == Result::OK);
        for (unsigned i = 0; i < 20000; ++i) { mockMicros += 100; motor.tick(); }
        CHECK(motor.inPosition()); CHECK(mockDigital[42] == HIGH);
        CHECK(mockDigital[40] == LOW); CHECK(mockDigital[41] == HIGH);
        motor.disable(); CHECK(mockDigital[42] == LOW);
        AxisStepper<STEPPER4WIRE> full(43, 44, 45, 46);
        AxisStepper<STEPPER4WIRE_HALF> half(47, 48, 49, 50);
        CHECK(full.begin(testConfig()) == Result::OK);
        CHECK(half.begin(testConfig()) == Result::OK);
        CHECK(full.moveBy(1, 1) == Result::OK);
        CHECK(half.moveBy(-1, 1) == Result::OK);
        for (unsigned i = 0; i < 20000; ++i) {
            mockMicros += 100; full.tick(); half.tick();
        }
        CHECK(full.inPosition()); CHECK(half.inPosition());
        full.disable(); half.disable();
        for (unsigned pin = 43; pin <= 50; ++pin) CHECK(mockDigital[pin] == LOW);
    }
}
