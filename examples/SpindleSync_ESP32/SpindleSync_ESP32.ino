// Classic ESP32: Z axis + external spindle encoder, 1 mm per revolution.
// GPIO34/35: external 3.3 V pull-ups if needed; no internal pull-ups available.
// h home Z; f LIMITED (can engage running spindle); g STRICT (engage at rest);
// s disengage and S-stop Z; d disable Z; c clear fault; e emergency stop Z.
// This example does not control spindle power. Use the fault callback below
// to request a spindle stop through YOUR hardware/application interface.
#include <Axis.h>

Axis<GM2::DIR_PWM> z(26, 25);
AxisGear coupling;
axis::Quadrature zEncoder, spindleEncoder;
axis::EncoderMaster spindle;
axis::EncoderMasterConfig spindleConfig;
portMUX_TYPE zMux = portMUX_INITIALIZER_UNLOCKED, spindleMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool stopSpindleRequested = false;
void ARDUINO_ISR_ATTR zISR() {
    portENTER_CRITICAL_ISR(&zMux); zEncoder.update(digitalRead(32), digitalRead(33)); portEXIT_CRITICAL_ISR(&zMux);
}
void ARDUINO_ISR_ATTR spindleISR() {
    portENTER_CRITICAL_ISR(&spindleMux);
    spindleEncoder.update(digitalRead(34), digitalRead(35));
    portEXIT_CRITICAL_ISR(&spindleMux);
}
uint32_t readZ(void*) {
    portENTER_CRITICAL(&zMux); uint32_t count = zEncoder.readUnsafe(); portEXIT_CRITICAL(&zMux); return count;
}
uint32_t readSpindle() {
    portENTER_CRITICAL(&spindleMux);
    uint32_t count = spindleEncoder.readUnsafe();
    portEXIT_CRITICAL(&spindleMux); return count;
}
void syncFault(axis::SyncError, void*) {
    stopSpindleRequested = true;  // Handle promptly, without blocking the axis loop.
}
void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}
void setup() {
    Serial.begin(115200);
    pinMode(32, INPUT_PULLUP); pinMode(33, INPUT_PULLUP);
    pinMode(34, INPUT); pinMode(35, INPUT);
    zEncoder.begin(digitalRead(32), digitalRead(33));
    spindleEncoder.begin(digitalRead(34), digitalRead(35));
    attachInterrupt(digitalPinToInterrupt(32), zISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(33), zISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(34), spindleISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(35), spindleISR, CHANGE);
    z.attachEncoder(readZ);
    axis::Pins pins;
    pins.minLimit = axis::InputPin(18); pins.maxLimit = axis::InputPin(19);
    pins.home = axis::InputPin(21); pins.emergency = axis::InputPin(22);
    z.setPins(pins);
    axis::Config config;
    config.encoder = axis::EncoderScale::rotary(2000, 5);
    config.maxSpeed = 10; config.acceleration = config.deceleration = 30; config.maxJerk = 300;
    config.home.maxTravel = 120; config.home.timeoutUs = 90000000;
    report(z.begin(config));
    spindleConfig.countsPerRevolution = 2000;  // Counts AFTER x4 decoding.
    spindleConfig.maxCountRate = 200000;       // Example plausibility bound, tune it.
    report(spindle.begin(spindleConfig, micros(), readSpindle()));
    coupling.setFaultHandler(syncFault);
}
void loop() {
    uint32_t now = micros();
    spindle.update(now, readSpindle());
    if (coupling.isActive()) coupling.tick(spindle.sample());
    else z.tick();  // Also completes local S-stop after disengagement.
    // Connect stopSpindleRequested to your spindle-stop interface here.
    if (Serial.available()) {
        int command = Serial.read();
        switch (command) {
            case 'h': report(z.home()); break;
            case 'f': case 'g': {
                axis::GearConfig config;
                config.ratio = 1;  // EncoderMaster uses turns: 1 mm/turn.
                config.mode = command == 'g' ? axis::GearMode::STRICT : axis::GearMode::LIMITED;
                config.maxPhaseError = 2; config.phaseErrorTimeUs = 100000;
                report(coupling.engage(z.syncPort(), spindle.sample(), config));
                break;
            }
            case 's': report(coupling.isActive() ? coupling.stop() : z.stop()); break;
            case 'd': coupling.disable(); z.disable(); break;
            case 'c': {
                if (!coupling.isActive() && !spindle.sample().valid)
                    report(spindle.begin(spindleConfig, micros(), readSpindle()));
                axis::Result result = coupling.clearFault();
                report(result); report(z.clearFault());
                if (result == axis::Result::OK) stopSpindleRequested = false;
                break;
            }
            case 'e': coupling.emergencyStop(); z.emergencyStop(); break;
        }
    }
}
