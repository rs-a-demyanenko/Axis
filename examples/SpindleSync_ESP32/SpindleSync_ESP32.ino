// Arduino-ESP32 3.x; hardware PCNT x4 encoders.
// Classic ESP32: Z axis + external spindle encoder, 1 mm per revolution.
// GPIO34/35: external 3.3 V pull-ups if needed; no internal pull-ups available.
// h home Z; f LIMITED (can engage running spindle); g STRICT (engage at rest);
// s disengage and S-stop Z; d disable Z; c clear fault; e emergency stop Z.
// This example does not control spindle power. Use the fault callback below
// to request a spindle stop through YOUR hardware/application interface.
#include <Axis.h>
#include <AxisPcntEncoder.h>

Axis<GM2::DIR_PWM> z(26, 25);
AxisGear coupling;
axis::PcntEncoder zEncoder, spindleEncoder;
axis::EncoderMaster spindle;
axis::EncoderMasterConfig spindleConfig;
volatile bool stopSpindleRequested = false;
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
    ESP_ERROR_CHECK(zEncoder.begin(32, 33));
    ESP_ERROR_CHECK(spindleEncoder.begin(34, 35));
    z.attachEncoder(axis::PcntEncoder::reader, &zEncoder);
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
    report(spindle.begin(spindleConfig, micros(), spindleEncoder.read()));
    coupling.setFaultHandler(syncFault);
}
void loop() {
    uint32_t now = micros();
    spindle.update(now, spindleEncoder.read());
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
                    report(spindle.begin(spindleConfig, micros(), spindleEncoder.read()));
                axis::Result result = coupling.clearFault();
                report(result); report(z.clearFault());
                if (result == axis::Result::OK) stopSpindleRequested = false;
                break;
            }
            case 'e': coupling.emergencyStop(); z.emergencyStop(); break;
        }
    }
}
