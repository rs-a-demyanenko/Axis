// Uno/Nano or ESP32/S3: STEP/DIR driver with active-low EN; no encoder.
// Serial 115200: h home, r/l relative +/-1 mm, a absolute 10 mm,
// s controlled stop, d disable, c clear fault. Commands never start at boot.
#include <AxisStepper.h>

AxisStepper<STEPPER2WIRE> machineAxis(5, 6, 7);  // STEP, DIR, EN.

void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}

void setup() {
    Serial.begin(115200);
    // 200 steps/rev * 16 microsteps / 5 mm screw pitch = 640 pulses/mm.
    machineAxis.setStepsPerUnit(640);
    axis::Pins pins;
    pins.home = axis::InputPin(10);
    pins.minLimit = axis::InputPin(8);
    pins.maxLimit = axis::InputPin(9);
    pins.emergency = axis::InputPin(11);
    machineAxis.setPins(pins);  // Switches to GND, internal pullups.
    axis::Config config;
    config.maxSpeed = 2;
    config.acceleration = config.deceleration = 10;
    config.maxJerk = 100;
    config.home.searchSpeed = 1;
    config.home.latchSpeed = 0.2;
    config.unhomedMaxSpeed = 1;
    // config.encoder is replaced with the pulse scale when no encoder is attached.
    report(machineAxis.begin(config));
}

void loop() {
    machineAxis.tick();  // Call much faster than maxSpeed * stepsPerUnit.
    if (!Serial.available()) return;
    switch (Serial.read()) {
        case 'h': report(machineAxis.home()); break;
        case 'r': report(machineAxis.moveBy(1, 1)); break;
        case 'l': report(machineAxis.moveBy(-1, 1)); break;
        case 'a': report(machineAxis.moveTo(10, 2)); break;
        case 's': report(machineAxis.stop()); break;
        case 'd': machineAxis.disable(); break;
        case 'c': report(machineAxis.clearFault()); break;
    }
}
