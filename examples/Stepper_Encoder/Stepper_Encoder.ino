// Uno/Nano or ESP32/S3: STEP/DIR with a quadrature encoder.
// Verify every GPIO against your module. Serial: h home; r/l +/-1 mm; s stop.
#include <AxisStepper.h>
#if defined(ARDUINO_ARCH_AVR)
#include <util/atomic.h>
#endif

const uint8_t ENC_A = 2, ENC_B = 3;
AxisStepper<STEPPER2WIRE> machineAxis(5, 6, 7);
axis::Quadrature encoder;

#if defined(ARDUINO_ARCH_ESP32)
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
void ARDUINO_ISR_ATTR encoderISR() {
    portENTER_CRITICAL_ISR(&encoderMux);
    encoder.update(digitalRead(ENC_A), digitalRead(ENC_B));
    portEXIT_CRITICAL_ISR(&encoderMux);
}
#else
void encoderISR() { encoder.update(digitalRead(ENC_A), digitalRead(ENC_B)); }
#endif

uint32_t readEncoder(void*) {
    uint32_t count;
#if defined(ARDUINO_ARCH_ESP32)
    portENTER_CRITICAL(&encoderMux);
    count = encoder.readUnsafe();
    portEXIT_CRITICAL(&encoderMux);
#elif defined(ARDUINO_ARCH_AVR)
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { count = encoder.readUnsafe(); }
#else
    // Called only from loop(), with interrupts enabled on entry.
    noInterrupts(); count = encoder.readUnsafe(); interrupts();
#endif
    return count;
}

void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}

void setup() {
    Serial.begin(115200);
    pinMode(ENC_A, INPUT_PULLUP); pinMode(ENC_B, INPUT_PULLUP);
    encoder.begin(digitalRead(ENC_A), digitalRead(ENC_B));
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);
    machineAxis.attachEncoder(readEncoder);
    machineAxis.setStepsPerUnit(640);  // 200 * 16 microsteps / 5 mm.
    axis::Pins pins;
    pins.home = axis::InputPin(10);
    pins.minLimit = axis::InputPin(8); pins.maxLimit = axis::InputPin(9);
    pins.emergency = axis::InputPin(11);
    machineAxis.setPins(pins);
    axis::Config config;
    config.encoder = axis::EncoderScale::rotary(2000, 5);  // 500 PPR * x4 / 5 mm.
    config.maxSpeed = 2;
    config.acceleration = config.deceleration = 10;
    config.maxJerk = 100;
    config.home.searchSpeed = 1;
    config.home.latchSpeed = 0.2;
    config.unhomedMaxSpeed = 1;
    config.maxFollowingError = 0.5;
    report(machineAxis.begin(config));
}

void loop() {
    machineAxis.tick();
    if (!Serial.available()) return;
    switch (Serial.read()) {
        case 'h': report(machineAxis.home()); break;
        case 'r': report(machineAxis.moveBy(1, 1)); break;
        case 'l': report(machineAxis.moveBy(-1, 1)); break;
        case 's': report(machineAxis.stop()); break;
        case 'd': machineAxis.disable(); break;
        case 'c': report(machineAxis.clearFault()); break;
    }
}
