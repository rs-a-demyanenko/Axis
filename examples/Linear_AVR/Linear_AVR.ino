// Arduino Uno/Nano ATmega328P. A/B encoder on both external interrupts.
// Serial 115200: h home; a absolute 20 mm; r/l relative +/-5 mm;
// s stop; d disable; c clear fault; e emergency stop;
// z zero relative position; o/O relative origin at +/-20 mm; p read positions;
// Compact command panel fits Nano's 30720-byte limit. Jog and limit-recovery
// commands are demonstrated in Linear_ESP32; all methods also exist on AVR.
#include <Axis.h>
#include <util/atomic.h>

const uint8_t ENC_A = 2, ENC_B = 3;
Axis<GM2::DIR_PWM> machineAxis(4, 5);  // DIR, PWM. Match your driver's truth table!
axis::Quadrature encoder;

void encoderISR() { encoder.update(digitalRead(ENC_A), digitalRead(ENC_B)); }

uint32_t readEncoder(void*) {
    uint32_t count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { count = encoder.readUnsafe(); }
    return count;
}

void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}

// Print supports 32-bit long on AVR; format int64_t without truncation.
void printCount(int64_t value) {
    char digits[22];
    uint8_t index = sizeof(digits) - 1;
    digits[index] = '\0';
    uint64_t magnitude = value < 0 ? static_cast<uint64_t>(-(value + 1)) + 1 : static_cast<uint64_t>(value);
    do { digits[--index] = '0' + magnitude % 10; magnitude /= 10; } while (magnitude);
    if (value < 0) digits[--index] = '-';
    Serial.print(digits + index);
}

void reportPositions() {
    if (machineAxis.isBusy()) { report(axis::Result::BUSY); return; }
    if (Serial.availableForWrite() < 60) return;
    Serial.print("H:"); Serial.print(machineAxis.isHomed());
    Serial.print(" F:"); Serial.println(static_cast<int>(machineAxis.getFault()));
    Serial.print("A:"); Serial.print(machineAxis.getAbsolutePosition(), 3);
    Serial.print(" mm "); printCount(machineAxis.getAbsolutePositionCounts()); Serial.println(" cnt");
    Serial.print("R:"); Serial.print(machineAxis.getRelativePosition(), 3);
    Serial.print(" mm "); printCount(machineAxis.getRelativePositionCounts()); Serial.println(" cnt");
}

void setup() {
    Serial.begin(115200);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    encoder.begin(digitalRead(ENC_A), digitalRead(ENC_B));
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);
    machineAxis.attachEncoder(readEncoder);

    axis::Pins pins;
    pins.minLimit = axis::InputPin(8);
    pins.maxLimit = axis::InputPin(9);
    pins.home = axis::InputPin(10);
    pins.emergency = axis::InputPin(11);
    machineAxis.setPins(pins);  // Example: switches short input to GND when active.

    axis::Config config;
    // Example: 500 PPR * x4 = 2000 counts, screw travel 5 mm per motor turn.
    config.encoder = axis::EncoderScale::rotary(2000, 5);
    config.maxSpeed = 10;
    config.acceleration = 30;
    config.deceleration = 30;
    config.maxJerk = 300;  // mm/s^3: acceleration rises/falls smoothly.
    config.velocitySetpointWeight = 0.8;  // 2-DOF PID, beta.
    config.velocityDerivativeWeight = 0;  // No derivative kick from setpoint.
    config.softMin = 0;
    config.softMax = 100;
    config.home.switchPosition = -1;
    config.home.parkPosition = 0;
    config.home.maxTravel = 120;
    config.home.timeoutUs = 90000000;
    // Starting values only: tune with YOUR motor and mechanical load.
    config.positionKp = 5;
    config.velocityKp = 0.08;
    config.velocityKi = 0.3;
    report(machineAxis.begin(config));
    // No automatic movement on power-up. Send 'h' after checking the wiring.
}

void loop() {
    machineAxis.tick();
    if (Serial.available()) {
        switch (Serial.read()) {
            case 'h': report(machineAxis.home()); break;
            case 'a': report(machineAxis.moveTo(20, 5)); break;
            case 'r': report(machineAxis.moveBy(5, 2)); break;
            case 'l': report(machineAxis.moveBy(-5, 2)); break;
            case 's': report(machineAxis.stop()); break;
            case 'd': machineAxis.disable(); break;
            case 'c': report(machineAxis.clearFault()); break;
            case 'z': report(machineAxis.setRelativePosition()); break;
            case 'o': report(machineAxis.setRelativeOrigin(20)); break;
            case 'O': report(machineAxis.setRelativeOrigin(-20)); break;
            case 'p': reportPositions(); break;
            case 'e': machineAxis.emergencyStop(); break;
        }
    }
}

