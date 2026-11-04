// Classic ESP32 Dev Module. Arduino-ESP32 2.x / 3.x analogWrite API.
// Other ESP32 variants: adapt ALL GPIO assignments to your module.
// Serial 115200: h home; a absolute 20 mm; r/l relative +/-5 mm;
// j jog; s stop; d disable; c clear fault; + / - limit recovery; e emergency.
// z zero relative position; o/O relative origin at +/-20 mm; p read positions.
#include <Axis.h>

const uint8_t ENC_A = 32, ENC_B = 33;
Axis<GM2::DIR_PWM> machineAxis(26, 25);
axis::Quadrature encoder;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

void ARDUINO_ISR_ATTR encoderISR() {
    portENTER_CRITICAL_ISR(&encoderMux);
    encoder.update(digitalRead(ENC_A), digitalRead(ENC_B));
    portEXIT_CRITICAL_ISR(&encoderMux);
}

uint32_t readEncoder(void*) {
    portENTER_CRITICAL(&encoderMux);
    uint32_t count = encoder.readUnsafe();
    portEXIT_CRITICAL(&encoderMux);
    return count;
}

void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}

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
    Serial.print("H:"); Serial.println(machineAxis.isHomed());
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
    pins.minLimit = axis::InputPin(18);
    pins.maxLimit = axis::InputPin(19);
    pins.home = axis::InputPin(21);
    pins.emergency = axis::InputPin(22);
    machineAxis.setPins(pins);

    axis::Config config;
    config.encoder = axis::EncoderScale::rotary(2000, 5);
    config.maxSpeed = 10;
    config.acceleration = config.deceleration = 30;
    config.maxJerk = 300;
    config.velocitySetpointWeight = 0.8;
    config.velocityDerivativeWeight = 0;
    config.home.switchPosition = -1;
    config.home.parkPosition = 0;
    config.home.maxTravel = 120;
    config.home.timeoutUs = 90000000;
    config.positionKp = 5;
    config.velocityKp = 0.08;
    config.velocityKi = 0.3;
    report(machineAxis.begin(config));
    // Axis does not own a task. Call every Axis method from this loop/task only.
}

void loop() {
    machineAxis.tick();
    if (Serial.available()) {
        switch (Serial.read()) {
            case 'h': report(machineAxis.home()); break;
            case 'a': report(machineAxis.moveTo(20, 5)); break;
            case 'r': report(machineAxis.moveBy(5, 2)); break;
            case 'l': report(machineAxis.moveBy(-5, 2)); break;
            case 'j': report(machineAxis.jog(2)); break;
            case 's': report(machineAxis.stop()); break;
            case 'd': machineAxis.disable(); break;
            case 'c': report(machineAxis.clearFault()); break;
            case 'z': report(machineAxis.setRelativePosition()); break;
            case 'o': report(machineAxis.setRelativeOrigin(20)); break;
            case 'O': report(machineAxis.setRelativeOrigin(-20)); break;
            case 'p': reportPositions(); break;
            case '+': report(machineAxis.recoverFromLimit(1, 2)); break;
            case '-': report(machineAxis.recoverFromLimit(-1, 2)); break;
            case 'e': machineAxis.emergencyStop(); break;
        }
    }
    static uint32_t lastReport = 0;
    if (millis() - lastReport >= 200 && Serial.availableForWrite() >= 60) {
        lastReport = millis();
        Serial.print("A:"); Serial.print(machineAxis.getAbsolutePosition(), 2);
        Serial.print(" R:"); Serial.print(machineAxis.getRelativePosition(), 2);
        Serial.print(" V:"); Serial.print(machineAxis.getVelocity(), 2);
        Serial.print(" M:"); Serial.print(static_cast<int>(machineAxis.getMode()));
        Serial.print(" F:"); Serial.println(static_cast<int>(machineAxis.getFault()));
    }
}

