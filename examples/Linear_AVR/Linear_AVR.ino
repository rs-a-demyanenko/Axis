// Arduino Uno/Nano ATmega328P. A/B encoder on both external interrupts.
// Serial 115200: h home; a absolute 20 mm; r/l relative +/-5 mm;
// j jog +2 mm/s; s stop; d disable; b brake; c clear fault;
// z set current work coordinate to zero; w move to work +5 mm;
// + recover from MIN; - recover from MAX; e emergency stop.
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
            case 'j': report(machineAxis.jog(2)); break;
            case 's': report(machineAxis.stop()); break;
            case 'd': machineAxis.disable(); break;
            case 'b': machineAxis.brake(); break;
            case 'c': report(machineAxis.clearFault()); break;
            case 'z': report(machineAxis.setWorkPosition(0)); break;
            case 'w': report(machineAxis.moveTo(5, 2, axis::Frame::WORK)); break;
            case '+': report(machineAxis.recoverFromLimit(1, 2)); break;
            case '-': report(machineAxis.recoverFromLimit(-1, 2)); break;
            case 'e': machineAxis.emergencyStop(); break;
        }
    }
    static uint32_t lastReport = 0;
    if (millis() - lastReport >= 200 && Serial.availableForWrite() >= 60) {
        lastReport = millis();
        Serial.print("P:"); Serial.print(machineAxis.getPosition(), 2);
        Serial.print(" V:"); Serial.print(machineAxis.getVelocity(), 2);
        Serial.print(" M:"); Serial.print(static_cast<int>(machineAxis.getMode()));
        Serial.print(" F:"); Serial.println(static_cast<int>(machineAxis.getFault()));
    }
}

