// Arduino-ESP32 3.x; hardware PCNT x4 encoder.
// Classic ESP32; continuous rotary axis, A/B encoder on the OUTPUT shaft.
// Serial 115200: h home; a shortest path to 10 degrees;
// p positive path to 350 degrees; n negative path to 10 degrees;
// r two full turns; s stop; d disable; c clear fault.
#include <Axis.h>
#include <AxisPcntEncoder.h>

const uint8_t ENC_A = 32, ENC_B = 33;
Axis<GM2::PWM_PWM_SPEED> rotaryAxis(25, 26);
axis::PcntEncoder encoder;

void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}

void setup() {
    Serial.begin(115200);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    ESP_ERROR_CHECK(encoder.begin(ENC_A, ENC_B));
    rotaryAxis.attachEncoder(axis::PcntEncoder::reader, &encoder);
    axis::Pins pins;
    pins.home = axis::InputPin(21);
    pins.emergency = axis::InputPin(22);
    rotaryAxis.setPins(pins);

    axis::Config config;
    config.kind = axis::Kind::ROTARY;
    config.continuous = true;
    config.softLimitsEnabled = false;  // Only for a mechanism permitting unlimited turns.
    config.encoder = axis::EncoderScale::rotary(1440);  // x4 counts/output revolution.
    config.maxSpeed = 90;
    config.acceleration = config.deceleration = 180;
    config.maxJerk = 1800;  // degrees/s^3.
    config.velocitySetpointWeight = 0.8;
    config.velocityDerivativeWeight = 0;
    config.positionTolerance = 0.5;    // One count = 0.25 degrees.
    config.velocityTolerance = 1;
    config.positionHysteresis = 0.5;
    config.positionKp = 5;
    config.velocityKp = 0.005;
    config.velocityKi = 0.02;
    config.maxFollowingError = 20;
    config.home.searchSpeed = 30;
    config.home.latchSpeed = 5;
    config.home.backoff = 5;
    config.home.switchPosition = -5;
    config.home.parkPosition = 0;
    config.home.maxTravel = 800;       // Search + backoff + latch + parking.
    config.home.timeoutUs = 60000000;
    report(rotaryAxis.begin(config));
}

void loop() {
    rotaryAxis.tick();
    if (!Serial.available()) return;
    switch (Serial.read()) {
        case 'h': report(rotaryAxis.home()); break;
        case 'a': report(rotaryAxis.moveToAngle(10)); break;
        case 'p': report(rotaryAxis.moveToAngle(350, axis::Direction::POSITIVE)); break;
        case 'n': report(rotaryAxis.moveToAngle(10, axis::Direction::NEGATIVE)); break;
        case 'r': report(rotaryAxis.moveBy(720, 60)); break;
        case 's': report(rotaryAxis.stop()); break;
        case 'd': rotaryAxis.disable(); break;
        case 'c': report(rotaryAxis.clearFault()); break;
        case 'e': rotaryAxis.emergencyStop(); break;
    }
}

