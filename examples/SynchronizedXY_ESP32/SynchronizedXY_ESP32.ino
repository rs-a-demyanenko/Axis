// Classic ESP32 WROOM. X:Y = 1:2, one common S-curve.
// Serial 115200: h home both axes; m move X+10/Y+20; s common stop;
// d disable; c clear group faults; e emergency stop.
// Encoder Y GPIO34/35 require EXTERNAL 3.3 V pull-ups if open collector.
// GPIO16/17 may be reserved on modules with PSRAM; adapt all pins to your board.
#include <Axis.h>

Axis<GM2::DIR_PWM> x(26, 25), y(14, 27);
AxisGroup<2> motion;
axis::Quadrature xEncoder, yEncoder;
portMUX_TYPE xMux = portMUX_INITIALIZER_UNLOCKED, yMux = portMUX_INITIALIZER_UNLOCKED;

void ARDUINO_ISR_ATTR xISR() {
    portENTER_CRITICAL_ISR(&xMux);
    xEncoder.update(digitalRead(32), digitalRead(33));
    portEXIT_CRITICAL_ISR(&xMux);
}
void ARDUINO_ISR_ATTR yISR() {
    portENTER_CRITICAL_ISR(&yMux);
    yEncoder.update(digitalRead(34), digitalRead(35));
    portEXIT_CRITICAL_ISR(&yMux);
}
uint32_t readX(void*) {
    portENTER_CRITICAL(&xMux); uint32_t count = xEncoder.readUnsafe(); portEXIT_CRITICAL(&xMux);
    return count;
}
uint32_t readY(void*) {
    portENTER_CRITICAL(&yMux); uint32_t count = yEncoder.readUnsafe(); portEXIT_CRITICAL(&yMux);
    return count;
}
void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}

void setup() {
    Serial.begin(115200);
    pinMode(32, INPUT_PULLUP); pinMode(33, INPUT_PULLUP);
    pinMode(34, INPUT); pinMode(35, INPUT);
    xEncoder.begin(digitalRead(32), digitalRead(33));
    yEncoder.begin(digitalRead(34), digitalRead(35));
    attachInterrupt(digitalPinToInterrupt(32), xISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(33), xISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(34), yISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(35), yISR, CHANGE);
    x.attachEncoder(readX); y.attachEncoder(readY);
    axis::Pins xp, yp;
    xp.minLimit = axis::InputPin(18); xp.maxLimit = axis::InputPin(19); xp.home = axis::InputPin(21);
    yp.minLimit = axis::InputPin(16); yp.maxLimit = axis::InputPin(17); yp.home = axis::InputPin(23);
    xp.emergency = yp.emergency = axis::InputPin(22);
    x.setPins(xp); y.setPins(yp);
    axis::Config config;
    config.encoder = axis::EncoderScale::rotary(2000, 5);
    config.maxSpeed = 10; config.acceleration = config.deceleration = 30; config.maxJerk = 300;
    config.home.maxTravel = 120; config.home.timeoutUs = 90000000;
    report(x.begin(config));
    config.maxSpeed = 15; config.acceleration = config.deceleration = 45; config.maxJerk = 450;
    report(y.begin(config));
    report(motion.add(x.syncPort(), 1));
    report(motion.add(y.syncPort(), 2));
    // Common limit: min(10, 15/2) = 7.5 X-mm/s. No independent follower ramp.
    report(motion.setPhaseLimit(0.2, 100000));  // Normalized contour error, in X-mm.
}

void loop() {
    motion.tick();  // Also polls independent homing/holding while group is idle.
    if (!Serial.available()) return;
    switch (Serial.read()) {
        case 'h': report(x.home()); report(y.home()); break;
        case 'm': report(motion.moveBy(10, 5)); break;
        case 's': report(motion.stop()); break;
        case 'd': motion.disable(); break;
        case 'c': report(motion.clearFault()); break;
        case 'e': motion.emergencyStop(); break;
    }
}
