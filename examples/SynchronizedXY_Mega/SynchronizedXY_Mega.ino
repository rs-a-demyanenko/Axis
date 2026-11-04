// Arduino Mega 2560: four external interrupts, two feedback axes.
// Serial 115200: h home; m X+10/Y+20; s common stop; d disable;
// c clear group fault; e emergency stop. All movement is operator-triggered.
#include <Axis.h>
#include <util/atomic.h>

Axis<GM2::DIR_PWM> x(4, 5), y(6, 7);
AxisGroup<2> motion;
axis::Quadrature xEncoder, yEncoder;
void xISR() { xEncoder.update(digitalRead(2), digitalRead(3)); }
void yISR() { yEncoder.update(digitalRead(18), digitalRead(19)); }
uint32_t readEncoder(void* context) {
    uint32_t count;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { count = static_cast<axis::Quadrature*>(context)->readUnsafe(); }
    return count;
}
void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}
void setup() {
    Serial.begin(115200);
    pinMode(2, INPUT_PULLUP); pinMode(3, INPUT_PULLUP);
    pinMode(18, INPUT_PULLUP); pinMode(19, INPUT_PULLUP);
    xEncoder.begin(digitalRead(2), digitalRead(3));
    yEncoder.begin(digitalRead(18), digitalRead(19));
    attachInterrupt(digitalPinToInterrupt(2), xISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(3), xISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(18), yISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(19), yISR, CHANGE);
    x.attachEncoder(readEncoder, &xEncoder); y.attachEncoder(readEncoder, &yEncoder);
    axis::Pins xp, yp;
    xp.minLimit = axis::InputPin(22); xp.maxLimit = axis::InputPin(23); xp.home = axis::InputPin(24);
    yp.minLimit = axis::InputPin(25); yp.maxLimit = axis::InputPin(26); yp.home = axis::InputPin(27);
    xp.emergency = yp.emergency = axis::InputPin(28);
    x.setPins(xp); y.setPins(yp);
    axis::Config config;
    config.encoder = axis::EncoderScale::rotary(2000, 5);
    config.maxSpeed = 10; config.acceleration = config.deceleration = 30; config.maxJerk = 300;
    config.home.maxTravel = 120; config.home.timeoutUs = 90000000;
    report(x.begin(config));
    config.maxSpeed = 15; config.acceleration = config.deceleration = 45; config.maxJerk = 450;
    report(y.begin(config));
    report(motion.add(x.syncPort(), 1)); report(motion.add(y.syncPort(), 2));
    report(motion.setPhaseLimit(0.2));
}
void loop() {
    motion.tick();
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
