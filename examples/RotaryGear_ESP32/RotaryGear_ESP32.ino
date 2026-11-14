// Arduino-ESP32 3.x; hardware PCNT x4 encoders.
// Classic ESP32 WROOM: rotary A is master, linear Z follows 1 mm per A turn.
// h home A and Z; g engage at rest; m A+360 degrees; s stop A while staying geared;
// u disengage and stop both; c clear faults; e emergency stop both.
// Encoder A on GPIO34/35: use external pull-ups for open-collector outputs.
#include <Axis.h>
#include <AxisPcntEncoder.h>

Axis<GM2::DIR_PWM> a(14, 27), z(26, 25);
AxisGear coupling;
axis::AxisMaster master(axis::MasterFeedback::MEASURED);
axis::PcntEncoder aEncoder, zEncoder;
void stopMaster(axis::SyncError, void*) { a.emergencyStop(); }
void report(axis::Result result) {
    if (Serial.availableForWrite() >= 32) Serial.println(axis::toString(result));
}
void setup() {
    Serial.begin(115200);
    pinMode(34, INPUT); pinMode(35, INPUT);
    pinMode(32, INPUT_PULLUP); pinMode(33, INPUT_PULLUP);
    ESP_ERROR_CHECK(aEncoder.begin(34, 35));
    ESP_ERROR_CHECK(zEncoder.begin(32, 33));
    a.attachEncoder(axis::PcntEncoder::reader, &aEncoder);
    z.attachEncoder(axis::PcntEncoder::reader, &zEncoder);
    axis::Pins ap, zp;
    ap.home = axis::InputPin(23); ap.emergency = axis::InputPin(22);
    zp.minLimit = axis::InputPin(18); zp.maxLimit = axis::InputPin(19);
    zp.home = axis::InputPin(21); zp.emergency = axis::InputPin(22);
    a.setPins(ap); z.setPins(zp);
    axis::Config rotation;
    rotation.kind = axis::Kind::ROTARY; rotation.continuous = true; rotation.softLimitsEnabled = false;
    rotation.encoder = axis::EncoderScale::rotary(1440);  // On output shaft, 4 counts/degree.
    rotation.maxSpeed = 90; rotation.acceleration = rotation.deceleration = 180; rotation.maxJerk = 1800;
    rotation.positionTolerance = rotation.positionHysteresis = 0.5; rotation.velocityTolerance = 1;
    rotation.velocityKp = 0.005; rotation.velocityKi = 0.02; rotation.positionKp = 5;
    rotation.maxFollowingError = 20;
    rotation.home.searchSpeed = 30; rotation.home.latchSpeed = 5; rotation.home.backoff = 5;
    rotation.home.switchPosition = -5; rotation.home.maxTravel = 800; rotation.home.timeoutUs = 60000000;
    report(a.begin(rotation));
    axis::Config linear;
    linear.encoder = axis::EncoderScale::rotary(2000, 5);
    linear.maxSpeed = 10; linear.acceleration = linear.deceleration = 30; linear.maxJerk = 300;
    linear.home.maxTravel = 120; linear.home.timeoutUs = 90000000;
    report(z.begin(linear));
    coupling.setFaultHandler(stopMaster);
}
void loop() {
    a.tick();
    const axis::MasterSample& sample = master.update(a.controlCore());
    if (coupling.isActive()) coupling.tick(sample);
    else z.tick();
    if (!Serial.available()) return;
    switch (Serial.read()) {
        case 'h': report(a.home()); report(z.home()); break;
        case 'g': {
            axis::GearConfig config;
            config.ratio = 1.0 / 360.0;  // 1 mm for 360 degrees, not for one degree.
            config.maxPhaseError = 0.2;
            report(coupling.engage(z.syncPort(), sample, config));
            break;
        }
        case 'm': if (coupling.isActive()) report(a.moveBy(360, 60)); break;
        case 's': report(a.stop()); break;  // Z stays phase-coupled through A's deceleration.
        case 'u': report(a.stop()); report(coupling.stop()); break;
        case 'c': report(a.clearFault()); report(coupling.clearFault()); report(z.clearFault()); break;
        case 'e': a.emergencyStop(); coupling.emergencyStop(); z.emergencyStop(); break;
    }
}
