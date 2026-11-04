// No motor or encoder needed. Open Arduino Serial Plotter at 115200 baud.
// Shows reference position p, velocity v, acceleration a and jerk j.
// Reset the board to replay the move; after arrival the endpoint is held.
#include <AxisProfile.h>

axis::SCurveProfile trajectory;
uint32_t startedUs, lastReportUs;
bool ready = false;

void setup() {
    Serial.begin(115200);
    // 100 mm, 20 mm/s, 40 mm/s^2 accel/decel, 200 mm/s^3 jerk.
    ready = trajectory.planPosition(100, 20, 40, 40, 200);
    startedUs = lastReportUs = micros();
}

void loop() {
    uint32_t now = micros();
    if (!ready || now - lastReportUs < 20000 || Serial.availableForWrite() < 60) return;
    lastReportUs = now;
    double seconds = static_cast<uint32_t>(now - startedUs) * 1e-6;
    axis::MotionState state = trajectory.sample(seconds);
    Serial.print("p:"); Serial.print(state.position, 3);
    Serial.print(" v:"); Serial.print(state.velocity, 3);
    Serial.print(" a:"); Serial.print(state.acceleration, 3);
    Serial.print(" j:"); Serial.println(state.jerk, 3);
}
