// Any Arduino: example adapter for an external encoder counter/driver status.
// Replace the callback bodies with your actual hardware reads before moving.
// As shipped encoderValid=false deliberately prevents motor operation.
#include <Axis.h>

Axis<GM2::DIR_DIR_PWM> machineAxis(4, 7, 5);

uint32_t readHardwareCounter(void*) {
    // Return a signed or unsigned incremental hardware count as uint32_t.
    // SPI/I2C reads must have a bounded execution time. No Serial or delay here.
    return 0;
}

axis::Inputs readHardwareInputs(void*) {
    axis::Inputs inputs;
    // Populate logical active states from your GPIO / expander / driver.
    inputs.encoderValid = false;  // Set true only after a successful real read.
    return inputs;
}

void setup() {
    Serial.begin(115200);
    machineAxis.attachEncoder(readHardwareCounter);
    machineAxis.attachInputs(readHardwareInputs);
    axis::Config config;
    config.encoder = axis::EncoderScale::linear(100);  // 100 counts/mm.
    Serial.println(axis::toString(machineAxis.begin(config)));
}

void loop() {
    machineAxis.tick();
    if (Serial.available() && Serial.read() == 'h') {
        Serial.println(axis::toString(machineAxis.home()));
    }
}
