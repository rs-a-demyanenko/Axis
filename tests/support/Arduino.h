#pragma once
#include <stdint.h>

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
#define CHANGE 2
#define constrain(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

extern uint32_t mockMicros;
extern int mockDigital[256], mockPwm[256], mockPinModes[256];
inline unsigned long micros() { return mockMicros; }
inline unsigned long millis() { return mockMicros / 1000; }
inline void pinMode(uint8_t pin, int mode) { mockPinModes[pin] = mode; }
inline void digitalWrite(uint8_t pin, int value) { mockDigital[pin] = value; }
inline int digitalRead(uint8_t pin) { return mockDigital[pin]; }
inline void analogWrite(uint8_t pin, int value) { mockPwm[pin] = value; }
inline void delayMicroseconds(unsigned int value) { mockMicros += value; }

