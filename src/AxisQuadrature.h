#pragma once

#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp_attr.h>
#define AXIS_QUADRATURE_ISR_ATTR IRAM_ATTR
#else
#define AXIS_QUADRATURE_ISR_ATTR
#endif

namespace axis {

// Portable x4 Gray-code decoder. No GPIO, interrupt registration or implicit
// interrupt enabling. Caller supplies platform-specific atomic access.
class Quadrature {
 public:
    void begin(bool a, bool b, uint32_t count = 0) {
        _previous = static_cast<uint8_t>((a ? 2 : 0) | (b ? 1 : 0));
        _count = count;
        _invalidTransitions = 0;
    }
    // Call on CHANGE of BOTH channels, with ISR-safe reads. A leads B positive.
    // No flash-resident lookup table: suitable for an ESP32 IRAM ISR wrapper.
    void AXIS_QUADRATURE_ISR_ATTR update(bool a, bool b) {
        uint8_t next = static_cast<uint8_t>((a ? 2 : 0) | (b ? 1 : 0));
        uint8_t previous = _previous;
        _previous = next;
        if (next == previous) return;
        if ((next ^ previous) == 3) { _invalidTransitions = _invalidTransitions + 1; return; }
        if (((next >> 1) ^ (previous & 1)) != 0) _count = _count + 1;
        else _count = _count - 1;
    }
    // Read only while the ISR is excluded (ATOMIC_BLOCK on AVR; portMUX on ESP32).
    uint32_t readUnsafe() const { return _count; }
    uint32_t invalidTransitionsUnsafe() const { return _invalidTransitions; }

 private:
    volatile uint32_t _count = 0, _invalidTransitions = 0;
    uint8_t _previous = 0;
};

}  // namespace axis

#undef AXIS_QUADRATURE_ISR_ATTR
