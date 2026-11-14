#pragma once

#include <stdint.h>

// The modern PCNT driver is available in Arduino-ESP32 3.x (ESP-IDF 5+).
// Include on any platform to query AXIS_HAS_PCNT without pulling in ESP headers.
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_idf_version.h>
#include <soc/soc_caps.h>
#if ESP_IDF_VERSION_MAJOR >= 5 && SOC_PCNT_SUPPORTED
#define AXIS_HAS_PCNT 1
#endif
#endif
#ifndef AXIS_HAS_PCNT
#define AXIS_HAS_PCNT 0
#endif

#if AXIS_HAS_PCNT
#include <driver/gpio.h>
#include <driver/pulse_cnt.h>
#include <esp_err.h>

namespace axis {

// Hardware x4 A/B decoder. One unit and two channels per encoder.
// A leads B positive, matching Quadrature. All lifecycle calls belong to one
// task; read() is an atomic driver snapshot. No user GPIO ISR is required.
class PcntEncoder {
 public:
    PcntEncoder() {}
    ~PcntEncoder() { end(); }
    PcntEncoder(const PcntEncoder&) = delete;
    PcntEncoder& operator=(const PcntEncoder&) = delete;

    // Configure GPIO input modes/pull-ups BEFORE begin(). Driver allocation
    // routes these inputs to PCNT. 0 disables the optional glitch filter.
    // begin() starts at zero; end()/begin() invalidates any axis reference.
    esp_err_t begin(int pinA, int pinB, uint32_t maxGlitchNs = 0) {
        if (_unit) return ESP_ERR_INVALID_STATE;
        if (!GPIO_IS_VALID_GPIO(pinA) || !GPIO_IS_VALID_GPIO(pinB) || pinA == pinB)
            return ESP_ERR_INVALID_ARG;
        pcnt_unit_config_t config = {};
        config.low_limit = -32767;
        config.high_limit = 32767;
        config.flags.accum_count = 1;
        esp_err_t error = pcnt_new_unit(&config, &_unit);
        if (error != ESP_OK) return error;
        error = configure(pinA, pinB, maxGlitchNs);
        if (error != ESP_OK) end();
        return error;
    }

    void end() {
        if (!_unit) return;
        if (_started) pcnt_unit_stop(_unit);
        if (_enabled) pcnt_unit_disable(_unit);
        if (_channels[1]) pcnt_del_channel(_channels[1]);
        if (_channels[0]) pcnt_del_channel(_channels[0]);
        pcnt_del_unit(_unit);
        _unit = nullptr;
        _channels[0] = _channels[1] = nullptr;
        _enabled = _started = false;
    }

    bool isInitialized() const { return _started; }

    // Modulo-2^32 snapshot for Axis/AxisStepper/EncoderMaster. Fail explicitly
    // on misuse or driver errors instead of returning a stale position.
    uint32_t read() const {
        int count = 0;
        ESP_ERROR_CHECK(_started ? pcnt_unit_get_count(_unit, &count) : ESP_ERR_INVALID_STATE);
        return static_cast<uint32_t>(count);
    }

    // machineAxis.attachEncoder(axis::PcntEncoder::reader, &encoder);
    static uint32_t reader(void* context) {
        ESP_ERROR_CHECK(context ? ESP_OK : ESP_ERR_INVALID_ARG);
        return static_cast<PcntEncoder*>(context)->read();
    }

 private:
    pcnt_unit_handle_t _unit = nullptr;
    pcnt_channel_handle_t _channels[2] = {nullptr, nullptr};
    bool _enabled = false, _started = false;

    esp_err_t configure(int pinA, int pinB, uint32_t maxGlitchNs) {
        if (maxGlitchNs) {
            pcnt_glitch_filter_config_t filter = {};
            filter.max_glitch_ns = maxGlitchNs;
            esp_err_t error = pcnt_unit_set_glitch_filter(_unit, &filter);
            if (error != ESP_OK) return error;
        }
        for (int index = 0; index < 2; ++index) {
            pcnt_chan_config_t channel = {};
            channel.edge_gpio_num = index == 0 ? pinA : pinB;
            channel.level_gpio_num = index == 0 ? pinB : pinA;
            esp_err_t error = pcnt_new_channel(_unit, &channel, &_channels[index]);
            if (error != ESP_OK) return error;
            error = pcnt_channel_set_edge_action(_channels[index],
                index == 0 ? PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                index == 0 ? PCNT_CHANNEL_EDGE_ACTION_DECREASE : PCNT_CHANNEL_EDGE_ACTION_INCREASE);
            if (error != ESP_OK) return error;
            // Arguments are HIGH-level action, LOW-level action.
            error = pcnt_channel_set_level_action(_channels[index],
                PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP);
            if (error != ESP_OK) return error;
        }
        // Both watch points are required for the driver's overflow accumulator.
        esp_err_t error = pcnt_unit_add_watch_point(_unit, -32767);
        if (error != ESP_OK) return error;
        error = pcnt_unit_add_watch_point(_unit, 32767);
        if (error != ESP_OK) return error;
        error = pcnt_unit_enable(_unit);
        if (error != ESP_OK) return error;
        _enabled = true;
        error = pcnt_unit_clear_count(_unit);
        if (error != ESP_OK) return error;
        error = pcnt_unit_start(_unit);
        if (error == ESP_OK) _started = true;
        return error;
    }
};

}  // namespace axis
#endif
