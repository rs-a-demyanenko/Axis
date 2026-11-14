// Behavioral PCNT model: applies configured edge/level actions to GPIO traces.
// This validates adapter semantics and cleanup, not physical ESP32 timing.
#include <AxisPcntEncoder.h>
#include <AxisQuadrature.h>
#include <assert.h>
#include <initializer_list>
#include <stdio.h>
#include <string.h>
#include <type_traits>

struct MockChannel {
    MockUnit* unit;
    pcnt_chan_config_t pins;
    pcnt_channel_edge_action_t rising, falling;
    pcnt_channel_level_action_t high, low;
};
struct MockUnit {
    bool used, enabled, running, watchLow, watchHigh;
    pcnt_unit_config_t config;
    MockChannel* channels[2];
    int raw;
    uint32_t accumulated, filter;
};
static MockUnit units[2];
static bool pins[40];
static int failAt, calls;
static esp_err_t operation() { return ++calls == failAt ? ESP_FAIL : ESP_OK; }

esp_err_t pcnt_new_unit(const pcnt_unit_config_t* config, pcnt_unit_handle_t* out) {
    if (operation() != ESP_OK) return ESP_FAIL;
    for (auto& unit : units) if (!unit.used) {
        unit = MockUnit(); unit.used = true; unit.config = *config; *out = &unit; return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}
esp_err_t pcnt_del_unit(pcnt_unit_handle_t unit) {
    assert(unit->used && !unit->enabled && !unit->running && !unit->channels[0] && !unit->channels[1]);
    unit->used = false; return ESP_OK;
}
esp_err_t pcnt_new_channel(pcnt_unit_handle_t unit, const pcnt_chan_config_t* config, pcnt_channel_handle_t* out) {
    if (operation() != ESP_OK) return ESP_FAIL;
    assert(!unit->enabled);
    for (auto& channel : unit->channels) if (!channel) {
        channel = new MockChannel(); channel->unit = unit; channel->pins = *config;
        *out = channel; return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}
esp_err_t pcnt_del_channel(pcnt_channel_handle_t channel) {
    assert(!channel->unit->enabled);
    for (auto& slot : channel->unit->channels) if (slot == channel) slot = nullptr;
    delete channel; return ESP_OK;
}
esp_err_t pcnt_channel_set_edge_action(pcnt_channel_handle_t channel, pcnt_channel_edge_action_t rising,
                                      pcnt_channel_edge_action_t falling) {
    if (operation() != ESP_OK) return ESP_FAIL;
    channel->rising = rising; channel->falling = falling; return ESP_OK;
}
esp_err_t pcnt_channel_set_level_action(pcnt_channel_handle_t channel, pcnt_channel_level_action_t high,
                                       pcnt_channel_level_action_t low) {
    if (operation() != ESP_OK) return ESP_FAIL;
    channel->high = high; channel->low = low; return ESP_OK;
}
esp_err_t pcnt_unit_set_glitch_filter(pcnt_unit_handle_t unit, const pcnt_glitch_filter_config_t* config) {
    if (operation() != ESP_OK) return ESP_FAIL;
    assert(!unit->enabled); unit->filter = config->max_glitch_ns; return ESP_OK;
}
esp_err_t pcnt_unit_add_watch_point(pcnt_unit_handle_t unit, int value) {
    if (operation() != ESP_OK) return ESP_FAIL;
    assert(!unit->enabled);
    if (value == unit->config.low_limit) unit->watchLow = true;
    if (value == unit->config.high_limit) unit->watchHigh = true;
    return ESP_OK;
}
esp_err_t pcnt_unit_enable(pcnt_unit_handle_t unit) {
    if (operation() != ESP_OK) return ESP_FAIL;
    assert(!unit->enabled); unit->enabled = true; return ESP_OK;
}
esp_err_t pcnt_unit_disable(pcnt_unit_handle_t unit) {
    assert(unit->enabled && !unit->running); unit->enabled = false; return ESP_OK;
}
esp_err_t pcnt_unit_clear_count(pcnt_unit_handle_t unit) {
    if (operation() != ESP_OK) return ESP_FAIL;
    assert(!unit->running); unit->raw = 0; unit->accumulated = 0; return ESP_OK;
}
esp_err_t pcnt_unit_start(pcnt_unit_handle_t unit) {
    if (operation() != ESP_OK) return ESP_FAIL;
    assert(unit->enabled); unit->running = true; return ESP_OK;
}
esp_err_t pcnt_unit_stop(pcnt_unit_handle_t unit) {
    assert(unit->enabled && unit->running); unit->running = false; return ESP_OK;
}
esp_err_t pcnt_unit_get_count(pcnt_unit_handle_t unit, int* value) {
    if (operation() != ESP_OK) return ESP_FAIL;
    uint32_t bits = unit->accumulated + static_cast<uint32_t>(unit->raw);
    static_assert(sizeof(bits) == sizeof(*value), "32-bit int required");
    memcpy(value, &bits, sizeof(bits)); return ESP_OK;
}

static void edge(int pin, bool level) {
    if (pins[pin] == level) return;
    pins[pin] = level;
    for (auto& unit : units) if (unit.running) {
        for (auto channel : unit.channels) if (channel && channel->pins.edge_gpio_num == pin) {
            auto action = level ? channel->rising : channel->falling;
            int delta = action == PCNT_CHANNEL_EDGE_ACTION_INCREASE ? 1 :
                        action == PCNT_CHANNEL_EDGE_ACTION_DECREASE ? -1 : 0;
            auto control = pins[channel->pins.level_gpio_num] ? channel->high : channel->low;
            if (control == PCNT_CHANNEL_LEVEL_ACTION_INVERSE) delta = -delta;
            unit.raw += delta;
            if (unit.raw == unit.config.low_limit || unit.raw == unit.config.high_limit) {
                bool watched = unit.raw < 0 ? unit.watchLow : unit.watchHigh;
                if (unit.config.flags.accum_count && watched) unit.accumulated += static_cast<uint32_t>(unit.raw);
                unit.raw = 0;
            }
        }
    }
}

int main() {
    static_assert(!std::is_copy_constructible<axis::PcntEncoder>::value, "Exclusive PCNT ownership");
    axis::PcntEncoder encoder, second, third;
    assert(encoder.begin(-1, 2) == ESP_ERR_INVALID_ARG);
    assert(encoder.begin(2, 2) == ESP_ERR_INVALID_ARG);
    assert(encoder.begin(40, 2) == ESP_ERR_INVALID_ARG);
    assert(!encoder.isInitialized());
    // All four starting phases and both directions; cross each 16-bit limit
    // repeatedly and compare every snapshot to the existing software decoder.
    const int phase[] = {0, 2, 3, 1};
    for (int initial = 0; initial < 4; ++initial) {
        pins[2] = (phase[initial] & 2) != 0; pins[3] = (phase[initial] & 1) != 0;
        axis::Quadrature reference;
        reference.begin(pins[2], pins[3]);
        assert(encoder.begin(2, 3, 1000) == ESP_OK);
        assert(units[0].filter == 1000);
        assert(encoder.begin(2, 3) == ESP_ERR_INVALID_STATE);
        assert(second.begin(4, 5) == ESP_OK);
        assert(third.begin(6, 7) == ESP_ERR_NOT_FOUND);
        int current = initial;
        for (int step = 0; step < 220000; ++step) {
            current = (current + (step < 70000 ? 1 : 3)) % 4;
            edge(2, (phase[current] & 2) != 0); edge(3, (phase[current] & 1) != 0);
            reference.update(pins[2], pins[3]);
            assert(encoder.read() == reference.readUnsafe());
            assert(second.read() == 0);
        }
        assert(axis::PcntEncoder::reader(&encoder) == reference.readUnsafe());
        // Snapshot conversion at the signed 32-bit boundary and modulo zero.
        for (uint32_t value : {0x7fffffffu, 0x80000000u, 0xffffffffu, 0u}) {
            units[0].raw = 0; units[0].accumulated = value;
            assert(encoder.read() == value);
        }
        second.end(); encoder.end(); encoder.end();
        assert(!units[0].used && !units[1].used);
    }
    // Inject an error at every driver call during initialization. Every partial
    // allocation must be reclaimed, and the same object must be retryable.
    calls = 0;
    assert(encoder.begin(2, 3, 1000) == ESP_OK);
    int stages = calls;
    encoder.end();
    for (int stage = 1; stage <= stages; ++stage) {
        calls = 0; failAt = stage;
        assert(encoder.begin(2, 3, 1000) == ESP_FAIL);
        assert(!encoder.isInitialized() && !units[0].used && !units[1].used);
        failAt = 0;
        assert(encoder.begin(2, 3) == ESP_OK);
        encoder.end();
    }
    { axis::PcntEncoder scoped; assert(scoped.begin(2, 3) == ESP_OK); }
    assert(!units[0].used && !units[1].used);
    bool failed = false;
    try { encoder.read(); } catch (const std::runtime_error&) { failed = true; }
    assert(failed);
    assert(encoder.begin(2, 3) == ESP_OK);
    failAt = calls + 1;
    failed = false;
    try { encoder.read(); } catch (const std::runtime_error&) { failed = true; }
    assert(failed);
    failAt = 0;
    puts("PASS: PCNT x4 direction, overflow, independent units and initialization failures");
}
