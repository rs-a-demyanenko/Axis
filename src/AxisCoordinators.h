#pragma once

#include <Arduino.h>
#include "AxisSync.h"

template <uint8_t Capacity>
class AxisGroup : public axis::Group<Capacity> {
 public:
    axis::Result moveBy(double distance, double speed = 0.0) {
        return this->moveByAt(static_cast<uint32_t>(micros()), distance, speed);
    }
    axis::Result moveTo(double firstAxisPosition, double speed = 0.0) {
        return this->moveToAt(static_cast<uint32_t>(micros()), firstAxisPosition, speed);
    }
    bool tick() { return this->tickAt(static_cast<uint32_t>(micros())); }
    axis::Result clearFault() { return this->clearFaultAt(static_cast<uint32_t>(micros())); }
};

class AxisGear : public axis::Gear {
 public:
    axis::Result engage(axis::AxisPort follower, const axis::MasterSample& master, const axis::GearConfig& config) {
        return engageAt(static_cast<uint32_t>(micros()), follower, master, config);
    }
    bool tick(const axis::MasterSample& master) { return tickAt(static_cast<uint32_t>(micros()), master); }
    axis::Result clearFault() { return clearFaultAt(static_cast<uint32_t>(micros())); }
};

