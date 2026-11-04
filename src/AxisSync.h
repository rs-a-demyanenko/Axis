#pragma once

#include "AxisCore.h"

namespace axis {

enum class SyncError : uint8_t {
    NONE, AXIS_FAULT, SOURCE_INVALID, SOURCE_TIMEOUT, SOURCE_CHANGED,
    REFERENCE_LIMIT, PHASE_ERROR, INTERRUPTED, NUMERIC_ERROR
};
enum class GearMode : uint8_t { STRICT, LIMITED };
enum class MasterFeedback : uint8_t { REFERENCE, MEASURED };
const char* toString(SyncError error);

// Created by Axis::syncPort(). POD callbacks avoid a vtable or extra state in
// ordinary single-axis instances. Hardware outputs are committed separately.
struct AxisPort {
    Core* core = nullptr;
    void* context = nullptr;
    bool (*read)(void*, uint32_t&, Inputs&) = nullptr;
    void (*apply)(void*) = nullptr;
    bool valid() const { return core && read && apply; }
    bool update(uint32_t now, const ExternalReference* reference = nullptr) const;
    void commit() const { if (apply) apply(context); }
};

struct MasterSample {
    MotionState state;
    uint32_t timestampUs = 0, epoch = 0;
    bool valid = false;
    bool ready = false;  // At least one kinematic measurement is available.
    const void* identity = nullptr;
    const Core* axis = nullptr;
    bool countsAvailable = false;
    int64_t counts = 0;
    double countsPerUnit = 1.0;
};

struct EncoderMasterConfig {
    double countsPerRevolution = 2000.0;
    bool inverted = false;
    uint32_t periodUs = 5000, maxGapUs = 50000;
    double filterTime = 0.02;
    double maxCountRate = 0.0;  // Zero disables the plausibility diagnostic.
};

// A motor driver is not needed: incremental spindle encoder -> unwrapped turns.
class EncoderMaster {
 public:
    EncoderMaster() {}
    EncoderMaster(const EncoderMaster&) = delete;
    EncoderMaster& operator=(const EncoderMaster&) = delete;
    Result begin(const EncoderMasterConfig& config, uint32_t now, uint32_t raw);
    bool update(uint32_t now, uint32_t raw, bool encoderValid = true);
    const MasterSample& sample() const { return _sample; }
 private:
    EncoderMasterConfig _config;
    MasterSample _sample;
    uint32_t _lastRaw = 0, _lastUs = 0;
    int64_t _counts = 0, _sampleCounts = 0;
    bool _initialized = false;
};

// Controlled axis as master. REFERENCE is noiseless planned p/v/a/j; MEASURED
// follows actual encoder position with filtered estimates of acceleration/jerk.
class AxisMaster {
 public:
    explicit AxisMaster(MasterFeedback feedback = MasterFeedback::REFERENCE, double filterTime = 0.02)
        : _feedback(feedback), _filterTime(filterTime) {}
    AxisMaster(const AxisMaster&) = delete;
    AxisMaster& operator=(const AxisMaster&) = delete;
    const MasterSample& update(const Core& core);
    const MasterSample& sample() const { return _sample; }
 private:
    MasterFeedback _feedback;
    double _filterTime;
    MasterSample _sample;
};

struct GearConfig {
    double ratio = 1.0;               // Follower units per master unit; negative reverses motion.
    GearMode mode = GearMode::STRICT;
    double maxPhaseError = 2.0;       // Positive, follower physical units.
    uint32_t phaseErrorTimeUs = 100000, sourceTimeoutUs = 50000;
    double captureGain = 2.0;         // LIMITED: catch-up velocity per unit of phase error.
};

struct GearSnapshot {
    bool active = false, inSync = false;
    SyncError error = SyncError::NONE;
    double idealPosition = 0.0, phaseError = 0.0, referenceLag = 0.0;
    MotionState reference;
};

class Gear {
 public:
    Gear() {}
    Gear(const Gear&) = delete;
    Gear& operator=(const Gear&) = delete;
    Result engageAt(uint32_t now, AxisPort follower, const MasterSample& master, const GearConfig& config);
    bool tickAt(uint32_t now, const MasterSample& master);
    Result stop();              // Explicitly disengage, then local S-stop; phase relation ends.
    void disable();
    void emergencyStop();
    Result clearFaultAt(uint32_t now);
    bool isActive() const { return _status.active; }
    GearSnapshot snapshot() const { return _status; }
    void setFaultHandler(void (*handler)(SyncError, void*), void* context = nullptr) {
        _handler = handler; _handlerContext = context;
    }
 private:
    AxisPort _follower;
    GearConfig _config;
    GearSnapshot _status;
    const void* _masterIdentity = nullptr;
    const Core* _masterAxis = nullptr;
    uint32_t _masterEpoch = 0, _lastUs = 0, _phaseErrorUs = 0;
    int64_t _masterCounts = 0;
    double _masterPosition = 0.0, _countsPerUnit = 1.0, _followerPosition = 0.0;
    bool _useCounts = false;
    SCurveProfile _ramp;
    void (*_handler)(SyncError, void*) = nullptr;
    void* _handlerContext = nullptr;
    void fail(SyncError error);
};

// Fixed-size coordinator. AxisGroup (in Axis.h) adds Arduino micros() wrappers.
// All members share a scalar S-curve and a common control cycle.
template <uint8_t Capacity>
class Group {
 public:
    Group() { static_assert(Capacity > 0, "Group must hold at least one axis"); }
    Group(const Group&) = delete;
    Group& operator=(const Group&) = delete;
    Result add(AxisPort port, double ratio = 1.0);
    Result setRatio(uint8_t member, double ratio);
    Result moveByAt(uint32_t now, double distance, double speed = 0.0);
    Result moveToAt(uint32_t now, double firstAxisPosition, double speed = 0.0);
    bool tickAt(uint32_t now);
    Result stop();
    void disable();
    void emergencyStop();
    Result clearFaultAt(uint32_t now);
    bool isActive() const { return _active; }
    bool isBusy() const { return _active; }
    SyncError getError() const { return _error; }
    MotionState getReferenceState() const { return _state; }
    uint8_t size() const { return _size; }
    // Actual contour error relative to the first member, in this member's units.
    double phaseError(uint8_t member) const;
    // In scalar master units; zero disables the extra contour diagnostic.
    Result setPhaseLimit(double error, uint32_t timeoutUs = 100000);
 private:
    AxisPort _axes[Capacity];
    double _ratios[Capacity] = {}, _starts[Capacity] = {};
    uint8_t _size = 0;
    bool _active = false, _stopping = false;
    SyncError _error = SyncError::NONE;
    SCurveProfile _profile;
    MotionState _state;
    double _time = 0.0, _profileOrigin = 0.0, _acceleration = 0.0, _deceleration = 0.0, _jerk = 0.0;
    uint32_t _lastUs = 0, _periodUs = 0, _settleUs = 0, _settleLimit = 0;
    double _phaseLimit = 0.0;
    uint32_t _phaseTimeoutUs = 100000, _phaseTimerUs = 0;
    void fail(SyncError error);
    void commit();
};

}  // namespace axis

#include "AxisSyncGroup.h"
