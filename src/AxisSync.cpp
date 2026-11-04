#include "AxisSync.h"

#include <limits.h>
#include <math.h>

namespace axis {
namespace {
bool positive(double x) { return isfinite(x) && x > 0.0; }
bool duration(uint32_t x) { return x > 0 && x < 0x80000000UL; }
double clamp(double x, double limit) { return fmax(-limit, fmin(limit, x)); }
bool differenceFits(int64_t a, int64_t b) {
    return !((b > 0 && a < INT64_MIN + b) || (b < 0 && a > INT64_MAX + b));
}
double filtered(double old, double value, double dt, double tau) {
    return old + (tau == 0.0 ? 1.0 : dt / (tau + dt)) * (value - old);
}
uint32_t accumulate(uint32_t value, uint32_t dt, uint32_t limit) {
    return value >= limit || dt >= limit - value ? limit : value + dt;
}
bool fresh(uint32_t now, const MasterSample& sample, uint32_t timeout) {
    return sample.valid && static_cast<uint32_t>(now - sample.timestampUs) <= timeout;
}
}  // namespace

bool AxisPort::update(uint32_t now, const ExternalReference* reference) const {
    if (!valid()) return false;
    uint32_t count;
    Inputs inputs;
    if (!read(context, count, inputs)) { core->synchronizationFault(); return false; }
    return reference ? core->tickSynchronized(now, count, inputs, *reference) : core->tick(now, count, inputs);
}

Result EncoderMaster::begin(const EncoderMasterConfig& config, uint32_t now, uint32_t raw) {
    if (!positive(config.countsPerRevolution) || !duration(config.periodUs) || !duration(config.maxGapUs) ||
        config.maxGapUs < config.periodUs || !isfinite(config.filterTime) || config.filterTime < 0.0 ||
        !isfinite(config.maxCountRate) || config.maxCountRate < 0.0) return Result::INVALID_ARGUMENT;
    uint32_t epoch = _sample.epoch + 1;
    _config = config;
    _lastUs = now; _lastRaw = raw; _counts = _sampleCounts = 0;
    _sample = MasterSample();
    _sample.timestampUs = now; _sample.epoch = epoch; _sample.valid = true;
    _sample.identity = this; _sample.countsAvailable = true;
    _sample.countsPerUnit = config.countsPerRevolution;
    _initialized = true;
    return Result::OK;
}

bool EncoderMaster::update(uint32_t now, uint32_t raw, bool valid) {
    if (!_initialized) return false;
    if (!valid) _sample.valid = false;  // Latched; a new begin creates a new epoch.
    uint32_t change = raw - _lastRaw;
    _lastRaw = raw;
    int64_t delta = change <= INT32_MAX ? static_cast<int64_t>(change) : static_cast<int64_t>(change) - 4294967296LL;
    if (_config.inverted) delta = -delta;
    if ((delta > 0 && _counts > INT64_MAX - delta) || (delta < 0 && _counts < INT64_MIN - delta)) {
        _sample.valid = false;
        return false;
    }
    _counts += delta;
    const uint32_t elapsed = now - _lastUs;
    if (elapsed < _config.periodUs) return false;
    if (elapsed > _config.maxGapUs || !differenceFits(_counts, _sampleCounts)) _sample.valid = false;
    double dt = elapsed * 1e-6;
    double counts = differenceFits(_counts, _sampleCounts) ? static_cast<double>(_counts - _sampleCounts) : 0.0;
    if (_config.maxCountRate > 0.0 && fabs(counts) > _config.maxCountRate * dt + 2.0) _sample.valid = false;
    double oldVelocity = _sample.state.velocity, oldAcceleration = _sample.state.acceleration;
    _sample.state.position = static_cast<double>(_counts) / _config.countsPerRevolution;
    _sample.state.velocity = filtered(oldVelocity, counts / (_config.countsPerRevolution * dt), dt, _config.filterTime);
    _sample.state.acceleration = filtered(oldAcceleration, (_sample.state.velocity - oldVelocity) / dt, dt, _config.filterTime);
    _sample.state.jerk = filtered(_sample.state.jerk, (_sample.state.acceleration - oldAcceleration) / dt, dt, _config.filterTime);
    _sample.counts = _counts; _sample.timestampUs = _lastUs = now; _sampleCounts = _counts;
    _sample.ready = true;
    return true;
}

const MasterSample& AxisMaster::update(const Core& core) {
    bool valid = core.isInitialized() && core.isHomed() && core.getFault() == Fault::NONE &&
                 isfinite(_filterTime) && _filterTime >= 0.0 &&
                 (_feedback == MasterFeedback::REFERENCE || _feedback == MasterFeedback::MEASURED);
    bool changed = _sample.axis != &core;
    uint32_t now = core.getSampleTimeUs();
    uint32_t elapsed = now - _sample.timestampUs;
    if (changed) { _sample = MasterSample(); _sample.epoch = 1; }
    _sample.axis = &core; _sample.identity = this; _sample.valid = valid;
    _sample.ready = valid;
    if (!changed && elapsed == 0) return _sample;
    if (_feedback == MasterFeedback::REFERENCE) {
        _sample.state = core.getReferenceState();
        _sample.countsAvailable = false;
    } else {
        double velocity = core.getVelocity();
        double dt = elapsed * 1e-6;
        double oldAcceleration = _sample.state.acceleration;
        double acceleration = !changed && dt > 0.0 ?
            filtered(oldAcceleration, (velocity - _sample.state.velocity) / dt, dt, _filterTime) : 0.0;
        _sample.state.jerk = !changed && dt > 0.0 ?
            filtered(_sample.state.jerk, (acceleration - oldAcceleration) / dt, dt, _filterTime) : 0.0;
        _sample.state.position = core.getPosition(); _sample.state.velocity = velocity;
        _sample.state.acceleration = acceleration;
        _sample.countsAvailable = true; _sample.counts = core.getAbsolutePositionCounts();
        _sample.countsPerUnit = core.getConfig().encoder.countsPerUnit;
    }
    _sample.timestampUs = now;
    return _sample;
}

Result Gear::engageAt(uint32_t now, AxisPort follower, const MasterSample& master, const GearConfig& config) {
    if (_status.active || (_follower.valid() && _follower.core->isBusy())) return Result::BUSY;
    if (_status.error != SyncError::NONE) return Result::FAULT_ACTIVE;
    if (!follower.valid() || !isfinite(config.ratio) || config.ratio == 0.0 ||
        !positive(config.maxPhaseError) || !positive(config.captureGain) ||
        !duration(config.phaseErrorTimeUs) || !duration(config.sourceTimeoutUs) ||
        (config.mode != GearMode::STRICT && config.mode != GearMode::LIMITED)) return Result::INVALID_ARGUMENT;
    if (follower.core->isBusy()) return Result::BUSY;
    if (!fresh(now, master, config.sourceTimeoutUs) || !master.identity) return Result::FAULT_ACTIVE;
    if (master.axis == follower.core || (master.axis && master.axis->getMode() == Mode::SYNCHRONIZED))
        return Result::INVALID_ARGUMENT;  // No cycles/chains in this version.
    if (master.countsAvailable && !positive(master.countsPerUnit)) return Result::INVALID_ARGUMENT;
    if (!isfinite(master.state.position) || !isfinite(master.state.velocity) ||
        !isfinite(master.state.acceleration) || !isfinite(master.state.jerk)) return Result::INVALID_ARGUMENT;
    follower.update(now); follower.commit();
    Result result = follower.core->canSynchronize();
    if (result != Result::OK) return result;
    if (config.mode == GearMode::STRICT &&
        (!master.ready || fabs(master.state.velocity * config.ratio) > follower.core->getConfig().velocityTolerance ||
         fabs(master.state.acceleration * config.ratio) > 1e-6)) return Result::BUSY;
    if (_follower.valid() && _follower.core != follower.core) { _follower.core->disable(); _follower.commit(); }
    result = follower.core->beginSynchronization();
    if (result != Result::OK) return result;
    _follower = follower; _config = config;
    _masterIdentity = master.identity; _masterAxis = master.axis; _masterEpoch = master.epoch;
    _masterPosition = master.state.position; _masterCounts = master.counts;
    _useCounts = master.countsAvailable; _countsPerUnit = master.countsPerUnit;
    _followerPosition = follower.core->getPosition();
    _status = GearSnapshot(); _status.active = true;
    _status.reference.position = _status.idealPosition = _followerPosition;
    _lastUs = now; _phaseErrorUs = 0;
    return Result::OK;
}

void Gear::fail(SyncError error) {
    if (_status.error == SyncError::NONE) _status.error = error;
    _status.active = _status.inSync = false;
    if (_follower.valid()) { _follower.core->synchronizationFault(); _follower.commit(); }
    if (_handler) _handler(_status.error, _handlerContext);
}

bool Gear::tickAt(uint32_t now, const MasterSample& master) {
    if (!_follower.valid()) return false;
    if (!_status.active) {
        if (_follower.core->getMode() == Mode::SYNCHRONIZED) return false;
        bool result = _follower.update(now); _follower.commit(); return result;
    }
    if (_follower.core->getMode() != Mode::SYNCHRONIZED) {
        fail(_follower.core->getFault() == Fault::NONE ? SyncError::INTERRUPTED : SyncError::AXIS_FAULT);
        return true;
    }
    if (!master.valid) { fail(SyncError::SOURCE_INVALID); return true; }
    if (master.axis && master.axis->getMode() == Mode::SYNCHRONIZED) { fail(SyncError::SOURCE_CHANGED); return true; }
    if (!fresh(now, master, _config.sourceTimeoutUs)) { fail(SyncError::SOURCE_TIMEOUT); return true; }
    if (master.identity != _masterIdentity || master.axis != _masterAxis || master.epoch != _masterEpoch ||
        master.countsAvailable != _useCounts || (_useCounts && master.countsPerUnit != _countsPerUnit)) {
        fail(SyncError::SOURCE_CHANGED); return true;
    }
    double delta;
    if (_useCounts) {
        if (!differenceFits(master.counts, _masterCounts)) { fail(SyncError::SOURCE_CHANGED); return true; }
        delta = static_cast<double>(master.counts - _masterCounts) / _countsPerUnit;
    } else delta = master.state.position - _masterPosition;
    _status.idealPosition = _followerPosition + _config.ratio * delta;
    const Config& c = _follower.core->getConfig();
    if (!isfinite(_status.idealPosition) || !isfinite(master.state.velocity) ||
        !isfinite(master.state.acceleration) || !isfinite(master.state.jerk)) {
        fail(SyncError::NUMERIC_ERROR); return true;
    }
    if (c.softLimitsEnabled && (_status.idealPosition < c.softMin - c.positionTolerance ||
                                _status.idealPosition > c.softMax + c.positionTolerance)) {
        fail(SyncError::REFERENCE_LIMIT); return true;
    }
    uint32_t elapsed = now - _lastUs;
    bool due = elapsed >= c.controlPeriodUs;
    ExternalReference reference;
    reference.timestampUs = now; reference.updateControl = due;
    double idealVelocity = master.state.velocity * _config.ratio;
    if (_config.mode == GearMode::STRICT) {
        reference.state.position = _status.idealPosition;
        reference.state.velocity = idealVelocity;
        reference.state.acceleration = master.state.acceleration * _config.ratio;
        reference.state.jerk = master.state.jerk * _config.ratio;
    } else {
        reference.state = _status.reference;
        if (due) {
            double dt = elapsed * 1e-6;
            double desired = clamp(idealVelocity + _config.captureGain *
                                   (_status.idealPosition - reference.state.position), c.maxSpeed);
            double limit = fmin(c.acceleration, c.deceleration);
            if (c.maxJerk > 0.0) {
                if (!_ramp.planTrackingVelocity(reference.state.velocity, reference.state.acceleration,
                                                desired, limit, c.maxJerk)) {
                    fail(SyncError::NUMERIC_ERROR); return true;
                }
                MotionState state = _ramp.sample(dt);
                reference.state.position += state.position;
                reference.state.velocity = state.velocity;
                reference.state.acceleration = state.acceleration;
                reference.state.jerk = state.jerk;
            } else {
                double previous = reference.state.velocity;
                reference.state.velocity += clamp(desired - previous, limit * dt);
                reference.state.position += 0.5 * (previous + reference.state.velocity) * dt;
                reference.state.acceleration = (reference.state.velocity - previous) / dt;
                reference.state.jerk = 0.0;
            }
        }
    }
    if (_follower.core->validateReference(reference) != Result::OK) {
        fail(SyncError::REFERENCE_LIMIT); return true;
    }
    bool updated = _follower.update(now, &reference);
    if (_follower.core->getFault() != Fault::NONE) { fail(SyncError::AXIS_FAULT); return true; }
    if (due) {
        _lastUs = now; _status.reference = reference.state;
        _status.phaseError = _status.idealPosition - _follower.core->getPosition();
        _status.referenceLag = _status.idealPosition - reference.state.position;
        _status.inSync = fabs(_status.phaseError) <= c.positionTolerance &&
                         fabs(_status.referenceLag) <= c.positionTolerance &&
                         fabs(_follower.core->getVelocity() - idealVelocity) <= c.velocityTolerance;
        if (fabs(_status.phaseError) > _config.maxPhaseError) {
            _phaseErrorUs = accumulate(_phaseErrorUs, elapsed, _config.phaseErrorTimeUs);
            if (_phaseErrorUs >= _config.phaseErrorTimeUs) { fail(SyncError::PHASE_ERROR); return true; }
        } else _phaseErrorUs = 0;
    }
    _follower.commit();
    return updated;
}

Result Gear::stop() {
    if (!_follower.valid()) return Result::NOT_INITIALIZED;
    if (_status.error != SyncError::NONE) return Result::FAULT_ACTIVE;
    if (!_status.active) return _follower.core->stop();
    Result result = _follower.core->releaseSynchronization(true);
    _status.active = _status.inSync = false;
    _follower.commit();
    return result;
}

void Gear::disable() {
    _status.active = _status.inSync = false;
    if (_follower.valid()) { _follower.core->disable(); _follower.commit(); }
}

void Gear::emergencyStop() {
    _status.active = _status.inSync = false;
    _status.error = SyncError::AXIS_FAULT;
    if (_follower.valid()) { _follower.core->emergencyStop(); _follower.commit(); }
    if (_handler) _handler(_status.error, _handlerContext);
}

Result Gear::clearFaultAt(uint32_t now) {
    if (_status.active) return Result::BUSY;
    if (!_follower.valid()) { _status.error = SyncError::NONE; return Result::OK; }
    if (_follower.core->getMode() == Mode::SYNCHRONIZED) return Result::BUSY;
    _follower.update(now);
    Result result = _follower.core->clearFault();
    _follower.commit();
    if (result == Result::OK) { _status.error = SyncError::NONE; _status.inSync = false; }
    return result;
}

const char* toString(SyncError error) {
    switch (error) {
        case SyncError::NONE: return "NONE";
        case SyncError::AXIS_FAULT: return "AXIS_FAULT";
        case SyncError::SOURCE_INVALID: return "SOURCE_INVALID";
        case SyncError::SOURCE_TIMEOUT: return "SOURCE_TIMEOUT";
        case SyncError::SOURCE_CHANGED: return "SOURCE_CHANGED";
        case SyncError::REFERENCE_LIMIT: return "REFERENCE_LIMIT";
        case SyncError::PHASE_ERROR: return "PHASE_ERROR";
        case SyncError::INTERRUPTED: return "INTERRUPTED";
        case SyncError::NUMERIC_ERROR: return "NUMERIC_ERROR";
    }
    return "UNKNOWN";
}

}  // namespace axis
