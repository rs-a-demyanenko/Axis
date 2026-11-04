#pragma once

#include <math.h>
#include <float.h>

namespace axis {

template <uint8_t N>
Result Group<N>::setRatio(uint8_t member, double ratio) {
    if (_active) return Result::BUSY;
    if (member >= _size || !isfinite(ratio) || (member == 0 && ratio == 0.0)) return Result::INVALID_ARGUMENT;
    _ratios[member] = ratio;
    for (uint8_t i = 0; i < _size; ++i) _starts[i] = _axes[i].core->getPosition();
    _state = MotionState();
    return Result::OK;
}

template <uint8_t N>
Result Group<N>::add(AxisPort port, double ratio) {
    if (_active) return Result::BUSY;
    if (!port.valid() || !isfinite(ratio) || (_size == 0 && ratio == 0.0) || _size >= N)
        return Result::INVALID_ARGUMENT;
    for (uint8_t i = 0; i < _size; ++i) if (_axes[i].core == port.core) return Result::INVALID_ARGUMENT;
    _axes[_size] = port; _ratios[_size++] = ratio;
    return Result::OK;
}

template <uint8_t N>
Result Group<N>::moveByAt(uint32_t now, double distance, double speed) {
    if (_active) return Result::BUSY;
    if (_error != SyncError::NONE) return Result::FAULT_ACTIVE;
    if (!_size || !isfinite(distance) || !isfinite(speed) || speed < 0.0) return Result::INVALID_ARGUMENT;
    for (uint8_t i = 0; i < _size; ++i) if (_axes[i].core->isBusy()) return Result::BUSY;
    double velocity = DBL_MAX, acceleration = DBL_MAX, deceleration = DBL_MAX, jerk = DBL_MAX;
    bool hasJerkLimit = false;
    _periodUs = _settleLimit = 0;
    for (uint8_t i = 0; i < _size; ++i) {
        _axes[i].update(now); _axes[i].commit();
        Result result = _axes[i].core->canSynchronize();
        if (result != Result::OK) return result;
        const Config& c = _axes[i].core->getConfig();
        _starts[i] = _axes[i].core->getPosition();
        double target = _starts[i] + _ratios[i] * distance;
        if (!isfinite(target)) return Result::INVALID_ARGUMENT;
        double tolerance = _ratios[i] == 0.0 || distance == 0.0 ? c.positionTolerance : 0.0;
        if (c.softLimitsEnabled && (target < c.softMin - tolerance || target > c.softMax + tolerance))
            return Result::OUT_OF_RANGE;
        double ratio = fabs(_ratios[i]);
        if (ratio > 0.0) {
            velocity = fmin(velocity, c.maxSpeed / ratio);
            acceleration = fmin(acceleration, c.acceleration / ratio);
            deceleration = fmin(deceleration, c.deceleration / ratio);
            if (c.maxJerk > 0.0) { jerk = fmin(jerk, c.maxJerk / ratio); hasJerkLimit = true; }
        }
        if (c.controlPeriodUs > _periodUs) _periodUs = c.controlPeriodUs;
        if (c.settleTimeUs > _settleLimit) _settleLimit = c.settleTimeUs;
    }
    for (uint8_t i = 0; i < _size; ++i)
        if (_axes[i].core->getConfig().maxTickGapUs < _periodUs) return Result::INVALID_ARGUMENT;
    if (!hasJerkLimit) jerk = acceleration * 10.0;
    if (speed > 0.0) velocity = fmin(velocity, speed);
    if (!_profile.planPosition(distance, velocity, acceleration, deceleration, jerk)) return Result::INVALID_ARGUMENT;
    for (uint8_t i = 0; i < _size; ++i) {
        Result result = _axes[i].core->beginSynchronization();
        if (result != Result::OK) {
            for (uint8_t j = 0; j < i; ++j) _axes[j].core->releaseSynchronization(false);
            commit();
            return result;
        }
    }
    _acceleration = acceleration; _deceleration = deceleration; _jerk = jerk;
    _state = MotionState(); _profileOrigin = _time = 0.0;
    _lastUs = now; _settleUs = _phaseTimerUs = 0;
    _active = true; _stopping = false;
    return Result::OK;
}

template <uint8_t N>
Result Group<N>::moveToAt(uint32_t now, double position, double speed) {
    if (!_size || !isfinite(position)) return Result::INVALID_ARGUMENT;
    if (_active || _axes[0].core->isBusy()) return Result::BUSY;
    _axes[0].update(now); _axes[0].commit();
    return moveByAt(now, (position - _axes[0].core->getPosition()) / _ratios[0], speed);
}

template <uint8_t N>
void Group<N>::commit() { for (uint8_t i = 0; i < _size; ++i) _axes[i].commit(); }

template <uint8_t N>
void Group<N>::fail(SyncError error) {
    if (_error == SyncError::NONE) _error = error;
    _active = false;
    for (uint8_t i = 0; i < _size; ++i) _axes[i].core->synchronizationFault();
    commit();
}

template <uint8_t N>
bool Group<N>::tickAt(uint32_t now) {
    if (!_active) {
        bool updated = false;
        for (uint8_t i = 0; i < _size; ++i) {
            if (_axes[i].core->getMode() == Mode::SYNCHRONIZED) continue;
            updated = _axes[i].update(now) || updated;
            _axes[i].commit();
        }
        return updated;
    }
    for (uint8_t i = 0; i < _size; ++i) {
        if (_axes[i].core->getMode() != Mode::SYNCHRONIZED) {
            fail(_axes[i].core->getFault() == Fault::NONE ? SyncError::INTERRUPTED : SyncError::AXIS_FAULT);
            return true;
        }
    }
    uint32_t elapsed = now - _lastUs;
    bool due = elapsed >= _periodUs;
    if (due) {
        _time += elapsed * 1e-6;
        _state = _profile.sample(_time);
        _state.position += _profileOrigin;
    }
    // Read/control every member before writing ANY new motor output. Thus a
    // later member's limit prevents a partially committed group command.
    for (uint8_t i = 0; i < _size; ++i) {
        ExternalReference reference;
        reference.timestampUs = now; reference.updateControl = due;
        reference.state.position = _starts[i] + _ratios[i] * _state.position;
        reference.state.velocity = _ratios[i] * _state.velocity;
        reference.state.acceleration = _ratios[i] * _state.acceleration;
        reference.state.jerk = _ratios[i] * _state.jerk;
        if (_axes[i].core->validateReference(reference) != Result::OK) {
            fail(SyncError::REFERENCE_LIMIT); return true;
        }
        _axes[i].update(now, &reference);
        if (_axes[i].core->getFault() != Fault::NONE) { fail(SyncError::AXIS_FAULT); return true; }
    }
    if (due) {
        _lastUs = now;
        bool phaseOK = true;
        if (_phaseLimit > 0.0) for (uint8_t i = 1; i < _size; ++i) {
            if (_ratios[i] != 0.0 && fabs(phaseError(i) / _ratios[i]) > _phaseLimit) phaseOK = false;
        }
        if (!phaseOK) {
            _phaseTimerUs = elapsed >= _phaseTimeoutUs - _phaseTimerUs ? _phaseTimeoutUs : _phaseTimerUs + elapsed;
            if (_phaseTimerUs >= _phaseTimeoutUs) { fail(SyncError::PHASE_ERROR); return true; }
        } else _phaseTimerUs = 0;
        bool settled = _time >= _profile.duration();
        for (uint8_t i = 0; i < _size; ++i) {
            const Config& c = _axes[i].core->getConfig();
            if (fabs(_axes[i].core->getPosition() - (_starts[i] + _ratios[i] * _state.position)) > c.positionTolerance ||
                fabs(_axes[i].core->getVelocity()) > c.velocityTolerance) settled = false;
        }
        if (settled) {
            _settleUs = elapsed >= _settleLimit - _settleUs ? _settleLimit : _settleUs + elapsed;
            if (_settleUs >= _settleLimit) {
                for (uint8_t i = 0; i < _size; ++i)
                    _axes[i].core->finishSynchronization(_stopping ? Completion::STOPPED : Completion::REACHED);
                _active = false;
            }
        } else _settleUs = 0;
    }
    commit(); return due;
}

template <uint8_t N>
Result Group<N>::stop() {
    if (_error != SyncError::NONE) return Result::FAULT_ACTIVE;
    if (!_active) {
        Result result = Result::OK;
        for (uint8_t i = 0; i < _size; ++i) {
            Result current = _axes[i].core->stop();
            if (current != Result::OK) result = current;
        }
        return result;
    }
    if (_stopping) return Result::OK;
    if (!_profile.planVelocity(_state.velocity, _state.acceleration, 0.0,
                               _acceleration, _deceleration, _jerk)) {
        fail(SyncError::NUMERIC_ERROR); return Result::FAULT_ACTIVE;
    }
    _profileOrigin = _state.position; _time = 0.0; _settleUs = 0; _stopping = true;
    return Result::OK;
}

template <uint8_t N>
void Group<N>::disable() {
    _active = false;
    for (uint8_t i = 0; i < _size; ++i) _axes[i].core->disable();
    commit();
}

template <uint8_t N>
void Group<N>::emergencyStop() {
    _active = false; _error = SyncError::AXIS_FAULT;
    for (uint8_t i = 0; i < _size; ++i) _axes[i].core->emergencyStop();
    commit();
}

template <uint8_t N>
Result Group<N>::clearFaultAt(uint32_t now) {
    if (_active) return Result::BUSY;
    for (uint8_t i = 0; i < _size; ++i) if (_axes[i].core->getMode() == Mode::SYNCHRONIZED) return Result::BUSY;
    Result result = Result::OK;
    for (uint8_t i = 0; i < _size; ++i) {
        _axes[i].update(now);
        Result current = _axes[i].core->clearFault();
        if (current != Result::OK) result = current;
    }
    commit();
    if (result == Result::OK) _error = SyncError::NONE;
    return result;
}

template <uint8_t N>
double Group<N>::phaseError(uint8_t member) const {
    if (member >= _size || !_size) return 0.0;
    double masterDelta = (_axes[0].core->getPosition() - _starts[0]) / _ratios[0];
    return (_axes[member].core->getPosition() - _starts[member]) - _ratios[member] * masterDelta;
}

template <uint8_t N>
Result Group<N>::setPhaseLimit(double error, uint32_t timeoutUs) {
    if (_active) return Result::BUSY;
    if (!isfinite(error) || error < 0.0 || timeoutUs == 0 || timeoutUs >= 0x80000000UL)
        return Result::INVALID_ARGUMENT;
    _phaseLimit = error; _phaseTimeoutUs = timeoutUs;
    return Result::OK;
}

}  // namespace axis
