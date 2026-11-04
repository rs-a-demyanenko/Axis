#pragma once

// This file is included after the existing Plant fixture; the port simulates
// separately committed motor outputs, as the real Arduino adapter does.
struct DrivenPlant : Plant {
    axis::Output applied;
    unsigned driveCommits = 0;
    explicit DrivenPlant(axis::Config c = testConfig()) : Plant(c) {}
    void prepare() { home(); core.disable(); applied = core.getOutput(); }
    void advance(uint32_t us) {
        double dt = us * 1e-6;
        if (!frozen) {
            double demand = applied.action == axis::OutputAction::DRIVE ? applied.duty * 20 : 0;
            double tau = applied.action == axis::OutputAction::BRAKE ? 0.01 : 0.04;
            velocity += (demand - velocity) * (1 - exp(-dt / tau));
            position += velocity * dt;
        } else velocity = 0;
        now += us; sensors();
    }
    static bool read(void* context, uint32_t& count, axis::Inputs& inputs) {
        DrivenPlant* self = static_cast<DrivenPlant*>(context);
        count = self->count(); inputs = self->inputs; return true;
    }
    static void write(void* context) {
        DrivenPlant* self = static_cast<DrivenPlant*>(context);
        self->applied = self->core.getOutput();
        if (self->applied.action == axis::OutputAction::DRIVE) ++self->driveCommits;
    }
    axis::AxisPort port() {
        axis::AxisPort result;
        result.core = &core; result.context = this; result.read = read; result.apply = write;
        return result;
    }
};

static uint32_t alignPlants(DrivenPlant& a, DrivenPlant& b) {
    uint32_t now = (a.now > b.now ? a.now : b.now) + 1000000;
    a.now = b.now = now;
    a.port().update(now); b.port().update(now); a.port().commit(); b.port().commit();
    return now;
}

static void coordinatedMotion() {
    axis::Config xConfig = testConfig(); xConfig.maxJerk = 100;
    xConfig.velocityFilterTime = 0.02;  // Match the common 10 ms group cycle.
    axis::Config yConfig = xConfig;
    yConfig.maxSpeed = 2; yConfig.acceleration = yConfig.deceleration = 6; yConfig.maxJerk = 30;
    yConfig.controlPeriodUs = 10000;
    DrivenPlant x(xConfig), y(yConfig);
    x.prepare(); y.prepare();
    uint32_t now = alignPlants(x, y);
    axis::Group<2> group;
    CHECK(group.add(x.port(), 1) == Result::OK);
    CHECK(group.add(x.port(), 2) == Result::INVALID_ARGUMENT);
    CHECK(group.add(y.port(), 2) == Result::OK);
    double xStart = x.core.getPosition(), yStart = y.core.getPosition();
    CHECK(group.moveByAt(now, 3, 5) == Result::OK);  // Y caps common velocity at 1.
    CHECK(group.moveByAt(now, 1) == Result::BUSY);
    CHECK(group.setRatio(1, 3) == Result::BUSY);
    CHECK(x.core.moveBy(1) == Result::BUSY);
    CHECK(y.core.stop() == Result::BUSY);
    uint32_t lastControl = x.core.getSampleTimeUs();
    for (unsigned i = 0; i < 6000 && group.isBusy(); ++i) {
        x.advance(5000); y.advance(5000); now += 5000;
        group.tickAt(now);
        CHECK(group.getError() == axis::SyncError::NONE);
        axis::MotionState xr = x.core.getReferenceState(), yr = y.core.getReferenceState();
        NEAR(yr.position - yStart, 2 * (xr.position - xStart), 1e-9);
        NEAR(yr.velocity, 2 * xr.velocity, 1e-9);
        NEAR(yr.acceleration, 2 * xr.acceleration, 1e-9);
        CHECK(fabs(xr.velocity) <= 1 + 1e-8);
        CHECK(fabs(yr.velocity) <= 2 + 1e-8);
        CHECK(fabs(yr.acceleration) <= 6 + 1e-8);
        CHECK(fabs(yr.jerk) <= 30 + 1e-8);
        CHECK(x.core.getSampleTimeUs() == y.core.getSampleTimeUs());
        if ((i % 2) == 0) CHECK(x.core.getSampleTimeUs() == lastControl);
        else lastControl = x.core.getSampleTimeUs();
    }
    if (group.isBusy()) fprintf(stderr, "Group timeout X=%f/%f v=%f, Y=%f/%f v=%f\n", x.core.getPosition(),
        x.core.getReferenceState().position, x.core.getVelocity(), y.core.getPosition(),
        y.core.getReferenceState().position, y.core.getVelocity());
    CHECK(!group.isBusy());
    NEAR(x.core.getPosition() - xStart, 3, xConfig.positionTolerance);
    NEAR(y.core.getPosition() - yStart, 6, yConfig.positionTolerance);
    CHECK(x.core.getCompletion() == axis::Completion::REACHED);
    CHECK(y.core.getCompletion() == axis::Completion::REACHED);
    CHECK(group.moveByAt(now, 100) == Result::OUT_OF_RANGE);
    CHECK(!group.isActive());
    CHECK(group.moveByAt(now, 2, 1) == Result::OK);
    for (unsigned i = 0; i < 16; ++i) {
        x.advance(5000); y.advance(5000); now += 5000; group.tickAt(now);
    }
    axis::MotionState before = group.getReferenceState();
    CHECK(before.acceleration > 0);
    CHECK(group.stop() == Result::OK);
    for (unsigned i = 0; i < 3000 && group.isBusy(); ++i) {
        x.advance(5000); y.advance(5000); now += 5000; group.tickAt(now);
        CHECK(group.getError() == axis::SyncError::NONE);
        NEAR(y.core.getReferenceState().velocity, 2 * x.core.getReferenceState().velocity, 1e-9);
    }
    if (group.isBusy()) fprintf(stderr, "Group stop timeout X=%f/%f v=%f, Y=%f/%f v=%f scalar=%f v=%f a=%f\n",
        x.core.getPosition(), x.core.getReferenceState().position, x.core.getVelocity(), y.core.getPosition(),
        y.core.getReferenceState().position, y.core.getVelocity(), group.getReferenceState().position,
        group.getReferenceState().velocity, group.getReferenceState().acceleration);
    CHECK(!group.isBusy());
    CHECK(x.core.getCompletion() == axis::Completion::STOPPED);
    CHECK(y.core.getCompletion() == axis::Completion::STOPPED);
    CHECK(group.moveToAt(now, x.core.getPosition() + 0.2, 0.5) == Result::OK);
    for (unsigned i = 0; i < 2000 && group.isBusy(); ++i) {
        x.advance(5000); y.advance(5000); now += 5000; group.tickAt(now);
    }
    CHECK(group.getError() == axis::SyncError::NONE);
    CHECK(!group.isBusy());
    group.disable();
    CHECK(group.setRatio(0, 0) == Result::INVALID_ARGUMENT);
    CHECK(group.setRatio(1, -2) == Result::OK);
    NEAR(group.phaseError(1), 0, 1e-12);
}

static void coordinatedFaults() {
    axis::Config c = testConfig(); c.maxJerk = 100;
    DrivenPlant x(c), y(c);
    x.prepare(); y.prepare(); uint32_t now = alignPlants(x, y);
    axis::Group<2> group;
    CHECK(group.add(x.port(), 1) == Result::OK);
    CHECK(group.add(y.port(), -1) == Result::OK);
    // Negative ratio is legal; start inside both axes' travel.
    CHECK(group.moveByAt(now, 1) == Result::OUT_OF_RANGE);
    CHECK(x.core.moveTo(5, 2) == Result::OK);
    CHECK(y.core.moveTo(5, 2) == Result::OK);
    for (unsigned i = 0; i < 3000 && (x.core.isBusy() || y.core.isBusy()); ++i) {
        x.advance(5000); y.advance(5000); now += 5000; group.tickAt(now);
    }
    CHECK(!x.core.isBusy() && !y.core.isBusy());
    CHECK(group.moveByAt(now, 1, 1) == Result::OK);
    for (unsigned i = 0; i < 50; ++i) {
        x.advance(5000); y.advance(5000); now += 5000; group.tickAt(now);
    }
    CHECK(x.core.getReferenceState().velocity > 0);
    CHECK(y.core.getReferenceState().velocity < 0);
    y.switches = false; y.inputs.minLimit = true;
    unsigned xWrites = x.driveCommits, yWrites = y.driveCommits;
    x.advance(1); y.advance(1); ++now; group.tickAt(now);
    CHECK(group.getError() == axis::SyncError::AXIS_FAULT);
    CHECK(x.core.getFault() == Fault::SYNC_FAULT);
    CHECK(y.core.getFault() == Fault::MIN_LIMIT);
    CHECK(x.applied.action == axis::OutputAction::COAST);
    CHECK(y.applied.action == axis::OutputAction::COAST);
    CHECK(x.driveCommits == xWrites && y.driveCommits == yWrites);
    CHECK(group.clearFaultAt(now) == Result::FAULT_ACTIVE);
    y.inputs.minLimit = false;
    for (unsigned i = 0; i < 300; ++i) {
        x.advance(5000); y.advance(5000); now += 5000; group.tickAt(now);
    }
    CHECK(group.clearFaultAt(now) == Result::OK);
    CHECK(!group.isActive());
    CHECK(x.applied.action == axis::OutputAction::COAST);
    CHECK(group.moveByAt(now, 0.5) == Result::OK);
    x.core.disable(); x.port().commit();
    x.advance(5000); y.advance(5000); now += 5000; group.tickAt(now);
    CHECK(group.getError() == axis::SyncError::INTERRUPTED);
    group.disable();
}

static void masterEncoderTests() {
    axis::EncoderMaster encoder;
    axis::EncoderMasterConfig config;
    config.countsPerRevolution = 1000; config.filterTime = 0;
    CHECK(encoder.begin(config, 0xfffffff0UL, 0xfffffff0UL) == Result::OK);
    encoder.update(0x1378, 0x10);
    CHECK(encoder.sample().valid);
    CHECK(encoder.sample().counts == 32);
    NEAR(encoder.sample().state.position, 0.032, 1e-12);
    uint32_t epoch = encoder.sample().epoch;
    config.inverted = true;
    CHECK(encoder.begin(config, 0, 0) == Result::OK);
    CHECK(encoder.sample().epoch != epoch);
    encoder.update(5000, 1000);
    NEAR(encoder.sample().state.position, -1, 1e-12);
    encoder.update(10000, 1000, false);
    CHECK(!encoder.sample().valid);
    encoder.update(15000, 1000, true);
    CHECK(!encoder.sample().valid);
    config.inverted = false; config.maxCountRate = 100;
    CHECK(encoder.begin(config, 0, 0) == Result::OK);
    encoder.update(5000, 1000);
    CHECK(!encoder.sample().valid);
    config.maxCountRate = 0;
    CHECK(encoder.begin(config, 0, 0) == Result::OK);
    encoder.update(60000, 0);
    CHECK(!encoder.sample().valid);
}

static void spindleStrict() {
    axis::Config c = testConfig(); c.maxJerk = 1000;
    DrivenPlant z(c); z.prepare();
    uint32_t now = z.now;
    axis::EncoderMaster spindle;
    axis::EncoderMasterConfig config; config.countsPerRevolution = 1000;
    CHECK(spindle.begin(config, now, 0) == Result::OK);
    CHECK(!spindle.sample().ready);
    z.advance(5000); now += 5000; spindle.update(now, 0);
    CHECK(spindle.sample().ready);
    axis::Gear gear;
    axis::GearConfig coupling; coupling.ratio = 1; coupling.maxPhaseError = 0.5;
    double origin = z.core.getPosition();
    CHECK(gear.engageAt(now, z.port(), spindle.sample(), coupling) == Result::OK);
    axis::SCurveProfile rotation;
    CHECK(rotation.planPosition(3, 1, 0.5, 0.5, 2));
    unsigned steps = static_cast<unsigned>((rotation.duration() + 2) / 0.005);
    for (unsigned i = 0; i < steps; ++i) {
        z.advance(5000); now += 5000;
        axis::MotionState master = rotation.sample((i + 1) * 0.005);
        uint32_t count = static_cast<uint32_t>(llround(master.position * 1000));
        spindle.update(now, count);
        gear.tickAt(now, spindle.sample());
        if (gear.snapshot().error != axis::SyncError::NONE)
            fprintf(stderr, "Strict spindle error %s, x=%f ideal=%f\n", axis::toString(gear.snapshot().error),
                    z.core.getPosition(), gear.snapshot().idealPosition);
        CHECK(gear.snapshot().error == axis::SyncError::NONE);
        NEAR(z.core.getReferenceState().position - origin, spindle.sample().counts / 1000.0, 1e-10);
    }
    NEAR(z.core.getPosition() - origin, 3, c.positionTolerance);
    CHECK(gear.snapshot().inSync);
    CHECK(gear.stop() == Result::OK);
    for (unsigned i = 0; i < 400 && z.core.isBusy(); ++i) {
        z.advance(5000); now += 5000; spindle.update(now, 3000); gear.tickAt(now, spindle.sample());
    }
    CHECK(!z.core.isBusy());
    gear.disable();
}

static void spindleLimitedAndFaults() {
    axis::Config c = testConfig(); c.maxJerk = 100;
    DrivenPlant z(c); z.prepare(); uint32_t now = z.now;
    axis::MasterSample master;
    master.valid = true; master.identity = &master; master.epoch = 1; master.timestampUs = now;
    master.state.position = 10; master.state.velocity = 3;
    axis::Gear gear;
    axis::GearConfig config; config.ratio = 1; config.maxPhaseError = 2;
    CHECK(gear.engageAt(now, z.port(), master, config) == Result::BUSY);  // Strict requires rest.
    config.mode = axis::GearMode::LIMITED;
    double origin = z.core.getPosition();
    CHECK(gear.engageAt(now, z.port(), master, config) == Result::OK);
    double previousAcceleration = 0;
    for (unsigned i = 0; i < 1600; ++i) {
        z.advance(5000); now += 5000; master.timestampUs = now;
        if (i < 400) master.state.position += 3 * 0.005;
        else master.state.velocity = 0;
        gear.tickAt(now, master);
        if (gear.snapshot().error != axis::SyncError::NONE)
            fprintf(stderr, "Limited error %s, x=%f ideal=%f ref=%f\n", axis::toString(gear.snapshot().error),
                    z.core.getPosition(), gear.snapshot().idealPosition, gear.snapshot().reference.position);
        CHECK(gear.snapshot().error == axis::SyncError::NONE);
        axis::MotionState state = gear.snapshot().reference;
        CHECK(fabs(state.acceleration) <= fmin(c.acceleration, c.deceleration) + 1e-8);
        CHECK(fabs(state.jerk) <= c.maxJerk + 1e-8);
        CHECK(fabs(state.acceleration - previousAcceleration) <= c.maxJerk * 0.005 + 1e-8);
        previousAcceleration = state.acceleration;
    }
    NEAR(z.core.getPosition() - origin, 6, 0.03);
    CHECK(gear.snapshot().inSync);
    now += 60000; z.advance(60000); gear.tickAt(now, master);
    CHECK(gear.snapshot().error == axis::SyncError::SOURCE_TIMEOUT);
    CHECK(z.applied.action == axis::OutputAction::COAST);
    for (unsigned i = 0; i < 200; ++i) {
        z.advance(5000); now += 5000; gear.tickAt(now, master);
    }
    CHECK(gear.clearFaultAt(now) == Result::OK);
    master.timestampUs = now;
    CHECK(gear.engageAt(now, z.port(), master, config) == Result::OK);
    ++master.epoch;
    z.advance(5000); now += 5000; master.timestampUs = now; gear.tickAt(now, master);
    CHECK(gear.snapshot().error == axis::SyncError::SOURCE_CHANGED);
    CHECK(z.applied.action == axis::OutputAction::COAST);
    gear.disable();
}

static void trackingRampTests() {
    axis::SCurveProfile ramp;
    axis::MotionState state;
    for (unsigned i = 0; i < 2000; ++i) {
        double target = i < 500 ? 2 * sin(i * 0.003) : (i < 1200 ? -1 : 0);
        double previousAcceleration = state.acceleration;
        CHECK(ramp.planTrackingVelocity(state.velocity, state.acceleration, target, 3, 20));
        axis::MotionState step = ramp.sample(0.005);
        CHECK(fabs(step.acceleration) <= 3 + 1e-9);
        CHECK(fabs(step.jerk) <= 20 + 1e-9);
        CHECK(fabs(step.velocity) <= 2 + 1e-9);
        CHECK(fabs(step.acceleration - previousAcceleration) <= 20 * 0.005 + 1e-8);
        state = step;
    }
    NEAR(state.velocity, 0, 1e-9); NEAR(state.acceleration, 0, 1e-9);
}

static void syncFailureDiagnostics() {
    axis::Gear unbound;
    unbound.emergencyStop();
    CHECK(unbound.clearFaultAt(0) == Result::OK);
    axis::Config c = testConfig(); c.maxJerk = 100; c.maxFollowingError = c.stallOutput = 0;
    DrivenPlant x(c), y(c); x.prepare(); y.prepare();
    uint32_t now = alignPlants(x, y);
    axis::Group<2> group;
    CHECK(group.add(x.port()) == Result::OK); CHECK(group.add(y.port()) == Result::OK);
    CHECK(group.setPhaseLimit(0.05, 50000) == Result::OK);
    y.frozen = true;
    CHECK(group.moveByAt(now, 1, 1) == Result::OK);
    for (unsigned i = 0; i < 500 && group.isActive(); ++i) {
        x.advance(5000); y.advance(5000); now += 5000; group.tickAt(now);
    }
    CHECK(group.getError() == axis::SyncError::PHASE_ERROR);
    CHECK(x.applied.action == axis::OutputAction::COAST && y.applied.action == axis::OutputAction::COAST);
    group.disable();

    DrivenPlant z(c); z.prepare(); now = z.now; z.frozen = true;
    axis::MasterSample master;
    master.identity = &master; master.valid = master.ready = true; master.timestampUs = now;
    axis::Gear gear;
    unsigned faults = 0;
    gear.setFaultHandler([](axis::SyncError, void* context) { ++*static_cast<unsigned*>(context); }, &faults);
    axis::GearConfig config; config.mode = axis::GearMode::LIMITED;
    config.maxPhaseError = 0.05; config.phaseErrorTimeUs = 50000;
    CHECK(gear.engageAt(now, z.port(), master, config) == Result::OK);
    for (unsigned i = 0; i < 300 && gear.isActive(); ++i) {
        z.advance(5000); now += 5000; master.timestampUs = now;
        master.state.position += 0.005; master.state.velocity = 1;
        gear.tickAt(now, master);
    }
    CHECK(gear.snapshot().error == axis::SyncError::PHASE_ERROR);
    CHECK(faults == 1);
    gear.tickAt(now, master); CHECK(faults == 1);
    CHECK(gear.clearFaultAt(now) == Result::OK);
    master.state.velocity = 0; master.timestampUs = now;
    config.mode = axis::GearMode::STRICT;
    CHECK(gear.engageAt(now, z.port(), master, config) == Result::OK);
    master.state.velocity = 100;
    z.advance(5000); now += 5000; master.timestampUs = now; gear.tickAt(now, master);
    CHECK(gear.snapshot().error == axis::SyncError::REFERENCE_LIMIT);
    CHECK(faults == 2);
    CHECK(gear.clearFaultAt(now) == Result::OK);
    master.state.velocity = 0;
    master.countsAvailable = true; master.counts = 9007199254740993LL; master.countsPerUnit = 1000;
    CHECK(gear.engageAt(now, z.port(), master, config) == Result::OK);
    double origin = z.core.getPosition();
    ++master.counts;
    z.advance(5000); now += 5000; master.timestampUs = now; gear.tickAt(now, master);
    NEAR(gear.snapshot().idealPosition - origin, 0.001, 1e-12);
    master.valid = false;
    z.advance(1); ++now; gear.tickAt(now, master);
    CHECK(gear.snapshot().error == axis::SyncError::SOURCE_INVALID);
    CHECK(z.applied.action == axis::OutputAction::COAST);
}

static void arduinoSyncAdapter() {
    ReaderData data;
    Axis<GM2::DIR_PWM> motor(40, 41);
    axis::Config config = testConfig(); config.maxJerk = 100;
    double position = 5, velocity = 0;
    data.count = 5000;
    CHECK(motor.attachEncoder(readCount, &data) == Result::OK);
    CHECK(motor.attachInputs(readInputs, &data) == Result::OK);
    CHECK(motor.begin(config) == Result::OK);
    CHECK(motor.home() == Result::OK);
    for (unsigned i = 0; i < 5000 && motor.isBusy(); ++i) {
        axis::Output output = motor.snapshot().output;
        double demand = output.action == axis::OutputAction::DRIVE ? output.duty * 20 : 0;
        velocity += (demand - velocity) * (1 - exp(-0.005 / 0.04));
        position += velocity * 0.005;
        data.count = static_cast<uint32_t>(static_cast<int64_t>(llround(position * 1000)));
        data.inputs.home = position <= 0;
        data.inputs.minLimit = position <= -2; data.inputs.maxLimit = position >= 23;
        mockMicros += 5000; motor.tick();
    }
    CHECK(motor.isHomed());
    AxisGroup<1> coordinator;
    CHECK(coordinator.add(motor.syncPort()) == Result::OK);
    CHECK(coordinator.moveBy(1, 1) == Result::OK);
    CHECK(motor.moveBy(1) == Result::BUSY);
    CHECK(motor.stop() == Result::BUSY);
    CHECK(motor.home() == Result::BUSY);
    CHECK(motor.getFault() == Fault::NONE);
    // An explicit unowned tick is still an error: it would omit the external reference.
    motor.tick();
    CHECK(motor.getFault() == Fault::SYNC_FAULT);
    CHECK(mockPwm[41] == 0);
    coordinator.tick();
    CHECK(coordinator.getError() == axis::SyncError::AXIS_FAULT);
    coordinator.disable();
}

static void rotaryAxisMaster() {
    axis::Config aConfig = testConfig();
    aConfig.kind = axis::Kind::ROTARY; aConfig.continuous = true;
    aConfig.softMax = 720; aConfig.maxJerk = 100;
    axis::Config zConfig = testConfig(); zConfig.maxJerk = 100;
    DrivenPlant a(aConfig), z(zConfig);
    a.prepare(); z.prepare(); uint32_t now = alignPlants(a, z);
    a.switches = false;  // This rotary fixture has no linear end stop at 23 units.
    axis::AxisMaster source(axis::MasterFeedback::REFERENCE);
    axis::Gear gear;
    axis::GearConfig config; config.ratio = 1.0 / 360.0; config.maxPhaseError = 0.1;
    double zOrigin = z.core.getPosition(), aOrigin = source.update(a.core).state.position;
    CHECK(gear.engageAt(now, z.port(), source.sample(), config) == Result::OK);
    CHECK(a.core.moveBy(360, 8) == Result::OK);
    for (unsigned i = 0; i < 11000; ++i) {
        a.advance(5000); z.advance(5000); now += 5000;
        a.port().update(now); a.port().commit();
        gear.tickAt(now, source.update(a.core));
        if (gear.snapshot().error != axis::SyncError::NONE) fprintf(stderr,
            "Rotary sync %s; master fault %s, x=%f v=%f; follower fault %s, x=%f ideal=%f\n",
            axis::toString(gear.snapshot().error), axis::toString(a.core.getFault()), a.core.getPosition(),
            a.core.getVelocity(), axis::toString(z.core.getFault()), z.core.getPosition(), gear.snapshot().idealPosition);
        CHECK(gear.snapshot().error == axis::SyncError::NONE);
        NEAR(z.core.getReferenceState().position - zOrigin,
             (a.core.getReferenceState().position - aOrigin) / 360, 1e-10);
    }
    CHECK(!a.core.isBusy());
    NEAR(z.core.getPosition() - zOrigin, 1, zConfig.positionTolerance);
    axis::Gear self;
    CHECK(self.engageAt(now, a.port(), source.sample(), config) == Result::INVALID_ARGUMENT);
    axis::AxisMaster measured(axis::MasterFeedback::MEASURED);
    const axis::MasterSample& sample = measured.update(a.core);
    CHECK(sample.countsAvailable && sample.axis == &a.core);
    CHECK(sample.counts == a.core.getAbsolutePositionCounts());
    NEAR(sample.state.position, a.core.getPosition(), 1e-12);
    gear.disable(); a.core.disable(); a.port().commit();
}

static void synchronizationTests() {
    trackingRampTests(); masterEncoderTests(); coordinatedMotion(); coordinatedFaults();
    spindleStrict(); spindleLimitedAndFaults();
    syncFailureDiagnostics(); rotaryAxisMaster();
    arduinoSyncAdapter();
}
