#include <Axis.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits>

uint32_t mockMicros = 0;
int mockDigital[256] = {}, mockPwm[256] = {}, mockPinModes[256] = {};
static int checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); exit(1); } } while (0)
#define NEAR(a, b, tolerance) CHECK(fabs((a) - (b)) <= (tolerance))

using axis::Result;
using axis::Fault;

static axis::Config testConfig() {
    axis::Config c;
    c.encoder = axis::EncoderScale::linear(1000);
    c.maxSpeed = 8;
    c.maxJerk = 0;  // Preserve explicit coverage of the legacy profile.
    c.velocitySetpointWeight = 1;
    c.acceleration = c.deceleration = 20;
    c.velocityKp = 0.12;
    c.velocityKi = 0.8;
    c.velocityFeedforward = 0.05;
    c.velocityFilterTime = 0.005;
    c.positionTolerance = 0.02;
    c.velocityTolerance = 0.04;
    c.softMax = 20;
    c.home.searchSpeed = 2;
    c.home.latchSpeed = 0.8;
    c.home.backoff = 0.4;
    c.home.switchPosition = -1;
    c.home.parkPosition = 0;
    c.maxFollowingError = 2;
    c.stallOutput = 0.7;
    return c;
}

struct Plant {
    axis::Core core;
    axis::Config config;
    axis::Inputs inputs;
    double position = 5, velocity = 0;
    uint32_t now = 1000;
    bool frozen = false, switches = true;
    bool seen[10] = {};

    explicit Plant(axis::Config c = testConfig(), double start = 5) : config(c), position(start) {
        sensors();
        CHECK(core.begin(config, now, count(), inputs) == Result::OK);
    }
    uint32_t count() const {
        double sign = config.encoder.inverted ? -1.0 : 1.0;
        return static_cast<uint32_t>(static_cast<int64_t>(llround(sign * position * config.encoder.countsPerUnit)));
    }
    void sensors() {
        if (!switches) return;
        inputs.home = position <= 0;
        inputs.minLimit = position <= (config.home.source == axis::HomeSource::MIN_LIMIT ? 0 : -2);
        inputs.maxLimit = position >= 23;
    }
    void step(unsigned us = 5000) {
        const double dt = us * 1e-6;
        axis::Output output = core.getOutput();
        if (!frozen) {
            double demand = output.action == axis::OutputAction::DRIVE ? output.duty * 20 : 0;
            double tau = output.action == axis::OutputAction::BRAKE ? 0.01 : 0.04;
            velocity += (demand - velocity) * (1 - exp(-dt / tau));
            position += velocity * dt;
        } else velocity = 0;
        now += us;
        sensors();
        core.tick(now, count(), inputs);
        seen[static_cast<unsigned>(core.getHomePhase())] = true;
        CHECK(fabs(core.getOutput().duty) <= config.maxOutput + 1e-9);
    }
    void run(double seconds) {
        for (unsigned i = 0; i < static_cast<unsigned>(seconds / 0.005); ++i) step();
    }
    void finish(double timeout = 25) {
        for (unsigned i = 0; i < static_cast<unsigned>(timeout / 0.005) && core.isBusy(); ++i) step();
        if (core.getFault() != Fault::NONE)
            fprintf(stderr, "Plant fault %s x=%f measured=%f phase=%u\n", axis::toString(core.getFault()),
                    position, core.getPosition(), static_cast<unsigned>(core.getHomePhase()));
        CHECK(core.getFault() == Fault::NONE);
        if (core.isBusy()) {
            axis::Snapshot s = core.snapshot();
            fprintf(stderr, "Timeout mode=%s phase=%u x=%f v=%f target=%f ref=%f rv=%f duty=%f\n",
                    axis::toString(s.mode), static_cast<unsigned>(s.homePhase), s.position,
                    s.velocity, s.target, s.referencePosition, s.referenceVelocity, s.output.duty);
        }
        CHECK(!core.isBusy());
    }
    void home() {
        CHECK(core.home() == Result::OK);
        finish();
        CHECK(core.isHomed());
        CHECK(core.inPosition());
        NEAR(core.getPosition(), config.home.parkPosition, config.positionTolerance);
    }
};

static void configuration() {
    axis::Core core;
    CHECK(core.moveBy(1) == Result::NOT_INITIALIZED);
    CHECK(core.home() == Result::NOT_INITIALIZED);
    CHECK(core.stop() == Result::NOT_INITIALIZED);
    CHECK(axis::Core::validConfig(axis::Config()));
    axis::Config c = testConfig();
    c.encoder.countsPerUnit = 0;
    CHECK(!axis::Core::validConfig(c));
    c = testConfig(); c.positionTolerance = 0.0001;
    CHECK(!axis::Core::validConfig(c));
    c = testConfig(); c.velocityKi = std::numeric_limits<double>::quiet_NaN();
    CHECK(!axis::Core::validConfig(c));
    c = testConfig(); c.home.source = axis::HomeSource::MIN_LIMIT; c.home.direction = 1;
    CHECK(!axis::Core::validConfig(c));
    c = testConfig(); c.maxTickGapUs = 0x80000000UL;
    CHECK(!axis::Core::validConfig(c));
    NEAR(axis::EncoderScale::rotary(2000, 5).countsPerUnit, 400, 1e-12);
    NEAR(axis::EncoderScale::rotary(1440).countsPerUnit, 4, 1e-12);
    CHECK(axis::EncoderScale::rotary(-100, -5).countsPerUnit == 0);
    c = testConfig(); c.acceleration = 17;
    CHECK(core.begin(c, 0, 0) == Result::OK);
    CHECK(core.begin(core.getConfig(), 0, 0) == Result::OK);
    NEAR(core.getConfig().acceleration, 17, 1e-12);
    c = testConfig(); c.maxJerk = -1;
    CHECK(!axis::Core::validConfig(c));
    c = testConfig(); c.velocitySetpointWeight = 1.1;
    CHECK(!axis::Core::validConfig(c));
    c = testConfig(); c.velocityDerivativeWeight = -0.1;
    CHECK(!axis::Core::validConfig(c));
}

static void profileInvariants() {
    axis::SCurveProfile profile;
    CHECK(!profile.planPosition(1, 1, 1, 1, 0));
    CHECK(!profile.planPosition(std::numeric_limits<double>::quiet_NaN(), 1, 1, 1, 1));
    CHECK(!profile.planVelocity(0, 0, 1, 1, 1, -1));
    const double lengths[] = {0, 0.00001, 0.1, 1, 50, -0.00001, -0.1, -50};
    const double speeds[] = {0.2, 2, 10};
    for (double length : lengths) for (double speed : speeds) {
        const double acceleration = 4, deceleration = 7, jerk = 30;
        CHECK(profile.planPosition(length, speed, acceleration, deceleration, jerk));
        const double duration = profile.duration();
        axis::MotionState start = profile.sample(0), end = profile.sample(duration);
        NEAR(start.position, 0, 1e-12); NEAR(start.velocity, 0, 1e-12); NEAR(start.acceleration, 0, 1e-12);
        NEAR(end.position, length, 1e-12); NEAR(end.velocity, 0, 1e-12); NEAR(end.acceleration, 0, 1e-12);
        if (duration == 0) continue;
        const double dt = duration / 2000;
        axis::MotionState previous = start;
        double sign = length < 0 ? -1.0 : 1.0;
        for (unsigned i = 1; i <= 2000; ++i) {
            axis::MotionState state = profile.sample(dt * i);
            CHECK(state.position * sign >= -1e-9 && state.position * sign <= fabs(length) + 1e-8);
            CHECK(state.velocity * sign >= -1e-9 && fabs(state.velocity) <= speed + 1e-8);
            CHECK(state.acceleration * sign <= acceleration + 1e-8);
            CHECK(state.acceleration * sign >= -deceleration - 1e-8);
            CHECK(fabs(state.jerk) <= jerk + 1e-8);
            CHECK(fabs(state.acceleration - previous.acceleration) <= jerk * dt + 1e-8);
            NEAR(state.position - previous.position,
                 0.5 * (state.velocity + previous.velocity) * dt, jerk * dt * dt * dt + 1e-8);
            previous = state;
        }
        NEAR(profile.sample(duration + 1).position, length, 1e-12);
        // Sampling is analytic: visiting the same time gives the same state.
        axis::MotionState middle = profile.sample(duration * 0.37);
        profile.sample(duration * 0.9);
        NEAR(profile.sample(duration * 0.37).position, middle.position, 1e-12);
    }
    const double initialVelocities[] = {-3, 0, 3};
    const double initialAccelerations[] = {-2, 0, 2};
    const double targets[] = {-2, 0, 2};
    for (double velocity : initialVelocities) for (double acceleration : initialAccelerations)
        for (double target : targets) {
            CHECK(profile.planVelocity(velocity, acceleration, target, 4, 7, 30));
            axis::MotionState initial = profile.sample(0);
            NEAR(initial.velocity, velocity, 1e-12);
            NEAR(initial.acceleration, acceleration, 1e-12);
            double total = profile.duration();
            axis::MotionState end = profile.sample(total);
            NEAR(end.velocity, target, 1e-12); NEAR(end.acceleration, 0, 1e-12);
            NEAR(profile.sample(total + 1).position, end.position + target, 1e-9);
            if (total == 0) continue;
            axis::MotionState previous = initial;
            const double dt = total / 1000;
            for (unsigned i = 1; i <= 1000; ++i) {
                axis::MotionState state = profile.sample(dt * i);
                CHECK(fabs(state.jerk) <= 30 + 1e-9);
                CHECK(fabs(state.acceleration) <= 7 + 1e-9);
                CHECK(fabs(state.acceleration - previous.acceleration) <= 30 * dt + 1e-8);
                NEAR(state.velocity - previous.velocity,
                     0.5 * (previous.acceleration + state.acceleration) * dt, 30 * dt * dt + 1e-8);
                previous = state;
            }
        }
}

static void smoothCoreAndStops() {
    axis::Config c = testConfig();
    c.maxJerk = 100;
    c.velocitySetpointWeight = 0.8;
    Plant p(c);
    p.home();
    CHECK(p.core.moveTo(10, 3) == Result::OK);
    double previousAcceleration = p.core.snapshot().referenceAcceleration;
    for (unsigned i = 0; i < 150 && p.core.isBusy(); ++i) {
        p.step();
        axis::Snapshot s = p.core.snapshot();
        CHECK(fabs(s.referenceAcceleration) <= c.acceleration + 1e-8);
        CHECK(fabs(s.referenceJerk) <= c.maxJerk + 1e-8);
        CHECK(fabs(s.referenceAcceleration - previousAcceleration) <= c.maxJerk * 0.005 + 1e-8);
        previousAcceleration = s.referenceAcceleration;
    }
    axis::Snapshot before = p.core.snapshot();
    CHECK(p.core.stop() == Result::OK);
    NEAR(p.core.snapshot().referenceVelocity, before.referenceVelocity, 1e-12);
    NEAR(p.core.snapshot().referenceAcceleration, before.referenceAcceleration, 1e-12);
    p.finish();
    CHECK(p.core.getCompletion() == axis::Completion::STOPPED);
    CHECK(p.core.getPosition() < 10);
    CHECK(p.core.jog(4) == Result::OK);
    p.finish(30);
    CHECK(p.core.getCompletion() == axis::Completion::STOPPED);
    CHECK(p.core.getPosition() <= c.softMax + c.positionTolerance);
    CHECK(p.core.getPosition() > c.softMax - 1.0);
    CHECK(p.core.moveTo(c.softMin, 3) == Result::OK);
    p.finish(30);
    NEAR(p.core.getPosition(), c.softMin, c.positionTolerance);
    CHECK(p.core.moveBy(0.01, 1) == Result::OK);
    p.finish();
    CHECK(p.core.moveTo(5, 3) == Result::OK);
    // Stop during positive acceleration, rather than only on a velocity plateau.
    p.run(0.08);
    before = p.core.snapshot();
    CHECK(before.referenceAcceleration > 0);
    CHECK(p.core.stop() == Result::OK);
    previousAcceleration = before.referenceAcceleration;
    for (unsigned i = 0; i < 400 && p.core.isBusy(); ++i) {
        p.step();
        axis::Snapshot s = p.core.snapshot();
        CHECK(fabs(s.referenceAcceleration - previousAcceleration) <= c.maxJerk * 0.005 + 1e-8);
        previousAcceleration = s.referenceAcceleration;
    }
    p.finish();
    p.core.disable();
    CHECK(p.core.setMaxJerk(200) == Result::OK);
    CHECK(p.core.setMaxJerk(-1) == Result::INVALID_ARGUMENT);
    CHECK(p.core.setMaxJerk(0) == Result::OK);
    CHECK(p.core.moveBy(1) == Result::OK);
    CHECK(p.core.setMaxJerk(200) == Result::BUSY);
    p.finish();
    const double jerks[] = {10, 100, 1000};
    for (double jerk : jerks) {
        axis::Config limits = testConfig();
        limits.maxJerk = jerk;
        limits.velocitySetpointWeight = 0.8;
        limits.softMax = 5;
        Plant bounded(limits);
        bounded.home();
        CHECK(bounded.core.jog(5) == Result::OK);
        bounded.finish(30);
        CHECK(bounded.core.getPosition() <= limits.softMax + limits.positionTolerance);
        CHECK(bounded.core.getPosition() > limits.softMax - 1.0);
        CHECK(bounded.core.jog(-5) == Result::OK);
        bounded.finish(30);
        CHECK(bounded.core.getPosition() >= limits.softMin - limits.positionTolerance);
        CHECK(bounded.core.getPosition() < limits.softMin + 1.0);
    }
    axis::Config sharedConfig = testConfig();
    sharedConfig.maxJerk = 100;
    sharedConfig.home.source = axis::HomeSource::MIN_LIMIT;
    Plant shared(sharedConfig, -0.2);
    shared.home();
    CHECK(shared.core.moveBy(3) == Result::OK);
    shared.run(0.05);
    CHECK(shared.core.snapshot().referenceAcceleration > 0);
    shared.core.emergencyStop();
    CHECK(shared.core.getFault() == Fault::EMERGENCY_STOP);
    CHECK(shared.core.getOutput().duty == 0);
    CHECK(shared.core.getOutput().action == axis::OutputAction::COAST);
}

static void twoDofController() {
    axis::Config c = testConfig();
    c.maxJerk = 100;
    c.velocityKp = 0.1;
    c.velocityKi = 0;
    c.velocityKd = 0;
    c.velocityFeedforward = 0;
    c.velocitySetpointWeight = 0.4;
    c.velocityDerivativeWeight = 0;
    c.derivativeFilterTime = 0;
    c.maxFollowingError = c.stallOutput = 0;
    axis::Core core;
    CHECK(core.begin(c, 0, 0) == Result::OK);
    CHECK(core.moveBy(2, 1) == Result::OK);
    for (uint32_t now = 5000; now <= 100000; now += 5000) {
        core.tick(now, 0);
        axis::Snapshot s = core.snapshot();
        NEAR(s.output.duty, c.velocityKp * (c.velocitySetpointWeight * s.velocityCommand - s.velocity), 1e-12);
    }
    axis::Core measurementD, weightedD;
    c.velocityKd = 0.002;
    CHECK(measurementD.begin(c, 0, 0) == Result::OK);
    c.velocityDerivativeWeight = 1;
    CHECK(weightedD.begin(c, 0, 0) == Result::OK);
    CHECK(measurementD.moveBy(2, 1) == Result::OK);
    CHECK(weightedD.moveBy(2, 1) == Result::OK);
    double previousCommand = 0;
    for (uint32_t now = 5000; now <= 100000; now += 5000) {
        measurementD.tick(now, 0); weightedD.tick(now, 0);
        double command = weightedD.snapshot().velocityCommand;
        NEAR(measurementD.getOutput().duty, c.velocityKp * c.velocitySetpointWeight * command, 1e-12);
        NEAR(weightedD.getOutput().duty - measurementD.getOutput().duty,
             c.velocityKd * (command - previousCommand) / 0.005, 1e-12);
        previousCommand = command;
    }
    c.velocityKp = 0.001;
    c.velocityKd = 0.00001;
    c.velocityFilterTime = 0;
    c.velocityDerivativeWeight = 0.3;
    axis::Core measured;
    CHECK(measured.begin(c, 0, 0) == Result::OK);
    CHECK(measured.moveBy(2, 1) == Result::OK);
    const uint32_t counts[] = {0, 1, 3, 3, 2, 4};
    double previousMeasured = 0;
    previousCommand = 0;
    for (unsigned i = 0; i < sizeof(counts) / sizeof(counts[0]); ++i) {
        measured.tick((i + 1) * 5000, counts[i]);
        axis::Snapshot s = measured.snapshot();
        double expected = c.velocityKp * (c.velocitySetpointWeight * s.velocityCommand - s.velocity) +
            c.velocityKd * (c.velocityDerivativeWeight * (s.velocityCommand - previousCommand) -
                            (s.velocity - previousMeasured)) / 0.005;
        NEAR(s.output.duty, expected, 1e-12);
        previousCommand = s.velocityCommand;
        previousMeasured = s.velocity;
    }
}

static void relativeAndProfiles() {
    Plant p;
    CHECK(p.core.moveTo(1) == Result::NOT_HOMED);
    CHECK(p.core.jog(1) == Result::NOT_HOMED);
    CHECK(p.core.moveBy(11) == Result::OUT_OF_RANGE);
    CHECK(p.core.moveBy(2) == Result::OK);
    CHECK(p.core.moveBy(1) == Result::BUSY);
    double last = 0;
    for (int i = 0; i < 100 && p.core.isBusy(); ++i) {
        p.step();
        double v = p.core.snapshot().referenceVelocity;
        CHECK(fabs(v) <= p.config.unhomedMaxSpeed + 1e-9);
        CHECK(fabs(v - last) <= p.config.acceleration * 0.005 + 1e-8);
        last = v;
    }
    p.finish();
    NEAR(p.core.getPosition(), 2, 0.02);
    CHECK(p.core.getCompletion() == axis::Completion::REACHED);
    CHECK(p.core.moveBy(-1) == Result::OK);
    p.finish();
    NEAR(p.core.getPosition(), 1, 0.02);
    CHECK(p.core.moveBy(std::numeric_limits<double>::infinity()) == Result::INVALID_ARGUMENT);
    p.core.disable();
    CHECK(p.core.getCompletion() == axis::Completion::CANCELLED);
}

static void homingAndCoordinates() {
    Plant p;
    p.home();
    CHECK(p.seen[static_cast<unsigned>(axis::HomePhase::SEARCH)]);
    CHECK(p.seen[static_cast<unsigned>(axis::HomePhase::BACKOFF)]);
    CHECK(p.seen[static_cast<unsigned>(axis::HomePhase::LATCH)]);
    CHECK(p.seen[static_cast<unsigned>(axis::HomePhase::PARK)]);
    NEAR(p.position, 1, 0.04);
    CHECK(p.core.setWorkPosition(10) == Result::OK);
    NEAR(p.core.getWorkPosition(), 10, 1e-9);
    CHECK(p.core.moveTo(12, 2, axis::Frame::WORK) == Result::OK);
    p.finish();
    NEAR(p.core.getWorkPosition(), 12, 0.02);
    CHECK(p.core.moveTo(-1) == Result::OUT_OF_RANGE);
    CHECK(p.core.moveBy(100) == Result::OUT_OF_RANGE);
    CHECK(p.core.moveTo(15, 3) == Result::OK);
    p.finish();
    NEAR(p.core.getPosition(), 15, 0.02);
    p.core.disable();
    CHECK(p.core.isHomed());
    p.core.invalidateReference();
    CHECK(!p.core.isHomed());
    CHECK(p.core.moveTo(1) == Result::NOT_HOMED);

    axis::Config shared = testConfig();
    shared.home.source = axis::HomeSource::MIN_LIMIT;
    Plant s(shared, -0.2);
    s.home();
    CHECK(s.seen[static_cast<unsigned>(axis::HomePhase::RELEASE)]);
    CHECK(!s.inputs.minLimit);
    NEAR(s.position, 1, 0.04);

    axis::Config upper = testConfig();
    upper.home.source = axis::HomeSource::MAX_LIMIT;
    upper.home.direction = 1;
    upper.home.switchPosition = 21;
    upper.home.parkPosition = 20;
    Plant m(upper, 23.2);
    m.home();
    CHECK(!m.inputs.maxLimit);
    NEAR(m.position, 22, 0.04);
}

static void stopAndSoftLimits() {
    Plant p;
    p.home();
    CHECK(p.core.moveTo(15) == Result::OK);
    p.run(0.4);
    CHECK(p.core.stop() == Result::OK);
    p.finish();
    CHECK(p.core.getCompletion() == axis::Completion::STOPPED);
    CHECK(p.core.getPosition() < 15);
    CHECK(p.core.jog(5) == Result::OK);
    p.finish(25);
    CHECK(p.core.getCompletion() == axis::Completion::STOPPED);
    CHECK(p.core.getPosition() <= p.config.softMax + p.config.positionTolerance);
    CHECK(p.core.getPosition() > p.config.softMax - 0.2);
    CHECK(p.core.moveBy(-2, 2) == Result::OK);
    p.finish();
    CHECK(p.core.moveTo(p.config.softMax, 3) == Result::OK);
    p.finish();
    NEAR(p.core.getPosition(), p.config.softMax, p.config.positionTolerance);
}

static void hardLimitsAndRecovery() {
    Plant p;
    p.home();
    CHECK(p.core.moveTo(10) == Result::OK);
    p.run(0.2);
    p.switches = false;
    p.inputs.minLimit = true;
    // A limit is acted on even before the next control period.
    p.step(1);
    CHECK(p.core.getFault() == Fault::MIN_LIMIT);
    CHECK(p.core.getOutput().action == axis::OutputAction::COAST);
    CHECK(p.core.clearFault() == Result::FAULT_ACTIVE);
    CHECK(p.core.moveBy(1) == Result::FAULT_ACTIVE);
    p.run(0.8);
    CHECK(p.core.recoverFromLimit(-1, 1) == Result::LIMIT_ACTIVE);
    CHECK(p.core.recoverFromLimit(1, 1) == Result::OK);
    p.run(0.2);
    CHECK(p.core.getOutput().duty >= 0);
    p.inputs.minLimit = false;
    p.finish();
    CHECK(p.core.isHomed());
    p.inputs.minLimit = p.inputs.maxLimit = true;
    p.step(1);
    CHECK(p.core.getFault() == Fault::BOTH_LIMITS);
}

static void faultsAndTiming() {
    Plant p;
    CHECK(p.core.moveBy(3) == Result::OK);
    p.step(60000);
    CHECK(p.core.getFault() == Fault::CONTROL_TIMEOUT);
    CHECK(p.core.getCompletion() == axis::Completion::FAULTED);
    p.run(1);
    CHECK(p.core.clearFault() == Result::OK);
    CHECK(p.core.getMode() == axis::Mode::IDLE);
    CHECK(!p.core.isHomed());
    CHECK(p.core.getOutput().duty == 0);
    p.inputs.encoderValid = false;
    p.step(1);
    CHECK(p.core.getFault() == Fault::ENCODER_INVALID);
    CHECK(p.core.clearFault() == Result::FAULT_ACTIVE);

    Plant e;
    e.home();
    e.inputs.emergency = true;
    e.step(1);
    CHECK(e.core.getFault() == Fault::EMERGENCY_STOP);
    CHECK(!e.core.isHomed());
    e.inputs.emergency = false;
    e.run(1);
    CHECK(e.core.clearFault() == Result::OK);
    CHECK(e.core.getOutput().action == axis::OutputAction::COAST);

    axis::Config c = testConfig();
    c.maxFollowingError = 0.1; c.followingErrorTimeUs = 10000; c.stallOutput = 0;
    Plant f(c); f.frozen = true;
    CHECK(f.core.moveBy(2) == Result::OK);
    f.run(1);
    CHECK(f.core.getFault() == Fault::FOLLOWING_ERROR);

    c = testConfig(); c.maxFollowingError = 0; c.stallOutput = 0.1; c.stallTimeUs = 100000;
    Plant s(c); s.frozen = true;
    CHECK(s.core.moveBy(3) == Result::OK);
    s.run(2);
    CHECK(s.core.getFault() == Fault::STALL);
    CHECK(fabs(s.core.snapshot().integral) <= c.maxOutput);

    c = testConfig(); c.home.timeoutUs = 100000; c.stallOutput = 0;
    Plant h(c); h.frozen = true;
    CHECK(h.core.home() == Result::OK);
    h.run(0.2);
    CHECK(h.core.getFault() == Fault::HOMING_TIMEOUT);

    c = testConfig(); c.home.maxTravel = 0.1;
    Plant travel(c);
    CHECK(travel.core.home() == Result::OK);
    travel.run(1);
    CHECK(travel.core.getFault() == Fault::HOMING_TRAVEL);
}

static void counterWrapAndEncoder() {
    axis::Core core;
    axis::Config c = testConfig();
    CHECK(core.begin(c, 0xfffffff0UL, 0xfffffff0UL) == Result::OK);
    core.tick(0x1378, 0x10);
    CHECK(core.getCounts() == 32);
    NEAR(core.getPosition(), 0.032, 1e-9);
    core.tick(0x2700, 0xfffffff0UL);
    CHECK(core.getCounts() == 0);
    c.encoder.inverted = true;
    CHECK(core.begin(c, 0, 0) == Result::OK);
    core.tick(5000, 100);
    NEAR(core.getPosition(), -0.1, 1e-9);
    c.maxEncoderRate = 100;
    CHECK(core.begin(c, 0, 0) == Result::OK);
    core.tick(5000, 100);
    CHECK(core.getFault() == Fault::ENCODER_RATE);

    axis::Quadrature q;
    q.begin(false, false);
    q.update(true, false); q.update(true, true); q.update(false, true); q.update(false, false);
    CHECK(q.readUnsafe() == 4);
    q.update(false, true); q.update(true, true); q.update(true, false); q.update(false, false);
    CHECK(q.readUnsafe() == 0);
    q.update(true, true);
    CHECK(q.invalidTransitionsUnsafe() == 1);
    CHECK(q.readUnsafe() == 0);
}

static void completionAndHolding() {
    axis::Config c = testConfig();
    c.endAction = axis::EndAction::COAST;
    Plant p(c);
    CHECK(p.core.moveBy(1) == Result::OK);
    p.finish();
    CHECK(p.core.inPosition());
    CHECK(p.core.getOutput().action == axis::OutputAction::COAST);
    p.position += 0.2;
    p.step();
    CHECK(!p.core.inPosition());
    CHECK(p.core.getMode() == axis::Mode::IDLE);

    Plant hold;
    CHECK(hold.core.moveBy(1) == Result::OK);
    hold.finish();
    hold.position += 0.2;
    hold.step();
    CHECK(!hold.core.inPosition());
    hold.run(3);
    CHECK(hold.core.inPosition());
    NEAR(hold.core.getPosition(), 1, 0.02);

    axis::Core numeric;
    c = testConfig(); c.velocityKp = std::numeric_limits<double>::max();
    CHECK(numeric.begin(c, 0, 0) == Result::OK);
    CHECK(numeric.moveBy(1) == Result::OK);
    for (uint32_t t = 5000; t < 500000 && numeric.getFault() == Fault::NONE; t += 5000)
        numeric.tick(t, 0);
    CHECK(numeric.getFault() == Fault::NUMERIC_ERROR);
    CHECK(numeric.getOutput().duty == 0);
}

static void stopDuringRecovery() {
    Plant p;
    p.switches = false;
    p.inputs.minLimit = true;
    p.step();
    CHECK(p.core.recoverFromLimit(1, 2) == Result::OK);
    p.run(0.2);
    CHECK(p.core.stop() == Result::OK);
    p.run(0.1);
    CHECK(p.core.getMode() == axis::Mode::STOPPING);
    CHECK(p.core.getFault() == Fault::NONE);
    CHECK(p.core.getOutput().duty >= 0);
    p.inputs.minLimit = false;
    p.finish();
    CHECK(p.core.getCompletion() == axis::Completion::STOPPED);

    axis::Config stuckConfig = testConfig();
    stuckConfig.positionTolerance = 0.001;
    stuckConfig.maxFollowingError = 0;
    stuckConfig.stallOutput = 0;
    Plant stuck(stuckConfig);
    stuck.switches = false;
    stuck.inputs.minLimit = true;
    stuck.step();
    CHECK(stuck.core.recoverFromLimit(1, 1) == Result::OK);
    stuck.run(3);
    CHECK(stuck.core.getFault() == Fault::MIN_LIMIT);
    CHECK(!stuck.core.isBusy());
}

static void rotary() {
    axis::Config c = testConfig();
    c.kind = axis::Kind::ROTARY; c.continuous = true; c.softLimitsEnabled = false;
    c.home.switchPosition = 350; c.home.parkPosition = 350;
    Plant p(c);
    p.home();
    CHECK(p.core.moveToAngle(10) == Result::OK);
    NEAR(p.core.getTarget(), 370, 1e-9);
    p.finish();
    NEAR(p.core.getAngle(), 10, 0.02);
    CHECK(p.core.moveToAngle(350, axis::Direction::NEGATIVE) == Result::OK);
    NEAR(p.core.getTarget(), 350, 1e-9);
    p.finish();
    CHECK(p.core.moveToAngle(170, axis::Direction::SHORTEST) == Result::OK);
    // Within measured-position tolerance, half-turn choice depends on actual angle;
    // exact tie behavior is tested with an exact snapshot below.
    p.core.disable();

    axis::Core exact;
    CHECK(exact.begin(c, 0, 0) == Result::OK);
    CHECK(exact.moveToAngle(10) == Result::NOT_HOMED);
    CHECK(p.core.moveBy(720, 8) == Result::OK);
    NEAR(p.core.getTarget() - p.core.getPosition(), 720, 1e-9);
    p.core.disable();
}

struct ReaderData { uint32_t count = 0; axis::Inputs inputs; };
static uint32_t readCount(void* context) { return static_cast<ReaderData*>(context)->count; }
static axis::Inputs readInputs(void* context) { return static_cast<ReaderData*>(context)->inputs; }

static void absoluteAndRelativePositions() {
    axis::Core core;
    CHECK(core.setRelativePosition() == Result::NOT_INITIALIZED);
    CHECK(core.setRelativeOrigin(1) == Result::NOT_INITIALIZED);
    CHECK(core.setRelativeOriginCounts(1) == Result::NOT_INITIALIZED);
    CHECK(core.begin(testConfig(), 0, 0) == Result::OK);
    core.tick(1, 123);
    NEAR(core.getAbsolutePosition(), 0.123, 1e-12);
    CHECK(core.getAbsolutePositionCounts() == 123);
    CHECK(core.setRelativeOrigin(-2) == Result::NOT_HOMED);
    CHECK(core.setRelativeOriginCounts(-2000) == Result::NOT_HOMED);
    CHECK(core.setRelativePosition() == Result::OK);
    NEAR(core.getRelativePosition(), 0, 1e-12);
    CHECK(core.getRelativePositionCounts() == 0);
    core.tick(2, 124);
    NEAR(core.getRelativePosition(), 0.001, 1e-12);
    CHECK(core.getRelativePositionCounts() == 1);
    core.tick(3, 100);
    NEAR(core.getRelativePosition(), -0.023, 1e-12);
    CHECK(core.getRelativePositionCounts() == -23);

    Plant p;
    p.home();
    const double absolute = p.core.getAbsolutePosition();
    const int64_t pulses = p.core.getAbsolutePositionCounts();
    CHECK(p.core.getCounts() != pulses);  // Home offset differs from raw startup count.
    CHECK(p.core.setRelativeOrigin(20) == Result::OK);
    NEAR(p.core.getAbsolutePosition(), absolute, 1e-12);
    NEAR(p.core.getRelativePosition(), absolute - 20, 1e-12);
    CHECK(p.core.getRelativePosition() < 0);
    CHECK(p.core.getRelativePositionCounts() == pulses - 20000);
    CHECK(p.core.setRelativeOrigin(-10) == Result::OK);
    NEAR(p.core.getRelativePosition(), absolute + 10, 1e-12);
    CHECK(p.core.getRelativePosition() > 0);
    CHECK(p.core.getRelativePositionCounts() == pulses + 10000);
    CHECK(p.core.setRelativeOriginCounts(-2500) == Result::OK);
    NEAR(p.core.getRelativePosition(), absolute + 2.5, 1e-12);
    CHECK(p.core.getRelativePositionCounts() == pulses + 2500);

    // Integer origins preserve an individual pulse beyond double precision.
    const int64_t largeOrigin = 9007199254740993LL;
    CHECK(p.core.setRelativeOriginCounts(largeOrigin) == Result::OK);
    CHECK(p.core.getRelativePositionCounts() == pulses - largeOrigin);
    CHECK(p.core.setRelativeOriginCounts(INT64_MIN) == Result::OK);
    CHECK(p.core.getRelativePositionCounts() == (pulses >= 0 ? INT64_MAX : (INT64_MAX + pulses) + 1));
    CHECK(p.core.setRelativePosition(-3.25) == Result::OK);
    NEAR(p.core.getRelativePosition(), -3.25, 1e-12);
    CHECK(p.core.getRelativePositionCounts() == -3250);
    CHECK(p.core.setRelativePosition(5) == Result::OK);
    CHECK(p.core.getRelativePositionCounts() == 5000);
    CHECK(p.core.setRelativePosition() == Result::OK);
    CHECK(p.core.getRelativePositionCounts() == 0);
    CHECK(p.core.setWorkPosition(1.5) == Result::OK);
    NEAR(p.core.getRelativePosition(), p.core.getWorkPosition(), 1e-12);
    CHECK(p.core.getRelativePositionCounts() == 1500);
    CHECK(p.core.setRelativePosition(0.0006) == Result::OK);
    NEAR(p.core.getRelativePosition(), 0.0006, 1e-12);
    CHECK(p.core.getRelativePositionCounts() == 1);  // Quantized to one count.
    CHECK(p.core.setRelativeOrigin(0.0006) == Result::OK);
    CHECK(p.core.getRelativePositionCounts() == pulses - 1);
    CHECK(p.core.setRelativeOrigin(std::numeric_limits<double>::infinity()) == Result::INVALID_ARGUMENT);
    CHECK(p.core.setRelativePosition(std::numeric_limits<double>::quiet_NaN()) == Result::INVALID_ARGUMENT);
    CHECK(p.core.setRelativeOrigin(1e30) == Result::INVALID_ARGUMENT);
    CHECK(p.core.setRelativePosition(1e30) == Result::INVALID_ARGUMENT);
    CHECK(p.core.getRelativePositionCounts() == pulses - 1);  // Failed writes are atomic.

    CHECK(p.core.setWorkPosition(1.5) == Result::OK);
    CHECK(p.core.moveTo(2.5, 1, axis::Frame::WORK) == Result::OK);
    NEAR(p.core.getTarget(), absolute + 1, 1e-12);
    CHECK(p.core.setRelativePosition() == Result::BUSY);
    CHECK(p.core.setRelativeOrigin(-1) == Result::BUSY);
    CHECK(p.core.setRelativeOriginCounts(-1000) == Result::BUSY);
    p.finish();
    NEAR(p.core.getRelativePosition(), 2.5, p.config.positionTolerance);
    CHECK(p.core.getAbsolutePositionCounts() == llround(p.core.getAbsolutePosition() * 1000));
    axis::Snapshot s = p.core.snapshot();
    CHECK(s.absolutePositionCounts == p.core.getAbsolutePositionCounts());
    CHECK(s.relativePositionCounts == p.core.getRelativePositionCounts());
    CHECK(p.core.moveTo(100) == Result::OUT_OF_RANGE);  // Origin never changes soft limits.
    CHECK(p.core.setRelativeOrigin(-10) == Result::OK);
    p.home();
    CHECK(p.core.getRelativePositionCounts() == p.core.getAbsolutePositionCounts());
    NEAR(p.core.getRelativePosition(), p.core.getAbsolutePosition(), 1e-12);
    CHECK(p.core.setRelativeOrigin(2) == Result::OK);
    p.core.invalidateReference();
    CHECK(p.core.getRelativePositionCounts() == p.core.getAbsolutePositionCounts());

    axis::Config inverted = testConfig();
    inverted.encoder.inverted = true;
    CHECK(core.begin(inverted, 0, 0xfffffff0UL) == Result::OK);
    core.tick(1, 0x10);
    CHECK(core.getCounts() == 32);
    CHECK(core.getAbsolutePositionCounts() == -32);
    NEAR(core.getAbsolutePosition(), -0.032, 1e-12);
    CHECK(core.setRelativePosition() == Result::OK);
    core.tick(2, 0x12);
    CHECK(core.getRelativePositionCounts() == -2);
    NEAR(core.getRelativePosition(), -0.002, 1e-12);

    Plant invertedPlant(inverted);
    invertedPlant.home();
    CHECK(invertedPlant.core.getAbsolutePositionCounts() == llround(invertedPlant.core.getAbsolutePosition() * 1000));
    CHECK(invertedPlant.core.setRelativeOriginCounts(-1000) == Result::OK);
    CHECK(invertedPlant.core.getRelativePositionCounts() == invertedPlant.core.getAbsolutePositionCounts() + 1000);

    CHECK(core.begin(testConfig(), 0, 0) == Result::OK);
    core.tick(1, 0x7ffffffeUL);
    core.tick(2, 0x80000000UL);
    CHECK(core.getAbsolutePositionCounts() == 2147483648LL);
    CHECK(core.setRelativePosition() == Result::OK);
    core.tick(3, 0x80000001UL);
    CHECK(core.getRelativePositionCounts() == 1);
}

static void realGyverAdapter() {
    ReaderData reader;
    Axis<GM2::DIR_PWM> motor(4, 5);
    CHECK(motor.begin(testConfig()) == Result::INVALID_ARGUMENT);
    CHECK(motor.attachEncoder(readCount, &reader) == Result::OK);
    CHECK(motor.attachInputs(readInputs, &reader) == Result::OK);
    CHECK(motor.begin(testConfig()) == Result::OK);
    reader.count = 123;
    motor.tick();
    CHECK(motor.getAbsolutePositionCounts() == 123);
    NEAR(motor.getAbsolutePosition(), 0.123, 1e-12);
    CHECK(motor.setRelativePosition() == Result::OK);
    reader.count = 100;
    motor.tick();
    CHECK(motor.getRelativePositionCounts() == -23);
    NEAR(motor.getRelativePosition(), -0.023, 1e-12);
    CHECK(motor.setRelativeOrigin(1) == Result::NOT_HOMED);
    CHECK(motor.setRelativeOriginCounts(1000) == Result::NOT_HOMED);
    reader.count = 0;
    motor.tick();
    CHECK(motor.setRelativePosition() == Result::OK);
    CHECK(motor.moveBy(1) == Result::OK);
    for (unsigned i = 0; i < 20; ++i) { mockMicros += 5000; motor.tick(); }
    CHECK(mockPwm[5] > 0);
    CHECK(mockDigital[4] == LOW);
    reader.inputs.emergency = true;
    // Command refresh polls inputs before accepting another command.
    CHECK(motor.moveBy(1) == Result::FAULT_ACTIVE);
    CHECK(mockPwm[5] == 0);
    CHECK(motor.getFault() == Fault::EMERGENCY_STOP);
    motor.brake(); CHECK(mockPwm[5] == 255);
    motor.disable(); CHECK(mockPwm[5] == 0);
    CHECK(motor.getFault() == Fault::EMERGENCY_STOP);
    Axis<GM2::ONLY_PWM> unsupported(6);
    CHECK(unsupported.begin(testConfig()) == Result::UNSUPPORTED);
    Axis<GM2::DIR_DIR> relay(6, 7);
    CHECK(relay.begin(testConfig()) == Result::UNSUPPORTED);

    Axis<GM2::DIR_PWM_INV> inverted(8, 9);
    Axis<GM2::PWM_PWM_SPEED> speed(10, 11);
    Axis<GM2::PWM_PWM_POWER> power(12, 13);
    Axis<GM2::DIR_DIR_PWM, 10> three(14, 15, 16);
    CHECK(inverted.attachEncoder(readCount, &reader) == Result::OK);
    CHECK(speed.attachEncoder(readCount, &reader) == Result::OK);
    CHECK(power.attachEncoder(readCount, &reader) == Result::OK);
    CHECK(three.attachEncoder(readCount, &reader) == Result::OK);
}

#include "test_sync.h"

int main() {
    synchronizationTests();
    profileInvariants(); smoothCoreAndStops(); twoDofController();
    configuration(); relativeAndProfiles(); homingAndCoordinates(); stopAndSoftLimits();
    hardLimitsAndRecovery(); faultsAndTiming(); counterWrapAndEncoder(); completionAndHolding();
    stopDuringRecovery(); rotary(); absoluteAndRelativePositions(); realGyverAdapter();
    printf("PASS: %d checks; sizeof(axis::Core)=%zu bytes on host\n", checks, sizeof(axis::Core));
    return 0;
}
