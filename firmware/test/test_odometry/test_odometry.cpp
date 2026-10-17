// Host tests of the odometry: the kinematic fit on the gait's own feet and stance flags, and the fused heading as
// MotionService feeds it, through the virtual robot (firmware/host/virtual_robot.h).

#include <unity.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <odometry.h>
#include <virtual_robot.h>

namespace {

constexpr float DT = 0.005f;
constexpr int TICKS_PER_SECOND = 200;
constexpr int COMMAND_PERIOD_TICKS = 4; // the app's 20 ms stick period
constexpr float TWO_PI = 2.0f * static_cast<float>(M_PI);
constexpr std::array<bool, 6> ALL_DOWN {true, true, true, true, true, true};

int ticksFor(float seconds) { return static_cast<int>(std::lround(seconds * TICKS_PER_SECOND)); }

// The pose `end` reached from `start`, in the frame the robot faced at `start`.
OdometryEstimate relative(const OdometryEstimate &start, const OdometryEstimate &end) {
    const float dx = end.x - start.x, dy = end.y - start.y, c = std::cos(start.yaw), s = std::sin(start.yaw);
    return {.x = c * dx + s * dy,
            .y = -s * dx + c * dy,
            .yaw = std::remainder(end.yaw - start.yaw, TWO_PI),
            .kinematicYaw = std::remainder(end.kinematicYaw - start.kinematicYaw, TWO_PI),
            .distance = end.distance - start.distance};
}

// The gait's resting stance: with no stroke and no lift every foot stands on its default position.
BodyStateMsg restingStance(GaitController &gait, gait_state_t still) {
    still.step_x = still.step_y = still.step_angle = still.step_height = 0;
    BodyStateMsg body {};
    gait.generateFeet(still, body);
    return body;
}

constexpr int SAMPLE_TICKS = 5; // the gyro's 40 Hz in the 200 Hz control loop

// A tripod walked by the gait alone at one cycle per second, its feet and stance flags fed to the odometries as
// MotionService feeds them, after each step; the fused one gets `gyro` on every fifth tick when it is set.
struct GaitWalk {
    GaitController gait;
    gait_state_t state {};
    BodyStateMsg body {};
    KinematicOdometry odometry;
    FusedOdometry fused;
    std::optional<float> gyro;
    int ticksRun = 0;

    GaitWalk(float stepX, float stepY, float stepAngle) {
        state.gait_type = GaitType::TRI_GATE;
        state.step_speed = 1;
        state.step_height = 15;
        state.step_depth = 0.002f;
        state.phase_rate = 1;
        gait.setGait(state);
        body = restingStance(gait, state);
        state.step_x = stepX;
        state.step_y = stepY;
        state.step_angle = stepAngle;
    }

    void run(int ticks) {
        for (int t = 0; t < ticks; ++t) {
            gait.step(state, body, DT);
            std::array<bool, 6> stance;
            for (int leg = 0; leg < 6; ++leg) stance[leg] = gait.inStance(state, leg);
            odometry.update(body, stance);
            fused.update(body, stance, ++ticksRun % SAMPLE_TICKS == 0 ? gyro : std::nullopt, DT);
        }
    }

    // The pose over `cycles` whole cycles, once the start's carry has let go (FEET_EASE_S).
    OdometryEstimate walkCycles(int cycles) {
        run(ticksFor(FEET_EASE_S));
        const OdometryEstimate start = odometry.estimate();
        run(cycles * TICKS_PER_SECOND);
        return relative(start, odometry.estimate());
    }

    // A stance foot strokes its whole stroke in stand_frac of the cycle and some foot is always down, so the body
    // moves stroke / stand_frac per cycle.
    float perCycle(float stroke) const { return stroke / state.stand_frac; }
};

constexpr int CYCLES = 4;
constexpr float STEP_MM = 30.0f;
constexpr float STEP_ANGLE_RAD = 0.2f;
// A gyro bias of the size a boot calibration leaves when the robot moved through it (Leika's BIAS_PRIOR scale).
constexpr float GYRO_BIAS = 0.02f;

// The gyro bias `odometry` learned, read off its heading on a copy: with no foot down the heading is the gyro's less
// the bias, so on a gyro reading zero it turns by -bias per second, and without feet it learns nothing more.
float learnedBias(FusedOdometry odometry) {
    const BodyStateMsg body {};
    const float start = odometry.estimate().yaw;
    for (int t = 1; t <= TICKS_PER_SECOND; ++t)
        odometry.update(body, {}, t % SAMPLE_TICKS == 0 ? std::optional<float>(0.0f) : std::nullopt, DT);
    return -std::remainder(odometry.estimate().yaw - start, TWO_PI);
}

} // namespace

void setUp() {}
// The IMU stand-in is process-wide, so a test that feeds it a gyro rate must not hand the rate on, even when it fails.
void tearDown() { Peripherals::setImu({}); }

void test_a_straight_walk_lands_on_the_commanded_displacement() {
    GaitWalk walk(0, STEP_MM, 0);
    const OdometryEstimate leg = walk.walkCycles(CYCLES);
    const float expected = CYCLES * walk.perCycle(STEP_MM) / 1000.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f * expected, expected, leg.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, leg.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, leg.yaw);
    TEST_ASSERT_FLOAT_WITHIN(0.01f * expected, expected, leg.distance);
}

void test_a_sideways_walk_lands_on_the_commanded_displacement() {
    GaitWalk walk(STEP_MM, 0, 0);
    const OdometryEstimate leg = walk.walkCycles(CYCLES);
    const float expected = CYCLES * walk.perCycle(STEP_MM) / 1000.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f * expected, expected, leg.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, leg.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, leg.yaw);
}

// Each foot strokes along a straight chord tangent to its circle, so its bearing turns a little slower toward the
// chord's ends: by at most 1 % at this step angle (d/dp atan(step_angle (1 - 2p) / 2)).
void test_a_turn_in_place_lands_on_the_commanded_yaw() {
    GaitWalk walk(0, 0, STEP_ANGLE_RAD);
    const OdometryEstimate leg = walk.walkCycles(CYCLES);
    const float expected = CYCLES * walk.perCycle(STEP_ANGLE_RAD);
    TEST_ASSERT_GREATER_THAN_FLOAT(1.0f, expected); // far enough for a mirrored or halved turn to show
    TEST_ASSERT_FLOAT_WITHIN(0.01f * expected, expected, leg.yaw);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, leg.yaw, leg.kinematicYaw);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0, std::hypot(leg.x, leg.y));
}

// A forward stroke with a counter-clockwise turn walks an arc to the left, of radius step_y / step_angle.
void test_an_arc_lands_on_the_commanded_pose() {
    GaitWalk walk(0, STEP_MM, STEP_ANGLE_RAD);
    const OdometryEstimate leg = walk.walkCycles(CYCLES);
    const float yaw = CYCLES * walk.perCycle(STEP_ANGLE_RAD);
    const float radius = STEP_MM / STEP_ANGLE_RAD / 1000.0f;
    const float x = -radius * (1 - std::cos(yaw)), y = radius * std::sin(yaw);
    const float path = radius * yaw;
    TEST_ASSERT_FLOAT_WITHIN(0.01f * yaw, yaw, leg.yaw);
    TEST_ASSERT_FLOAT_WITHIN(0.01f * path, x, leg.x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f * path, y, leg.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f * path, path, leg.distance);
}

// A foot the stance flags call down but that hangs 3 mm or more above the lowest, as a stopped walk eases one down,
// carries nothing however it moves; below that, it counts.
void test_a_foot_lifted_3_mm_or_more_is_ignored() {
    BodyStateMsg body = GaitWalk(0, 0, 0).body;
    KinematicOdometry odometry;
    body.feet[0][2] += KinematicOdometry::LIFTED_MM;
    odometry.update(body, ALL_DOWN);
    body.feet[0][0] += 20.0f;
    body.feet[0][1] += 20.0f;
    odometry.update(body, ALL_DOWN);
    TEST_ASSERT_EQUAL_FLOAT(0, odometry.estimate().x);
    TEST_ASSERT_EQUAL_FLOAT(0, odometry.estimate().y);
    TEST_ASSERT_EQUAL_FLOAT(0, odometry.estimate().yaw);

    body.feet[0][2] -= 0.1f;
    odometry.update(body, ALL_DOWN);
    body.feet[0][1] += 20.0f;
    odometry.update(body, ALL_DOWN);
    TEST_ASSERT_LESS_THAN_FLOAT(-0.002f, odometry.estimate().y);
}

// While the gait cycles, a leg is in stance for the first stand_frac of its own cycle. A gait that stopped cycling
// eases every foot home on the floor and swings none, so the ease of a stopped walk is counted over all the feet down.
void test_every_foot_is_in_stance_once_the_gait_stops_cycling() {
    GaitWalk walk(0, STEP_MM, 0);
    walk.run(ticksFor(0.3f));
    int down = 0;
    for (int leg = 0; leg < 6; ++leg) {
        const bool stance = std::fmod(walk.gait.getPhase() + walk.state.offset[leg], 1.0f) < walk.state.stand_frac;
        TEST_ASSERT_EQUAL(stance, walk.gait.inStance(walk.state, leg));
        down += stance;
    }
    TEST_ASSERT_EQUAL(3, down);
    walk.state.step_y = 0;
    walk.run(1);
    for (int leg = 0; leg < 6; ++leg) TEST_ASSERT_TRUE(walk.gait.inStance(walk.state, leg));
}

// Paused, the odometry is not shown the feet; where it finds them next is no travel.
void test_feet_moved_while_paused_are_no_travel() {
    BodyStateMsg body = GaitWalk(0, 0, 0).body;
    KinematicOdometry odometry;
    odometry.update(body, ALL_DOWN);
    odometry.pause();
    for (auto &foot : body.feet) foot[1] += 20.0f;
    odometry.update(body, ALL_DOWN);
    odometry.update(body, ALL_DOWN);
    TEST_ASSERT_EQUAL_FLOAT(0, odometry.estimate().y);
    TEST_ASSERT_EQUAL_FLOAT(0, odometry.estimate().distance);
}

// With no foot down to see the turn, the heading is the gyro's, each sample integrated over the time since the last.
// A sample after a gap longer than GYRO_STALE_S, a stalled read, does not stand for the gap.
void test_the_gyro_turns_the_heading_only_over_fresh_samples() {
    constexpr float RATE = 0.5f; // rad/s
    const BodyStateMsg body = GaitWalk(0, 0, 0).body;
    FusedOdometry odometry;
    const auto tick = [&](bool sampled) {
        odometry.update(body, {}, sampled ? std::optional<float>(RATE) : std::nullopt, DT);
    };
    tick(true); // the first sample has no time since the last
    for (int t = 1; t <= 8 * SAMPLE_TICKS; ++t) tick(t % SAMPLE_TICKS == 0);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, RATE * 8 * SAMPLE_TICKS * DT, odometry.estimate().yaw);
    for (int t = 1; t <= 4 * SAMPLE_TICKS; ++t) tick(t == 4 * SAMPLE_TICKS);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, RATE * 8 * SAMPLE_TICKS * DT, odometry.estimate().yaw);
    for (int t = 1; t <= SAMPLE_TICKS; ++t) tick(t == SAMPLE_TICKS);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, RATE * 9 * SAMPLE_TICKS * DT, odometry.estimate().yaw);
    TEST_ASSERT_EQUAL_FLOAT(0, odometry.estimate().kinematicYaw);
}

// A reset between two gyro samples: the next sample turns the new frame only by the time since the reset.
void test_a_reset_between_gyro_samples_carries_no_turn_into_the_new_frame() {
    constexpr float RATE = 0.5f; // rad/s
    const BodyStateMsg body {};
    FusedOdometry odometry;
    const auto tick = [&](bool sampled) {
        odometry.update(body, {}, sampled ? std::optional<float>(RATE) : std::nullopt, DT);
    };
    tick(true);
    for (int t = 1; t <= SAMPLE_TICKS; ++t) tick(t == SAMPLE_TICKS);
    tick(false);
    tick(false);
    odometry.reset();
    tick(false);
    tick(false);
    tick(true);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, RATE * 3 * DT, odometry.estimate().yaw);
}

// Standing still, the bias is learned; this is what the next tests deny to motions that only look still.
void test_a_still_stand_learns_the_gyro_bias() {
    GaitWalk walk(0, 0, 0);
    walk.gyro = GYRO_BIAS;
    walk.run(ticksFor(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.1f * GYRO_BIAS, GYRO_BIAS, learnedBias(walk.fused));
}

// A stance change keeps its planted feet still while it swings the others, and the gyro reads the body rocking on
// them: it is no stillness, nor is the settling for SETTLE_S after it.
void test_a_gyro_rate_during_a_stance_change_is_not_learned_as_bias() {
    GaitWalk walk(0, 0, 0);
    float wide[6][4];
    for (int leg = 0; leg < 6; ++leg) {
        for (int j = 0; j < 4; ++j) wide[leg][j] = walk.body.feet[leg][j];
        wide[leg][0] *= 1.2f;
    }
    walk.gait.setDefaultFootTarget(wide);
    walk.gyro = GYRO_BIAS;
    int ticks = 0;
    for (; walk.gait.hasPendingStanceChange() && ticks < ticksFor(3.0f); ++ticks) walk.run(1);
    TEST_ASSERT_FALSE(walk.gait.hasPendingStanceChange());
    TEST_ASSERT_GREATER_THAN(ticksFor(0.5f), ticks);
    walk.run(ticksFor(0.75f * FusedOdometry::SETTLE_S));
    TEST_ASSERT_FLOAT_WITHIN(0.1f * GYRO_BIAS, 0, learnedBias(walk.fused));
}

// The body's yaw (psi) easing back to 0 over the planted feet, as after an animation, turns the gyro and not the feet.
void test_an_easing_body_yaw_is_not_learned_as_bias() {
    GaitWalk walk(0, 0, 0);
    walk.body.psi = 0.1f;
    float rate = 0;
    for (int t = 0; t < ticksFor(0.4f); ++t) {
        const float before = walk.body.psi;
        walk.body.psi = lerpf(before, 0.0f, BODY_SMOOTHING);
        rate = (before - walk.body.psi) / DT;
        walk.gyro = rate;
        walk.run(1);
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(FusedOdometry::PSI_STILL_RATE, rate);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, learnedBias(walk.fused));
}

// Feet the gait moves on the floor without cycling, as a stopped walk eases them home, keep the robot from being still.
void test_feet_moving_on_the_floor_teach_no_bias() {
    constexpr float FEET_MM_PER_S = 20.0f;
    BodyStateMsg body = GaitWalk(0, 0, 0).body;
    FusedOdometry odometry;
    for (int t = 1; t <= ticksFor(1.0f); ++t) {
        for (auto &foot : body.feet) foot[1] += FEET_MM_PER_S * DT;
        odometry.update(body, ALL_DOWN, t % SAMPLE_TICKS == 0 ? std::optional<float>(GYRO_BIAS) : std::nullopt, DT);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f * GYRO_BIAS, 0, learnedBias(odometry));
}

// A walk too slow for the feet's motion to tell it from standing (under STILL_RATE / KIN_PER_SPEED) still swings its
// feet, so a biased gyro teaches it nothing.
void test_a_slow_walk_teaches_no_bias() {
    constexpr float SECONDS = 4.0f;
    GaitWalk walk(0, 2.5f, 0);
    walk.state.phase_rate = 0.5f;
    walk.gyro = GYRO_BIAS;
    walk.run(ticksFor(SECONDS));
    const float speed = walk.odometry.estimate().distance / SECONDS;
    TEST_ASSERT_GREATER_THAN_FLOAT(0.001f, speed);
    TEST_ASSERT_LESS_THAN_FLOAT(FusedOdometry::STILL_RATE / FusedOdometry::KIN_PER_SPEED, speed);
    TEST_ASSERT_FLOAT_WITHIN(0.1f * GYRO_BIAS, 0, learnedBias(walk.fused));
}

namespace {

const CommandMsg FORWARD {0, 1, 0, 0, 0, 0, 0, 0};
const CommandMsg CENTRED {0, 0, 0, 0, 0, 0, 0, 0};

void run(VirtualRobot &robot, const CommandMsg &command, float seconds) {
    for (int t = 0; t < ticksFor(seconds); ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(command);
        robot.tick();
    }
}

void standUp(VirtualRobot &robot) {
    robot.setMode(MOTION_STATE::STAND);
    run(robot, CENTRED, 1.0f);
}

Peripherals::Imu gyroZ(float rate) {
    Peripherals::Imu imu;
    imu.gyroRad[2] = rate;
    return imu;
}

// The standing time the bias is learned within: SETTLE_S of stillness, then some gyro samples.
constexpr float LEARN_S = 0.5f;
constexpr float WALK_S = 2.0f;

// Stands the robot up and holds it still for `seconds`, its gyro reading `rate` from power-on, as a bias does.
void standOnGyro(VirtualRobot &robot, float rate, float seconds) {
    robot.setImu(gyroZ(rate));
    robot.setMode(MOTION_STATE::STAND);
    run(robot, CENTRED, seconds);
}

// The fused heading a forward walk turns by while the gyro reads `rate`, the robot itself walking straight.
float yawWalkedOnGyro(VirtualRobot &robot, float rate) {
    robot.setImu(gyroZ(rate));
    robot.setMode(MOTION_STATE::WALK);
    const float start = robot.odometry().yaw;
    run(robot, FORWARD, WALK_S);
    return std::remainder(robot.odometry().yaw - start, TWO_PI);
}

} // namespace

// The body travel the gait's stance strokes command is integrated tick by tick from the stroke and cadence it walks.
void test_a_tripod_walk_forward_moves_the_odometry_forward_and_straight() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    float commandedMm = 0;
    for (int t = 0; t < ticksFor(WALK_S); ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(FORWARD);
        robot.tick();
        const gait_state_t &g = robot.gaitState();
        TEST_ASSERT_EQUAL(GaitType::TRI_GATE, g.gait_type);
        if (gaitIsCommanded(g)) commandedMm += g.step_y * g.phase_rate * DT / g.stand_frac;
    }
    const float commanded = commandedMm / 1000.0f;
    const OdometryEstimate e = robot.odometry();
    TEST_ASSERT_GREATER_THAN_FLOAT(0.1f, commanded);
    TEST_ASSERT_FLOAT_WITHIN(0.03f * commanded, commanded, e.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f * e.y, 0, e.x);
    // On a gyro reading zero the fused heading stays; the feet's own turns by about -0.015 rad as every walk starts,
    // and by none in the steady walk.
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0, e.yaw);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0, e.kinematicYaw);
}

// A stance change steps every foot to a new place, but each one only through the air: the robot does not travel.
void test_a_stance_change_at_rest_adds_no_travel() {
    VirtualRobot robot;
    standUp(robot);
    float before[18];
    std::copy_n(robot.angles(), 18, before);
    const CommandMsg wide {0, 0, 0, 0, 0, 0, 0, 0.6f};
    run(robot, wide, 3.0f);
    float moved = 0;
    for (int j = 0; j < 18; ++j) moved = std::max(moved, std::fabs(robot.angles()[j] - before[j]));
    TEST_ASSERT_GREATER_THAN_FLOAT(5.0f, moved);
    const OdometryEstimate e = robot.odometry();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, e.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, e.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, e.yaw);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, e.distance);
}

// The stand's sticks tilt and shift the body over the planted feet; the robot does not travel.
void test_a_stand_tilt_and_shift_are_no_travel() {
    VirtualRobot robot;
    standUp(robot);
    const CommandMsg leaning {1, 1, 1, 1, 0, 0, 0, 0};
    run(robot, leaning, 1.0f);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.2f, robot.bodyCommand().phi);
    TEST_ASSERT_GREATER_THAN_FLOAT(40.0f, robot.bodyCommand().xm);
    const OdometryEstimate e = robot.odometry();
    TEST_ASSERT_EQUAL_FLOAT(0, e.x);
    TEST_ASSERT_EQUAL_FLOAT(0, e.y);
    TEST_ASSERT_EQUAL_FLOAT(0, e.yaw);
    TEST_ASSERT_EQUAL_FLOAT(0, e.distance);
}

// Walking, the fused heading follows the gyro; a bias it learned standing still is taken out of it. One it never saw
// standing, as when the gyro read none until the walk, turns the heading by nearly the bias over the walk.
void test_a_gyro_bias_seen_standing_is_learned_and_kept_out_of_the_heading() {
    {
        VirtualRobot robot;
        standOnGyro(robot, GYRO_BIAS, LEARN_S);
        TEST_ASSERT_FLOAT_WITHIN(0.1f * GYRO_BIAS * WALK_S, 0, yawWalkedOnGyro(robot, GYRO_BIAS));
    }
    {
        VirtualRobot robot;
        standOnGyro(robot, 0, LEARN_S);
        TEST_ASSERT_GREATER_THAN_FLOAT(0.5f * GYRO_BIAS * WALK_S, yawWalkedOnGyro(robot, GYRO_BIAS));
    }
}

// The stand's tilt stick ramps the body's phi, a rotation about y that the gyro's z does not see: a robot that tilts
// while it stands still learns its bias all the same, and walks straight on it.
void test_a_stand_tilt_ramp_does_not_keep_the_bias_from_being_learned() {
    constexpr float TILT_S = 2.0f;
    VirtualRobot robot;
    robot.setImu(gyroZ(GYRO_BIAS));
    robot.setMode(MOTION_STATE::STAND);
    for (int t = 0; t < ticksFor(TILT_S); ++t) {
        const CommandMsg tilt {0, 0, static_cast<float>(t) / ticksFor(TILT_S), 0, 0, 0, 0, 0};
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(tilt);
        robot.tick();
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(0.2f, robot.bodyCommand().phi);
    TEST_ASSERT_FLOAT_WITHIN(0.1f * GYRO_BIAS * WALK_S, 0, yawWalkedOnGyro(robot, GYRO_BIAS));
}

// A mode that steps no gait leaves the pose where the last stepping tick put it.
void test_the_odometry_holds_its_pose_while_deactivated() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    run(robot, FORWARD, 1.0f);
    const OdometryEstimate walked = robot.odometry();
    TEST_ASSERT_GREATER_THAN_FLOAT(0.05f, walked.y);
    robot.setMode(MOTION_STATE::DEACTIVATED);
    run(robot, CENTRED, 0.5f);
    const OdometryEstimate held = robot.odometry();
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, robot.mode());
    TEST_ASSERT_EQUAL_FLOAT(walked.x, held.x);
    TEST_ASSERT_EQUAL_FLOAT(walked.y, held.y);
    TEST_ASSERT_EQUAL_FLOAT(walked.yaw, held.yaw);
    TEST_ASSERT_EQUAL_FLOAT(walked.distance, held.distance);
}

// A reset mid-walk, where no bias can be learned again, starts the pose from where the robot is and keeps the bias.
void test_resetting_the_odometry_zeroes_the_pose_and_keeps_the_gyro_bias() {
    VirtualRobot robot;
    standOnGyro(robot, GYRO_BIAS, LEARN_S);
    robot.setMode(MOTION_STATE::WALK);
    run(robot, FORWARD, 1.0f);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.05f, robot.odometry().y);
    robot.resetOdometry();
    run(robot, FORWARD, DT);
    // The tick after the reset counts from it: one tick's step, 1 mm at this walk's 0.2 m/s.
    const OdometryEstimate reset = robot.odometry();
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0, reset.x);
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0, reset.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, reset.yaw);
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0, reset.distance);
    TEST_ASSERT_FLOAT_WITHIN(0.1f * GYRO_BIAS * WALK_S, 0, yawWalkedOnGyro(robot, GYRO_BIAS));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_straight_walk_lands_on_the_commanded_displacement);
    RUN_TEST(test_a_sideways_walk_lands_on_the_commanded_displacement);
    RUN_TEST(test_a_turn_in_place_lands_on_the_commanded_yaw);
    RUN_TEST(test_an_arc_lands_on_the_commanded_pose);
    RUN_TEST(test_a_foot_lifted_3_mm_or_more_is_ignored);
    RUN_TEST(test_every_foot_is_in_stance_once_the_gait_stops_cycling);
    RUN_TEST(test_feet_moved_while_paused_are_no_travel);
    RUN_TEST(test_the_gyro_turns_the_heading_only_over_fresh_samples);
    RUN_TEST(test_a_reset_between_gyro_samples_carries_no_turn_into_the_new_frame);
    RUN_TEST(test_a_still_stand_learns_the_gyro_bias);
    RUN_TEST(test_a_gyro_rate_during_a_stance_change_is_not_learned_as_bias);
    RUN_TEST(test_an_easing_body_yaw_is_not_learned_as_bias);
    RUN_TEST(test_feet_moving_on_the_floor_teach_no_bias);
    RUN_TEST(test_a_slow_walk_teaches_no_bias);
    RUN_TEST(test_a_tripod_walk_forward_moves_the_odometry_forward_and_straight);
    RUN_TEST(test_a_stance_change_at_rest_adds_no_travel);
    RUN_TEST(test_a_stand_tilt_and_shift_are_no_travel);
    RUN_TEST(test_a_gyro_bias_seen_standing_is_learned_and_kept_out_of_the_heading);
    RUN_TEST(test_a_stand_tilt_ramp_does_not_keep_the_bias_from_being_learned);
    RUN_TEST(test_the_odometry_holds_its_pose_while_deactivated);
    RUN_TEST(test_resetting_the_odometry_zeroes_the_pose_and_keeps_the_gyro_bias);
    return UNITY_END();
}
