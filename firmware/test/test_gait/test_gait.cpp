// Host-side tests for the gait engine. GaitController is header-only float math with no ESP
// dependency, so the coordination invariants the robot depends on are checkable off-device.
//
// The previous test in this directory timed 1000 steps and asserted a wall-clock budget. Host
// timing says nothing about the 5 ms control loop on the ESP32-S3, so it is replaced by the
// invariants that actually break when the gait tables or stroke signs are edited.

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <gait.h>

namespace {

constexpr float STAND[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                               {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

constexpr float DT = 0.005f; // the control-task period

gait_state_t makeGait(GaitType type, float stepX, float stepY, float stepAngle) {
    gait_state_t gait{};
    gait.gait_type = type;
    gait.step_x = stepX;
    gait.step_y = stepY;
    gait.step_angle = stepAngle;
    gait.step_speed = 1.0f;
    gait.step_height = 40.0f;
    gait.step_depth = 0.002f;
    return gait;
}

BodyStateMsg makeBody() {
    BodyStateMsg body{};
    body.updateFeet(STAND);
    return body;
}


// Mirrors the GaitType::TUNED path in MotionService::handleCommand. Kept here rather than shared
// because MotionService itself drags in esp_log.h and the peripheral drivers; the constants it uses
// live in gait.h precisely so this mirror cannot drift on the numbers that matter.
struct TunedCommand {
    gait_state_t gait;
    float zm;
};

TunedCommand makeTuned(float ly, float lx, float rx, float s1) {
    const float cmd[3] = {ly * TUNED_CMD_VX_MAX, -lx * TUNED_CMD_VY_MAX, rx * TUNED_CMD_YAW_MAX};
    float a[6];
    tuned_gait::velocity_to_gait(cmd, a);

    TunedCommand out{};
    out.gait.gait_type = GaitType::TUNED;
    out.gait.step_x = a[0] * tuned_gait::STEP_XY_MM;
    out.gait.step_y = a[1] * tuned_gait::STEP_XY_MM;
    out.gait.step_angle = a[2] * tuned_gait::STEP_ANGLE_RAD;
    out.gait.step_speed = 1.0f;
    out.gait.step_height = CLIP(tuned_gait::STEP_HEIGHT_MM + s1 * TUNED_LIFT_TRIM_MM,
                                tuned_gait::STEP_HEIGHT_MIN_MM, tuned_gait::STEP_HEIGHT_MAX_MM);
    out.gait.step_depth = tuned_gait::STEP_DEPTH_MM;
    out.gait.phase_rate = tuned_gait::PHASE_RATE_MIN +
                          (a[5] + 1.f) * 0.5f * (tuned_gait::PHASE_RATE_MAX - tuned_gait::PHASE_RATE_MIN);
    // Ride height is what makes the searched foot lift reachable at all, so a sweep that leaves it
    // at zero measures a robot this gait never runs as.
    out.zm = -tuned_gait::RIDE_MM;
    return out;
}

// Worst overshoot past JOINT_LIMIT_DEG over a full gait cycle, per joint type (coxa, femur, tibia).
// Also reports the peak absolute command, so a clean sweep still says how much travel was left.
void sweepTunedCycle(float ly, float lx, float rx, float s1, float worst[3], float peak[3]) {
    TunedCommand cmd = makeTuned(ly, lx, rx, s1);
    GaitController controller;
    controller.snapDefaultFootTarget(STAND);
    controller.setGait(cmd.gait);
    Kinematics kin;
    BodyStateMsg body = makeBody();
    body.zm = cmd.zm;

    constexpr int STEPS = 360;
    for (int k = 0; k < STEPS; ++k) {
        controller.setPhase((float)k / STEPS);
        controller.generateFeet(cmd.gait, body);
        float angles[18];
        kin.inverseKinematics(body, angles);
        for (int i = 0; i < 18; ++i) {
            const int joint = i % 3;
            const float mag = std::fabs(angles[i]);
            if (mag > peak[joint]) peak[joint] = mag;
            const float over = mag - JOINT_LIMIT_DEG[joint];
            if (over > worst[joint]) worst[joint] = over;
        }
    }
}
} // namespace

void setUp() {}
void tearDown() {}

void test_tripod_keeps_three_feet_loaded() {
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 0, 0);
    GaitController controller;
    controller.setGait(gait);

    // The tripod table is hand-staggered (offsets 0, .52, .08, .58, .16, .66 against a .5167 stand
    // fraction) rather than two exact groups of three, so it is not a perfect tripod: measured over
    // the cycle it holds 3 feet down 89.3% of the time, 4 feet down 10.3%, and drops to 2 for a
    // 0.33% sliver where four legs overlap in swing. Bound that sliver instead of pretending it is
    // zero -- a table or stand_frac edit that opens a real 2-foot window fails here.
    constexpr int SAMPLES = 100000;
    int underSupported = 0;
    int swingSamples[6] = {0, 0, 0, 0, 0, 0};

    for (int s = 0; s < SAMPLES; ++s) {
        const float phase = static_cast<float>(s) / SAMPLES;
        int swinging = 0;
        for (int i = 0; i < 6; ++i) {
            if (std::fmod(phase + gait.offset[i], 1.0f) >= gait.stand_frac) {
                ++swinging;
                ++swingSamples[i];
            }
        }
        TEST_ASSERT_LESS_OR_EQUAL_INT(4, swinging);
        if (swinging > 3) ++underSupported;
    }

    TEST_ASSERT_LESS_THAN_FLOAT(0.01f, static_cast<float>(underSupported) / SAMPLES);

    for (int i = 0; i < 6; ++i) {
        // Duty: every leg spends the same fraction of the cycle in swing, 1 - stand_frac.
        TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f - gait.stand_frac,
                                 static_cast<float>(swingSamples[i]) / SAMPLES);
    }
}

void test_idle_command_settles_on_the_default_stance() {
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 0, 0);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();
    for (int i = 0; i < 6; ++i) body.feet[i][2] += 30.0f; // feet lifted off the stance

    for (int t = 0; t < 400; ++t) controller.step(gait, body, DT);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.getPhase());
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            TEST_ASSERT_FLOAT_WITHIN(0.5f, STAND[i][j], body.feet[i][j]);
        }
    }
}

void test_command_below_deadband_does_not_start_the_cycle() {
    // step() treats |step_x| < 2 and |step_y| < 2 with no yaw as standing still; a joystick at
    // rest must not creep the phase forward.
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 1.9f, -1.9f, 0.0f);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();
    for (int t = 0; t < 200; ++t) controller.step(gait, body, DT);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.getPhase());
}

void test_phase_stays_normalized_over_a_long_walk() {
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 60.0f, 0.0f, 0.0f);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();
    for (int t = 0; t < 20000; ++t) {
        controller.step(gait, body, DT);
        const float phase = controller.getPhase();
        TEST_ASSERT_TRUE(phase >= 0.0f && phase < 1.0f);
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 3; ++j) TEST_ASSERT_TRUE(std::isfinite(body.feet[i][j]));
        }
    }
}

void test_swing_lifts_the_foot_and_stance_keeps_it_down() {
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 60.0f, 0.0f, 0.0f);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();
    float apex[6] = {0, 0, 0, 0, 0, 0};
    float lowest[6] = {0, 0, 0, 0, 0, 0};

    for (int s = 0; s < 1000; ++s) {
        controller.setPhase(static_cast<float>(s) / 1000.0f);
        controller.generateFeet(gait, body);
        for (int i = 0; i < 6; ++i) {
            const float lift = body.feet[i][2] - STAND[i][2];
            if (lift > apex[i]) apex[i] = lift;
            if (lift < lowest[i]) lowest[i] = lift;
        }
    }

    for (int i = 0; i < 6; ++i) {
        // The Bezier swing peaks near step_height; it must clear the ground and must not overshoot
        // the commanded lift, which is what the servo range and the terrain search assume.
        TEST_ASSERT_GREATER_THAN_FLOAT(0.5f * gait.step_height, apex[i]);
        TEST_ASSERT_LESS_OR_EQUAL_FLOAT(1.15f * gait.step_height, apex[i]);
        // Stance presses down by step_depth at most, never a hole in the floor.
        TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(-1.1f * gait.step_depth, lowest[i]);
    }
}

void test_swing_reaches_its_end_without_a_jump() {
    // (1 - t)^11 underflows float32 once 1 - t drops below ~8e-5. Built by dividing back up from that
    // power, every Bernstein weight was zero for the last ticks of a swing and the foot was commanded
    // to the curve's origin, a jump of half the stride, about once a swing in a hundred at 5 ms ticks.
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 60.0f, 0.0f, 0.0f);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);
    BodyStateMsg body = makeBody();

    constexpr int SAMPLES = 400000;
    float previous[6][2];
    float worst = 0.0f;
    for (int s = 0; s <= SAMPLES; ++s) {
        controller.setPhase(static_cast<float>(s) / SAMPLES * 0.999999f);
        controller.generateFeet(gait, body);
        for (int i = 0; i < 6; ++i) {
            if (s > 0) {
                const float jump = std::hypot(body.feet[i][0] - previous[i][0], body.feet[i][1] - previous[i][1]);
                if (jump > worst) worst = jump;
            }
            previous[i][0] = body.feet[i][0];
            previous[i][1] = body.feet[i][1];
        }
    }
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(1.0f, worst, "a foot jumped between neighbouring phases");
}

void test_stance_sweeps_the_foot_against_the_commanded_direction() {
    // Body moves +x because the loaded feet travel -x. A sign flip here walks the robot backwards
    // while every other assertion above still passes.
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 60.0f, 0.0f, 0.0f);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();

    controller.setPhase(0.0f);
    controller.generateFeet(gait, body);
    float startX[6];
    for (int i = 0; i < 6; ++i) startX[i] = body.feet[i][0];

    controller.setPhase(gait.stand_frac * 0.98f);
    controller.generateFeet(gait, body);

    for (int i = 0; i < 6; ++i) {
        // Only legs that are in stance across both samples are comparable.
        const bool inStanceAtStart = std::fmod(0.0f + gait.offset[i], 1.0f) < gait.stand_frac;
        const bool inStanceAtEnd = std::fmod(gait.stand_frac * 0.98f + gait.offset[i], 1.0f) < gait.stand_frac;
        if (!inStanceAtStart || !inStanceAtEnd) continue;
        TEST_ASSERT_LESS_THAN_FLOAT(startX[i], body.feet[i][0]);
    }
}

// The stance presses the foot by step_depth mid-stance and levels it at both ends, along the stroke whatever its
// direction. Measured along x + y instead, one diagonal lifted the foot off level at the end of the stance and the
// other kept it pressed to lift-off, so it jumped there by the whole depth.
void test_stance_levels_the_foot_at_lift_off_in_every_direction() {
    const float d = 42.4264f;
    const float strokes[][2] = {{60, 0}, {0, 60}, {d, d}, {d, -d}, {-d, d}, {-d, -d}};
    for (const auto &stroke : strokes) {
        gait_state_t gait = makeGait(GaitType::TRI_GATE, stroke[0], stroke[1], 0.0f);
        gait.step_depth = 5.0f;
        GaitController controller;
        controller.setGait(gait);
        controller.snapDefaultFootTarget(STAND);
        BodyStateMsg body = makeBody();
        for (int i = 0; i < 6; ++i) {
            for (const float through : {0.5f, 0.995f}) {
                controller.setPhase(std::fmod(gait.stand_frac * through - gait.offset[i] + 1.0f, 1.0f));
                controller.generateFeet(gait, body);
                const float pressed = std::fabs(body.feet[i][2] - STAND[i][2]);
                if (through == 0.5f)
                    TEST_ASSERT_FLOAT_WITHIN(0.05f * gait.step_depth, gait.step_depth, pressed);
                else
                    TEST_ASSERT_LESS_THAN_FLOAT(0.1f * gait.step_depth, pressed);
            }
        }
    }
}

void test_yaw_command_sweeps_feet_tangentially() {
    // A pure in-place turn must move each foot perpendicular to its own radius, in a consistent
    // rotational sense: the cross product radius x displacement has the same sign for all six.
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0.0f, 0.0f, 0.35f);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();
    controller.setPhase(0.0f);
    controller.generateFeet(gait, body);

    float sign = 0.0f;
    for (int i = 0; i < 6; ++i) {
        const bool inStance = std::fmod(gait.offset[i], 1.0f) < gait.stand_frac;
        if (!inStance) continue;
        const float dx = body.feet[i][0] - STAND[i][0];
        const float dy = body.feet[i][1] - STAND[i][1];
        const float cross = STAND[i][0] * dy - STAND[i][1] * dx;
        TEST_ASSERT_TRUE(std::fabs(cross) > 1.0f);
        if (sign == 0.0f) sign = cross > 0 ? 1.0f : -1.0f;
        TEST_ASSERT_TRUE(cross * sign > 0.0f);
    }
    TEST_ASSERT_TRUE(sign != 0.0f);
}

void test_gait_tables_cover_every_leg_once_per_cycle() {
    // Same tripod reasoning applied to the remaining coordination patterns: offsets must be a
    // permutation over the cycle, so no leg is left permanently loaded or permanently swinging.
    const GaitType types[] = {GaitType::TRI_GATE, GaitType::BI_GATE, GaitType::WAVE, GaitType::RIPPLE};
    for (GaitType type : types) {
        gait_state_t gait = makeGait(type, 0, 0, 0);
        GaitController controller;
        controller.setGait(gait);

        TEST_ASSERT_TRUE(gait.stand_frac > 0.0f && gait.stand_frac < 1.0f);
        for (int i = 0; i < 6; ++i) {
            TEST_ASSERT_TRUE(gait.offset[i] >= 0.0f && gait.offset[i] < 1.0f);
            bool swingSeen = false;
            bool stanceSeen = false;
            for (int s = 0; s < 1000; ++s) {
                const float ph = std::fmod(static_cast<float>(s) / 1000.0f + gait.offset[i], 1.0f);
                if (ph >= gait.stand_frac)
                    swingSeen = true;
                else
                    stanceSeen = true;
            }
            TEST_ASSERT_TRUE(swingSeen);
            TEST_ASSERT_TRUE(stanceSeen);
        }
    }
}

void test_inverse_kinematics_holds_the_nominal_stance() {
    // The stance the gait engine hands to IK must resolve to angles a servo can reach; a mount
    // angle or link length typo shows up here as a clamped acos or an out-of-range joint.
    Kinematics kinematics;
    BodyStateMsg body = makeBody();
    body.zm = 0;

    float angles[18];
    kinematics.inverseKinematics(body, angles);

    for (int i = 0; i < 18; ++i) {
        TEST_ASSERT_TRUE(std::isfinite(angles[i]));
        // ServoController::setAngles adds 90 deg to every tibia before the PWM conversion, so the
        // servo-frame angle is what has to stay inside the +/-90 deg the calibration maps.
        const float servoAngle = angles[i] + (i % 3 == 2 ? 90.0f : 0.0f);
        TEST_ASSERT_TRUE(std::fabs(servoAngle) <= 90.0f);
    }

    // The stance is mirror-symmetric about the body x axis, so the left and right legs of each
    // pair must carry the same femur/tibia angles.
    const int mirrored[3][2] = {{0, 2}, {3, 5}};
    for (const auto &pair : mirrored) {
        TEST_ASSERT_FLOAT_WITHIN(0.01f, angles[pair[0] * 3 + 1], angles[pair[1] * 3 + 1]);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, angles[pair[0] * 3 + 2], angles[pair[1] * 3 + 2]);
    }
}


// --- Locomotion command ramp (issue #7) -------------------------------------------------------
// The command used to be written straight into gait_state, so a stick slammed forward changed the
// stride between one 5 ms tick and the next. These pin the ramp's shape rather than just its
// existence: a ramp that snapped, overshot, or depended on loop rate would still "smooth" but would
// not be what the robot needs.

namespace {
gait_state_t rampState() {
    return {15, 0, 0, 0, 1, 0.002f, default_stand_frac, GaitType::TRI_GATE, {0, 0.52f, 0.08f, 0.58f, 0.16f, 0.66f}, 0};
}
}  // namespace

void test_command_ramp_moves_toward_target_without_jumping() {
    gait_state_t cur = rampState();
    gait_state_t target = rampState();
    target.step_y = 100.0f;

    approachGaitCommand(cur, target, 0.005f, 0.15f);
    TEST_ASSERT_TRUE_MESSAGE(cur.step_y > 0.0f, "one tick should make progress");
    TEST_ASSERT_TRUE_MESSAGE(cur.step_y < 10.0f, "one 5 ms tick must not jump most of the way");
    TEST_ASSERT_TRUE_MESSAGE(cur.step_y <= target.step_y, "ramp must not overshoot the command");
}

void test_command_ramp_converges_on_the_command() {
    gait_state_t cur = rampState();
    gait_state_t target = rampState();
    target.step_x = -70.0f;
    target.step_y = 100.0f;
    target.step_angle = 0.4f;

    for (int i = 0; i < 200; i++) approachGaitCommand(cur, target, 0.005f, 0.15f);  // 1 s

    TEST_ASSERT_FLOAT_WITHIN(0.5f, target.step_x, cur.step_x);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, target.step_y, cur.step_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, target.step_angle, cur.step_angle);
}

void test_command_ramp_is_independent_of_loop_rate() {
    // Same elapsed time in different sized steps must land in the same place, or control-loop
    // jitter would silently change how the robot accelerates.
    gait_state_t fine = rampState(), coarse = rampState();
    gait_state_t target = rampState();
    target.step_y = 100.0f;

    for (int i = 0; i < 20; i++) approachGaitCommand(fine, target, 0.005f, 0.15f);  // 20 x 5 ms
    approachGaitCommand(coarse, target, 0.100f, 0.15f);                             // 1 x 100 ms

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, fine.step_y, coarse.step_y, "ramp depends on tick size");
}

void test_command_ramp_switches_cadence_mode_outright() {
    // phase_rate 0 selects the stride-derived law; it is not a slow cadence. Easing across that
    // boundary would run the robot at cadences neither law asks for.
    gait_state_t cur = rampState();
    gait_state_t target = rampState();

    cur.phase_rate = 1.2f;
    target.phase_rate = 0.0f;
    approachGaitCommand(cur, target, 0.005f, 0.15f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, cur.phase_rate, "leaving explicit cadence must be instant");

    target.phase_rate = 1.4f;
    approachGaitCommand(cur, target, 0.005f, 0.15f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(1.4f, cur.phase_rate, "entering explicit cadence must be instant");

    gait_state_t between = rampState();
    between.phase_rate = 1.0f;
    target.phase_rate = 2.0f;
    approachGaitCommand(between, target, 0.005f, 0.15f);
    TEST_ASSERT_TRUE_MESSAGE(between.phase_rate > 1.0f && between.phase_rate < 1.2f,
                             "between two explicit rates the cadence should ease");
}

void test_command_ramp_leaves_the_gait_schedule_alone() {
    // Blending two leg schedules would produce a third that coordinates nothing.
    gait_state_t cur = rampState();
    gait_state_t target = rampState();
    target.gait_type = GaitType::BI_GATE;
    target.stand_frac = 0.75f;
    for (int i = 0; i < 6; i++) target.offset[i] = 0.9f;
    target.step_y = 100.0f;

    for (int i = 0; i < 50; i++) approachGaitCommand(cur, target, 0.005f, 0.15f);

    TEST_ASSERT_EQUAL_INT((int)GaitType::TRI_GATE, (int)cur.gait_type);
    TEST_ASSERT_EQUAL_FLOAT(default_stand_frac, cur.stand_frac);
    TEST_ASSERT_EQUAL_FLOAT(0.52f, cur.offset[1]);
}


// --- Stance repositioning (issue #9) ----------------------------------------------------------
// With no locomotion command but a changed stance target, step() runs the gait anyway so each foot
// is lifted and placed on its own swing instead of dragged. Nothing covered this path before: every
// other test uses snapDefaultFootTarget(), which teleports the stance and skips it entirely.

void test_stance_change_walks_the_feet_to_the_new_target() {
    GaitController controller;
    controller.snapDefaultFootTarget(STAND);
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 0, 0);
    controller.setGait(gait);
    BodyStateMsg body = makeBody();

    float wider[6][4];
    for (int i = 0; i < 6; i++) {
        wider[i][0] = STAND[i][0] * 1.25f;
        wider[i][1] = STAND[i][1] * 1.25f;
        wider[i][2] = STAND[i][2];
        wider[i][3] = 1;
    }
    controller.setDefaultFootTarget(wider);
    TEST_ASSERT_TRUE(controller.hasPendingStanceChange());

    float maxLift[6] = {0, 0, 0, 0, 0, 0};
    for (int t = 0; t < 2000 && controller.hasPendingStanceChange(); t++) {
        controller.step(gait, body, DT);
        for (int i = 0; i < 6; i++) {
            const float lift = body.feet[i][2] - STAND[i][2];
            if (lift > maxLift[i]) maxLift[i] = lift;
        }
    }

    TEST_ASSERT_FALSE_MESSAGE(controller.hasPendingStanceChange(), "stance change never converged");
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_TRUE_MESSAGE(maxLift[i] > 1.0f, "a foot slid to the new stance instead of stepping");
    }
}


void test_settled_legs_hold_station_while_others_reposition() {
    // Only one foot is off target. The other five are already standing where they belong, so they
    // should stay planted rather than cycling through empty lifts until the odd one out catches up.
    GaitController controller;
    controller.snapDefaultFootTarget(STAND);
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 0, 0);
    controller.setGait(gait);
    BodyStateMsg body = makeBody();

    float tgt[6][4];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 4; j++) tgt[i][j] = STAND[i][j];
    tgt[0][0] += 30.0f;
    controller.setDefaultFootTarget(tgt);

    float maxLift[6] = {0, 0, 0, 0, 0, 0};
    int ticks = 0;
    while (controller.hasPendingStanceChange() && ticks < 4000) {
        controller.step(gait, body, DT);
        ticks++;
        for (int i = 0; i < 6; i++) {
            const float lift = body.feet[i][2] - STAND[i][2];
            if (lift > maxLift[i]) maxLift[i] = lift;
        }
    }

    TEST_ASSERT_FALSE_MESSAGE(controller.hasPendingStanceChange(), "stance change never converged");
    TEST_ASSERT_TRUE_MESSAGE(maxLift[0] > 1.0f, "the foot that had to move never stepped");
    for (int i = 1; i < 6; i++) {
        TEST_ASSERT_TRUE_MESSAGE(maxLift[i] < 1.0f, "a leg already on target took an empty step");
    }
}

// A foot that begins a repositioning swing finishes it. Holding station used to be decided every tick, so a foot left
// a little off its target swung for it and, once its anchor came within 0.5 mm mid-swing while another foot was still
// stepping (foot 2 here, whose swing overlaps foot 0's), dropped to the floor in one tick. A swing descends about
// 0.5 mm a tick at this cadence; 2 mm tells that from a drop.
void test_a_repositioning_swing_once_begun_is_finished() {
    GaitController controller;
    controller.snapDefaultFootTarget(STAND);
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 0, 0);
    gait.step_height = 15.0f;
    controller.setGait(gait);
    BodyStateMsg body = makeBody();

    float tgt[6][4];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 4; j++) tgt[i][j] = STAND[i][j];
    tgt[0][0] += 0.8f;
    tgt[2][0] += 30.0f;
    controller.setDefaultFootTarget(tgt);

    float maxLift = 0.0f, maxDrop = 0.0f, previous = body.feet[0][2];
    for (int t = 0; t < 4000; t++) {
        controller.step(gait, body, DT);
        maxLift = std::fmax(maxLift, body.feet[0][2] - STAND[0][2]);
        maxDrop = std::fmax(maxDrop, previous - body.feet[0][2]);
        previous = body.feet[0][2];
    }
    TEST_ASSERT_FALSE_MESSAGE(controller.hasPendingStanceChange(), "stance change never converged");
    TEST_ASSERT_TRUE_MESSAGE(maxLift > 1.0f, "the foot that had to move never stepped");
    TEST_ASSERT_LESS_THAN_FLOAT(2.0f, maxDrop);
}

// A cycle starts every swing from where the feet stand. A stance change ends as the last foot's anchor comes within
// 0.5 mm, mid-swing, and that foot was left flagged as swinging; a walk started next with it in swing did not take a
// new swing start and pulled its anchor back toward the old stance, 30 mm here, in one tick. A walking foot moves
// 1-2 mm a tick; 5 mm tells that from a jump.
void test_a_walk_after_a_stance_change_starts_its_swings_from_the_new_stance() {
    GaitController controller;
    controller.snapDefaultFootTarget(STAND);
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 0, 0);
    gait.step_height = 15.0f;
    controller.setGait(gait);
    BodyStateMsg body = makeBody();

    float tgt[6][4];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 4; j++) tgt[i][j] = STAND[i][j];
    tgt[1][0] += 30.0f; // foot 1 swings at phase 0, where the walk below starts
    controller.setDefaultFootTarget(tgt);
    for (int t = 0; t < 4000 && controller.hasPendingStanceChange(); t++) controller.step(gait, body, DT);
    TEST_ASSERT_FALSE(controller.hasPendingStanceChange());
    for (int t = 0; t < 200; t++) controller.step(gait, body, DT);

    gait_state_t target = gait;
    target.step_y = 60.0f; // eased in as MotionService does, so a swing starting at rest starts at no stroke
    float largest = 0.0f, previous[3] = {body.feet[1][0], body.feet[1][1], body.feet[1][2]};
    for (int t = 0; t < 400; t++) {
        approachGaitCommand(gait, target, DT, 0.15f);
        controller.step(gait, body, DT);
        largest = std::fmax(largest, std::hypot(body.feet[1][0] - previous[0], body.feet[1][1] - previous[1],
                                                body.feet[1][2] - previous[2]));
        for (int j = 0; j < 3; j++) previous[j] = body.feet[1][j];
    }
    TEST_ASSERT_LESS_THAN_FLOAT(5.0f, largest);
}

void test_walking_still_swings_every_foot() {
    // The hold-station rule must not leak into normal walking, where every foot sits at its default
    // position and would otherwise qualify as "settled" and stop lifting entirely.
    GaitController controller;
    controller.snapDefaultFootTarget(STAND);
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 60, 0);
    controller.setGait(gait);
    BodyStateMsg body = makeBody();

    float maxLift[6] = {0, 0, 0, 0, 0, 0};
    for (int t = 0; t < 1200; t++) {
        controller.step(gait, body, DT);
        for (int i = 0; i < 6; i++) {
            const float lift = body.feet[i][2] - STAND[i][2];
            if (lift > maxLift[i]) maxLift[i] = lift;
        }
    }
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_TRUE_MESSAGE(maxLift[i] > 1.0f, "a foot stopped swinging during normal walking");
    }
}

void test_nominal_stance_is_inside_joint_travel() {
    // The servo path now clamps to JOINT_LIMIT_DEG. If the resting stance itself needed a clamp the
    // robot would fight its own end stops while standing still, so pin that it does not.
    Kinematics kin;
    BodyStateMsg body = makeBody();
    float angles[18];
    kin.inverseKinematics(body, angles);
    for (int i = 0; i < 18; ++i) {
        const float limit = JOINT_LIMIT_DEG[i % 3];
        TEST_ASSERT_TRUE_MESSAGE(std::fabs(angles[i]) <= limit, "nominal stance needs a clamped servo");
    }
}

void test_walking_stays_inside_joint_travel() {
    // A moderate walk must be executable without clamping. The tuned gait's own envelope is pinned
    // separately by test_tuned_gait_is_feasible_at_the_bench_envelope.
    GaitController controller;
    controller.snapDefaultFootTarget(STAND);
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 40, 0);
    controller.setGait(gait);
    Kinematics kin;
    BodyStateMsg body = makeBody();

    float worst[3] = {0, 0, 0};
    for (int t = 0; t < 600; ++t) {
        controller.step(gait, body, DT);
        float angles[18];
        kin.inverseKinematics(body, angles);
        for (int i = 0; i < 18; ++i) {
            const float over = std::fabs(angles[i]) - JOINT_LIMIT_DEG[i % 3];
            if (over > worst[i % 3]) worst[i % 3] = over;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(worst[0] <= 0.0f, "coxa exceeds travel while walking");
    TEST_ASSERT_TRUE_MESSAGE(worst[1] <= 0.0f, "femur exceeds travel while walking");
    TEST_ASSERT_TRUE_MESSAGE(worst[2] <= 0.0f, "tibia exceeds travel while walking");
}

// The searched gait's foot lift is only reachable at the bottom of the s1 trim. At the default trim
// the femur is commanded past travel from stride ~40 mm upward; at the top of the trim it is out of
// range at every stride. At the bottom the femur is clean across the entire forward x turn plane --
// the plane the bench ladder is driven over -- which is the claim the hardware test relies on.
// Lateral is the exception and is characterized separately below.
void test_tuned_lift_trim_keeps_the_femur_inside_travel() {
    float worst[3] = {0, 0, 0};
    float peak[3] = {0, 0, 0};
    for (float ly = -1.0f; ly <= 1.0f; ly += 0.125f)
        for (float rx = -1.0f; rx <= 1.0f; rx += 0.125f) sweepTunedCycle(ly, 0.f, rx, -1.0f, worst, peak);

    char msg[176];
    std::snprintf(msg, sizeof(msg),
                  "fwd x turn plane at s1=-1: femur peak %.1f of %.1f deg; coxa over %.1f, tibia over %.1f",
                  peak[1], JOINT_LIMIT_DEG[1], worst[0], worst[2]);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(worst[1] <= 0.0f,
                             "femur exceeds travel on the forward x turn plane at the bottom of the lift trim");
}

// Lateral is the axis that breaks the femur once combined with forward and turn, and there is no
// safe margin: with adversarial signs even a tenth of lateral stick costs ~0.5 deg of femur travel
// on a full forward+turn command, growing roughly linearly to ~15 deg at full lateral. Recorded
// rather than gated, because the useful conclusion is operational -- do not push all three sticks at
// once -- and because pure lateral on its own is entirely feasible (it saturates at ~28 mm stride).
void test_tuned_gait_records_lateral_femur_cost() {
    char msg[192];
    int n = std::snprintf(msg, sizeof(msg), "femur over vs |lx| on full fwd+turn:");
    for (float lx : {0.1f, 0.25f, 0.5f, 1.0f}) {
        float worst[3] = {0, 0, 0};
        float peak[3] = {0, 0, 0};
        // Both sign pairings; lateral opposing the turn is the worse of the two.
        sweepTunedCycle(1.0f, lx, -1.0f, -1.0f, worst, peak);
        sweepTunedCycle(-1.0f, lx, 1.0f, -1.0f, worst, peak);
        n += std::snprintf(msg + n, sizeof(msg) - n, " %.2f=%.1f", lx, worst[1]);
    }
    TEST_MESSAGE(msg);

    float full[3] = {0, 0, 0};
    float fpeak[3] = {0, 0, 0};
    sweepTunedCycle(-1.0f, -1.0f, -0.5f, -1.0f, full, fpeak);
    TEST_ASSERT_TRUE_MESSAGE(full[1] > 0.0f,
                             "full three-axis command now fits -- the lateral caveat can be dropped");
}

// On a single axis at a time the whole gait is executable, coxa included. Combined forward+turn is
// what pushes the coxa past its 31.5 deg travel, and that is pre-existing rather than a tuned-gait
// defect: the shipped TRI_GATE reaches 33.3 deg of coxa overshoot at full stick with turn, against
// this gait's 19.6 deg. So pin the single-axis envelope and characterize the combined one below.
void test_tuned_gait_is_feasible_on_single_axis_commands() {
    const float sticks[] = {0.11f, 0.22f, 0.32f, 0.41f};
    float worst[3] = {0, 0, 0};
    float peak[3] = {0, 0, 0};
    for (float v : sticks) {
        sweepTunedCycle(v, 0.f, 0.f, -1.0f, worst, peak);   // forward only
        sweepTunedCycle(0.f, v, 0.f, -1.0f, worst, peak);   // lateral only
        sweepTunedCycle(0.f, 0.f, v, -1.0f, worst, peak);   // turn only
    }
    sweepTunedCycle(0.f, 1.0f, 0.f, -1.0f, worst, peak);    // lateral saturates at only ~28 mm stride
    TEST_ASSERT_TRUE_MESSAGE(worst[0] <= 0.0f, "coxa exceeds travel on a single-axis command");
    TEST_ASSERT_TRUE_MESSAGE(worst[1] <= 0.0f, "femur exceeds travel on a single-axis command");
    TEST_ASSERT_TRUE_MESSAGE(worst[2] <= 0.0f, "tibia exceeds travel on a single-axis command");
}

// Characterization, not a gate: records where combined forward+turn starts costing coxa travel, so
// the bench operator knows which clamps to expect and a regression stays visible. Onset at the
// bottom of the lift trim is around forward 0.32 with half turn, or forward 0.41 with quarter turn.
void test_tuned_gait_records_combined_command_overshoot() {
    char msg[192];
    int n = 0;
    n += std::snprintf(msg + n, sizeof(msg) - n, "coxa overshoot (s1=-1) fwd/turn:");
    const float pairs[][2] = {{0.22f, 1.0f}, {0.32f, 0.5f}, {0.41f, 0.25f}, {0.41f, 1.0f}};
    for (auto &pr : pairs) {
        float worst[3] = {0, 0, 0};
        float peak[3] = {0, 0, 0};
        sweepTunedCycle(pr[0], 0.f, pr[1], -1.0f, worst, peak);
        n += std::snprintf(msg + n, sizeof(msg) - n, " %.2f/%.2f=%.1f", pr[0], pr[1], worst[0]);
    }
    TEST_MESSAGE(msg);
}

// The femur overshoot at the DEFAULT lift trim is the reason the robot must not ship at s1 = 0.
// If this ever reaches zero the gait has become fully feasible and the trim default can be revisited.
void test_tuned_default_lift_trim_still_exceeds_femur_travel() {
    float worst[3] = {0, 0, 0};
    float peak[3] = {0, 0, 0};
    for (float s1 : {0.0f, 1.0f})
        for (float ly : {0.5f, 1.0f}) sweepTunedCycle(ly, 0.f, 0.f, s1, worst, peak);
    char msg[160];
    std::snprintf(msg, sizeof(msg), "femur overshoot at default/top trim: %.1f deg", worst[1]);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(worst[1] > 0.0f, "femur now fits at the default trim -- revisit the trim default");
}

void test_auto_gait_picks_by_speed_and_resists_flapping() {
    TEST_ASSERT_EQUAL_INT((int)GaitType::RIPPLE, (int)selectAutoGait(0.10f, GaitType::RIPPLE));
    TEST_ASSERT_EQUAL_INT((int)GaitType::TRI_GATE, (int)selectAutoGait(0.50f, GaitType::RIPPLE));
    TEST_ASSERT_EQUAL_INT((int)GaitType::BI_GATE, (int)selectAutoGait(0.90f, GaitType::TRI_GATE));

    // Sitting just past a boundary must not flip back and forth: the band a gait already holds is
    // wider than the one it would move to.
    const float edge = 0.32f;
    TEST_ASSERT_EQUAL_INT((int)GaitType::RIPPLE, (int)selectAutoGait(edge, GaitType::RIPPLE));
    TEST_ASSERT_EQUAL_INT((int)GaitType::TRI_GATE, (int)selectAutoGait(edge, GaitType::TRI_GATE));
}

namespace {
// Walks a stop-slow-stop-fast-stop profile and reports the worst per-tick foot movement anywhere in
// it. With autoSwitch the schedule may change; where it may change is the thing under test.
float runGaitProfile(bool autoSwitch, GaitType fixedGait, int &switches) {
    GaitController controller;
    controller.snapDefaultFootTarget(STAND);
    gait_state_t gait = makeGait(autoSwitch ? GaitType::RIPPLE : fixedGait, 0, 0, 0);
    controller.setGait(gait);
    BodyStateMsg body = makeBody();
    gait_state_t target = gait;

    float prev[6][3];
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 3; ++j) prev[i][j] = body.feet[i][j];

    const float targets[5] = {0.0f, 0.15f, 0.0f, 0.95f, 0.0f};
    float worst = 0.0f;
    switches = 0;

    for (int leg = 0; leg < 5; ++leg) {
        target.step_y = targets[leg] * 100.0f;
        for (int t = 0; t < 600; ++t) {
            const bool stepping = std::fabs(gait.step_x) >= 2.0f || std::fabs(gait.step_y) >= 2.0f ||
                                  gait.step_angle != 0.0f;
            if (autoSwitch && !stepping) {
                const GaitType next = selectAutoGait(std::fabs(target.step_y) / 100.0f, gait.gait_type);
                if (next != gait.gait_type) {
                    gait.gait_type = next;
                    controller.setGait(gait);
                    target.gait_type = next;
                    target.stand_frac = gait.stand_frac;
                    for (int i = 0; i < 6; ++i) target.offset[i] = gait.offset[i];
                    switches++;
                }
            }
            approachGaitCommand(gait, target, DT, 0.15f);

            controller.step(gait, body, DT);
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 3; ++j) {
                    const float d = std::fabs(body.feet[i][j] - prev[i][j]);
                    if (d > worst) worst = d;
                    prev[i][j] = body.feet[i][j];
                }
            }
        }
    }
    return worst;
}
}  // namespace

void test_auto_gait_switch_adds_no_discontinuity() {
    // Starting to walk always snaps some legs into mid-swing, because at phase 0 any leg whose
    // offset exceeds stand_frac is already swinging. That transient is pre-existing. What must be
    // true is that switching schedules during the standstill does not make it worse -- switching
    // mid-stride did, by ~78 mm.
    int switches = 0, unused = 0;
    const float withSwitching = runGaitProfile(true, GaitType::TRI_GATE, switches);
    const float bipodOnly = runGaitProfile(false, GaitType::BI_GATE, unused);
    const float rippleOnly = runGaitProfile(false, GaitType::RIPPLE, unused);
    const float baseline = bipodOnly > rippleOnly ? bipodOnly : rippleOnly;

    char dbg[128];
    snprintf(dbg, sizeof(dbg), "switches=%d auto=%.2f bipod=%.2f ripple=%.2f mm", switches, withSwitching,
             bipodOnly, rippleOnly);
    TEST_ASSERT_TRUE_MESSAGE(switches >= 2, dbg);
    TEST_ASSERT_TRUE_MESSAGE(withSwitching <= baseline + 1.0f, dbg);
}

// --- Stopping ---------------------------------------------------------------------------------
// A mode change out of WALK zeroes the locomotion command and lets the ramp bring the legs down.
// Nothing downstream may keep the cycle alive once the operator has stopped asking for motion.

void test_released_command_stops_the_cycle() {
    // The regression this pins: step_angle was tested against exact zero, which the exponential ramp
    // never reaches -- it decayed to a subnormal (measured 2.1e-44) and stalled there, so the gait
    // kept stepping in place with full foot lift for as long as the robot was powered. STAND looked
    // like it could not stop a walk that had ever been given a turn command.
    gait_state_t cur = makeGait(GaitType::TRI_GATE, -30.0f, 80.0f, 0.4f);
    gait_state_t target = cur;
    GaitController controller;
    controller.setGait(cur);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();
    for (int t = 0; t < 400; ++t) {  // 2 s of walking with yaw
        approachGaitCommand(cur, target, DT, 0.15f);
        controller.step(cur, body, DT);
    }

    target.step_x = target.step_y = target.step_angle = 0.0f;  // MotionService's stop
    for (int t = 0; t < 600; ++t) {                            // 3 s to settle
        approachGaitCommand(cur, target, DT, 0.15f);
        controller.step(cur, body, DT);
    }

    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, cur.step_angle, "yaw command must reach zero, not a subnormal");
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, controller.getPhase(), "a released command must not keep the cycle running");
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) TEST_ASSERT_FLOAT_WITHIN(0.5f, STAND[i][j], body.feet[i][j]);
    }
}

// A stop starts the walk's stroke from rest, where a first-order ramp moved it at its fastest on the first tick, and
// shrinks the cadence with the stroke, as a slower command would. Any other change, a walk starting, eases first order.
void test_a_stop_eases_the_stroke_out_on_the_speed_law() {
    const float a = 1.0f - std::exp(-DT / 0.15f);
    gait_state_t walking = makeGait(GaitType::TRI_GATE, 0, 60.0f, 0);
    walking.phase_rate = 0.8f;
    gait_state_t live = makeGait(GaitType::TRI_GATE, 0, 0, 0);
    GaitCommandRamp ramp;
    ramp.step(live, walking, DT, 0.15f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 60.0f * a, live.step_y);
    for (int t = 0; t < 600; ++t) ramp.step(live, walking, DT, 0.15f);
    TEST_ASSERT_EQUAL_FLOAT(60.0f, live.step_y);

    gait_state_t stop = walking;
    stop.step_y = 0.0f;
    stop.phase_rate = 0.0f;
    ramp.step(live, stop, DT, 0.15f);
    TEST_ASSERT_TRUE(60.0f - live.step_y < 0.1f * 60.0f * a);
    int ticks = 1;
    float fastest = 0.0f, previous = live.step_y;
    while (gaitIsCommanded(live)) {
        ramp.step(live, stop, DT, 0.15f);
        ++ticks;
        fastest = std::max(fastest, previous - live.step_y);
        previous = live.step_y;
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, gaitIsCommanded(live) ? 0.8f * live.step_y / 60.0f : 0.0f, live.phase_rate);
    }
    // Two stages in a row peak at 1/e of a single stage's first step.
    TEST_ASSERT_FLOAT_WITHIN(0.02f * 60.0f * a, 60.0f * a / std::exp(1.0f), fastest);
    TEST_ASSERT_TRUE(ticks < 200);
}

void test_yaw_below_deadband_does_not_start_the_cycle() {
    // The yaw counterpart of test_command_below_deadband_does_not_start_the_cycle: 0.004 rad over the
    // 171 mm leg radius is 0.7 mm of foot travel, which is not a step.
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0.0f, 0.0f, 0.004f);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();
    for (int t = 0; t < 200; ++t) controller.step(gait, body, DT);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.getPhase());
}

void test_yaw_above_deadband_still_steps() {
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0.0f, 0.0f, 0.05f);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);

    BodyStateMsg body = makeBody();
    for (int t = 0; t < 200; ++t) controller.step(gait, body, DT);

    TEST_ASSERT_TRUE_MESSAGE(controller.getPhase() > 0.0f, "a real turn command must still walk");
}

void test_command_ramp_lands_exactly_on_a_released_command() {
    gait_state_t cur = rampState();
    cur.step_x = -70.0f;
    cur.step_y = 100.0f;
    cur.step_angle = 0.4f;
    cur.step_height = 68.0f;
    cur.step_speed = 2.0f;
    cur.phase_rate = 1.35f;
    gait_state_t target = rampState();

    for (int i = 0; i < 600; i++) approachGaitCommand(cur, target, DT, 0.15f);  // 3 s

    TEST_ASSERT_EQUAL_FLOAT(target.step_x, cur.step_x);
    TEST_ASSERT_EQUAL_FLOAT(target.step_y, cur.step_y);
    TEST_ASSERT_EQUAL_FLOAT(target.step_angle, cur.step_angle);
    TEST_ASSERT_EQUAL_FLOAT(target.step_speed, cur.step_speed);
    TEST_ASSERT_EQUAL_FLOAT(target.step_height, cur.step_height);
    TEST_ASSERT_EQUAL_FLOAT(target.step_depth, cur.step_depth);
    TEST_ASSERT_EQUAL_FLOAT(target.phase_rate, cur.phase_rate);
}

void test_collapsed_ramp_takes_the_command_outright() {
    // How MotionService stops a mode that drives nothing: dt/tau of zero means "adopt the command
    // now", with no residue left to resume from.
    gait_state_t cur = rampState();
    cur.step_x = 60.0f;
    cur.step_angle = 0.3f;
    gait_state_t target = rampState();

    approachGaitCommand(cur, target, 0.0f, 0.0f);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, cur.step_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, cur.step_angle);
}

// --- Starting a walk ----------------------------------------------------------------------------
// Entering WALK from a settled STAND, or swapping the gait while standing, must not move any joint
// faster than the walk itself does. Leika eae7c8fe found the same engine lifting a foot in one tick
// at up to 3,700 deg/s: the cycle restarted with feet whose offset puts them mid-swing.

namespace {
constexpr GaitType CLASSICAL_GAITS[4] = {GaitType::TRI_GATE, GaitType::BI_GATE, GaitType::WAVE, GaitType::RIPPLE};
constexpr int ENTRY_TICKS = 200;  // 1 s: the ramp's rise and the first cycle at least
constexpr int SETTLE_TICKS = 600; // 3 s: the ramp is within 0.1 mm of the command
constexpr int STEADY_TICKS = 400; // 2 s: at least one full cycle at every classical cadence
// The entry may move a joint somewhat faster than steady walking does, not by an order of magnitude.
constexpr float ENTRY_STEP_BOUND = 1.5f;

// MotionService's STAND command, and a full 100 mm forward stride on the legacy stride-derived cadence (phase_rate 0),
// the law WALK_NN's residual modes and repositioning still run on.
gait_state_t standCommand(GaitType type) {
    gait_state_t gait {15.0f, 0, 0, 0, 1.0f, 0.002f, default_stand_frac, type, {}, 0};
    GaitController {}.setGait(gait);
    return gait;
}

gait_state_t fullForwardCommand(const gait_state_t& stand) {
    gait_state_t walk = stand;
    walk.step_y = 100.0f;
    walk.step_height = 20.0f;
    return walk;
}

// MotionService::loadGait's schedule swap: only while not stepping, onto both the live and the target state.
void swapGait(GaitController& controller, gait_state_t& live, gait_state_t& target, GaitType next) {
    live.gait_type = next;
    controller.setGait(live);
    target.gait_type = next;
    target.stand_frac = live.stand_frac;
    for (int i = 0; i < 6; ++i) target.offset[i] = live.offset[i];
}

struct JointTracker {
    Kinematics kin;
    float previous[18];

    explicit JointTracker(BodyStateMsg& body) { kin.inverseKinematics(body, previous); }

    // Largest per-tick joint change in degrees over `ticks` control ticks.
    float run(GaitController& controller, gait_state_t& live, const gait_state_t& target, BodyStateMsg& body,
              int ticks) {
        float worst = 0.0f;
        for (int t = 0; t < ticks; ++t) {
            approachGaitCommand(live, target, DT, 0.15f);
            controller.step(live, body, DT);
            float angles[18];
            kin.inverseKinematics(body, angles);
            for (int i = 0; i < 18; ++i) {
                worst = std::fmax(worst, std::fabs(angles[i] - previous[i]));
                previous[i] = angles[i];
            }
        }
        return worst;
    }
};

void assertEntryWithinSteadyBound(const char* how, GaitType type, float entry, float steady) {
    char msg[160];
    std::snprintf(msg, sizeof(msg), "%s gait %d: entry %.2f deg/tick (%.0f deg/s), steady %.2f deg/tick", how,
                  (int)type, entry, entry / DT, steady);
    TEST_MESSAGE(msg);
    TEST_ASSERT_TRUE_MESSAGE(entry <= ENTRY_STEP_BOUND * steady, msg);
}
} // namespace

void test_walk_entered_from_stand_moves_no_joint_faster_than_walking() {
    for (const GaitType type : CLASSICAL_GAITS) {
        GaitController controller;
        controller.snapDefaultFootTarget(STAND);
        BodyStateMsg body = makeBody();
        gait_state_t live = standCommand(type);
        gait_state_t target = live;
        JointTracker joints(body);

        joints.run(controller, live, target, body, SETTLE_TICKS);
        target = fullForwardCommand(target);
        const float entry = joints.run(controller, live, target, body, ENTRY_TICKS);
        joints.run(controller, live, target, body, SETTLE_TICKS);
        const float steady = joints.run(controller, live, target, body, STEADY_TICKS);

        assertEntryWithinSteadyBound("stand -> walk", type, entry, steady);
    }
}

void test_gait_swapped_at_a_standstill_moves_no_joint_faster_than_walking() {
    for (const GaitType from : CLASSICAL_GAITS) {
        for (const GaitType to : CLASSICAL_GAITS) {
            if (from == to) continue;
            GaitController controller;
            controller.snapDefaultFootTarget(STAND);
            BodyStateMsg body = makeBody();
            gait_state_t live = standCommand(from);
            const gait_state_t stand = live;
            gait_state_t target = fullForwardCommand(stand);
            JointTracker joints(body);

            joints.run(controller, live, target, body, SETTLE_TICKS);
            target = stand;
            joints.run(controller, live, target, body, SETTLE_TICKS);
            swapGait(controller, live, target, to);
            target = fullForwardCommand(target);
            const float entry = joints.run(controller, live, target, body, ENTRY_TICKS);
            joints.run(controller, live, target, body, SETTLE_TICKS);
            const float steady = joints.run(controller, live, target, body, STEADY_TICKS);

            assertEntryWithinSteadyBound("walk -> stop -> swap -> walk", to, entry, steady);
        }
    }
}

// --- Leaving a walk -----------------------------------------------------------------------------
// Stopping ramps the command into the deadband, after which step() brings the feet home from
// wherever the cycle left them, lifted feet included. That move used to be an exponential pull,
// dt * 10 of the remaining distance per tick, which went from rest to its top speed in one tick.

namespace {
// Walks on until the ramp is one tick from the deadband, so the next tick is the first one at rest.
void walkUntilTheCommandIsReleased(JointTracker& joints, GaitController& controller, gait_state_t& live,
                                   const gait_state_t& target, BodyStateMsg& body) {
    for (;;) {
        gait_state_t next = live;
        approachGaitCommand(next, target, DT, 0.15f);
        if (!gaitIsCommanded(next)) return;
        joints.run(controller, live, target, body, 1);
    }
}
} // namespace

void test_feet_come_home_from_a_stopped_walk_without_a_jolt() {
    // Stopped at eight points of the cycle, so feet are caught at every height of their swing. The
    // exponential pull moved a joint up to 0.82 deg in a tick (tripod 0.816, bipod 0.806), all of it
    // on the first tick at rest; the ease peaks mid-move, at 1.875x its mean speed, and measures 0.30.
    // The feet are blended home together rather than stepped there in tripod order: by the time the
    // command is inside the deadband the stride is under 2 mm, and a planted foot slides at most 1.03 mm.
    constexpr float FLOOR_Z = -66.0f;
    for (const GaitType type : {GaitType::TRI_GATE, GaitType::BI_GATE}) {
        float worst = 0.0f, lowest = 0.0f, homeError = 0.0f;
        for (int stopAt = 0; stopAt < 8; ++stopAt) {
            GaitController controller;
            controller.snapDefaultFootTarget(STAND);
            BodyStateMsg body = makeBody();
            gait_state_t live = standCommand(type);
            const gait_state_t stand = live;
            gait_state_t target = fullForwardCommand(stand);
            JointTracker joints(body);

            joints.run(controller, live, target, body, SETTLE_TICKS + 17 * stopAt);
            target = stand;
            walkUntilTheCommandIsReleased(joints, controller, live, target, body);
            for (int t = 0; t < ENTRY_TICKS; ++t) {
                worst = std::fmax(worst, joints.run(controller, live, target, body, 1));
                for (int i = 0; i < 6; ++i) lowest = std::fmin(lowest, body.feet[i][2] - FLOOR_Z);
            }
            for (int i = 0; i < 6; ++i)
                for (int j = 0; j < 3; ++j) homeError = std::fmax(homeError, std::fabs(body.feet[i][j] - STAND[i][j]));
        }
        char msg[160];
        std::snprintf(msg, sizeof(msg),
                      "gait %d at rest: largest joint step %.3f deg/tick, lowest foot %.4f mm, home error %.5f mm",
                      (int)type, worst, lowest, homeError);
        TEST_MESSAGE(msg);
        TEST_ASSERT_TRUE_MESSAGE(worst <= 0.4f, msg);
        TEST_ASSERT_TRUE_MESSAGE(lowest >= -0.01f, msg);
        TEST_ASSERT_TRUE_MESSAGE(homeError <= 1e-3f, msg);
    }
}

void test_walk_resumed_while_the_feet_come_home_moves_no_joint_faster_than_walking() {
    // Resumed on the first tick at rest, a cycle restarted at phase 0 put the feet the stopped walk had
    // left up to 14.7 mm in the air straight onto its own targets: 14.8 deg in one tick on the tripod,
    // 16.4 on the bipod. The ease made it worse for longer, holding the feet away from home through
    // the first 0.3 s at rest. The new cycle now carries that offset and lets it go along the ease.
    constexpr int REST_TICKS[] = {1, 40}; // the first tick at rest, and 0.2 s into the ease
    for (const GaitType type : {GaitType::TRI_GATE, GaitType::BI_GATE}) {
        for (const int restTicks : REST_TICKS) {
            GaitController controller;
            controller.snapDefaultFootTarget(STAND);
            BodyStateMsg body = makeBody();
            gait_state_t live = standCommand(type);
            const gait_state_t stand = live;
            gait_state_t target = fullForwardCommand(stand);
            JointTracker joints(body);

            joints.run(controller, live, target, body, SETTLE_TICKS);
            target = stand;
            walkUntilTheCommandIsReleased(joints, controller, live, target, body);
            joints.run(controller, live, target, body, restTicks);
            target = fullForwardCommand(stand);
            const float entry = joints.run(controller, live, target, body, ENTRY_TICKS);
            joints.run(controller, live, target, body, SETTLE_TICKS);
            const float steady = joints.run(controller, live, target, body, STEADY_TICKS);

            char how[64];
            std::snprintf(how, sizeof(how), "walk -> stop -> resume after %d ticks at rest", restTicks);
            assertEntryWithinSteadyBound(how, type, entry, steady);
        }
    }
}

void test_feet_moved_while_standing_are_eased_back() {
    // The policy and the animations write the feet without the gait. Back at rest, the feet must be
    // eased home from where those left them, not snapped there because an earlier ease had finished.
    gait_state_t gait = makeGait(GaitType::TRI_GATE, 0, 0, 0);
    GaitController controller;
    controller.setGait(gait);
    controller.snapDefaultFootTarget(STAND);
    BodyStateMsg body = makeBody();
    for (int t = 0; t < 200; ++t) controller.step(gait, body, DT);

    for (int i = 0; i < 6; ++i) body.feet[i][2] += 30.0f;
    controller.step(gait, body, DT);
    for (int i = 0; i < 6; ++i) TEST_ASSERT_GREATER_THAN_FLOAT(STAND[i][2] + 29.0f, body.feet[i][2]);

    for (int t = 0; t < 99; ++t) controller.step(gait, body, DT);
    for (int i = 0; i < 6; ++i) TEST_ASSERT_EQUAL_FLOAT(STAND[i][2], body.feet[i][2]);
}

// --- Gait trace parity --------------------------------------------------------------------------
// The foot targets of a scripted session, which simulation/test_gait_trace_parity.py replays
// through firmware_gait.py coordinate by coordinate. The first stop is cut short, so the lateral
// walk starts while the feet are still easing home. From the repository root, regenerate
// simulation/fixtures/gait_trace.json with
//   GAIT_TRACE_OUT=simulation/fixtures/gait_trace.json pio test -e native -f test_gait
// Without GAIT_TRACE_OUT the test writes nothing and only checks that the session ends at rest.

namespace {
struct TraceSegment {
    const char* name;
    float stepX, stepY, stepHeight;
    int ticks;
};

// MotionService's STAND command, and full 100 mm forward and left strides on the legacy stride-derived cadence.
constexpr TraceSegment TRACE_SEGMENTS[] = {{"stand", 0.0f, 0.0f, 15.0f, 5},
                                           {"forward", 0.0f, 100.0f, 20.0f, 150},
                                           {"stop", 0.0f, 0.0f, 15.0f, 160},
                                           {"lateral", -100.0f, 0.0f, 20.0f, 150},
                                           {"stop", 0.0f, 0.0f, 15.0f, 230}};
constexpr int TRACE_SEGMENT_COUNT = sizeof(TRACE_SEGMENTS) / sizeof(TRACE_SEGMENTS[0]);
constexpr float TRACE_TAU_S = 0.15f;
// Every tick is run, every third one recorded: a law the port gets wrong leaves its mark for many
// ticks, and the fixture stays under 100 KB.
constexpr int TRACE_RECORD_EVERY = 3;

void writeTraceHeader(FILE* out) {
    std::fprintf(out, "{\n  \"source\": \"firmware/test/test_gait/test_gait.cpp\",\n");
    std::fprintf(out, "  \"dt\": %.9g,\n  \"command_tau_s\": %.9g,\n  \"record_every\": %d,\n  \"segments\": [\n", DT,
                 TRACE_TAU_S, TRACE_RECORD_EVERY);
    for (int k = 0; k < TRACE_SEGMENT_COUNT; ++k) {
        const TraceSegment& seg = TRACE_SEGMENTS[k];
        std::fprintf(
            out, "    {\"name\": \"%s\", \"step_x\": %.9g, \"step_y\": %.9g, \"step_height\": %.9g, \"ticks\": %d}%s\n",
            seg.name, seg.stepX, seg.stepY, seg.stepHeight, seg.ticks, k + 1 < TRACE_SEGMENT_COUNT ? "," : "");
    }
    std::fprintf(out, "  ],\n  \"feet_from_stance_mm\": {\n");
}

// To 1e-4 mm, without trailing zeros.
void writeCoordinate(FILE* out, float mm) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.4f", mm);
    char* end = text + std::strlen(text);
    while (end[-1] == '0') *--end = '\0';
    if (end[-1] == '.') *--end = '\0';
    std::fputs(std::strcmp(text, "-0") == 0 ? "0" : text, out);
}
} // namespace

void test_gait_trace_ends_at_rest_on_the_stance() {
    const char* outPath = std::getenv("GAIT_TRACE_OUT");
    FILE* out = outPath ? std::fopen(outPath, "w") : nullptr;
    if (outPath) TEST_ASSERT_NOT_NULL_MESSAGE(out, outPath);
    if (out) writeTraceHeader(out);

    const struct {
        const char* name;
        GaitType type;
    } gaits[] = {{"TRI_GATE", GaitType::TRI_GATE}, {"BI_GATE", GaitType::BI_GATE}};
    for (const auto& gaitCase : gaits) {
        GaitController controller;
        controller.snapDefaultFootTarget(STAND);
        BodyStateMsg body = makeBody();
        gait_state_t live = standCommand(gaitCase.type);
        gait_state_t target = live;
        if (out) std::fprintf(out, "    \"%s\": [\n", gaitCase.name);

        int tick = 0;
        for (const TraceSegment& seg : TRACE_SEGMENTS) {
            target.step_x = seg.stepX;
            target.step_y = seg.stepY;
            target.step_height = seg.stepHeight;
            for (int t = 0; t < seg.ticks; ++t) {
                approachGaitCommand(live, target, DT, TRACE_TAU_S);
                controller.step(live, body, DT);
                if (!out || ++tick % TRACE_RECORD_EVERY != 0) continue;
                std::fprintf(out, "%s      [", tick == TRACE_RECORD_EVERY ? "" : ",\n");
                for (int i = 0; i < 6; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        if (i + j > 0) std::fputc(',', out);
                        writeCoordinate(out, body.feet[i][j] - STAND[i][j]);
                    }
                }
                std::fprintf(out, "]");
            }
        }
        TEST_ASSERT_EQUAL_FLOAT(0.0f, controller.getPhase());
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 3; ++j) TEST_ASSERT_FLOAT_WITHIN(1e-3f, STAND[i][j], body.feet[i][j]);
        if (out) std::fprintf(out, "\n    ]%s\n", gaitCase.type == GaitType::BI_GATE ? "" : ",");
    }
    if (out) {
        std::fprintf(out, "  }\n}\n");
        std::fclose(out);
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_tripod_keeps_three_feet_loaded);
    RUN_TEST(test_idle_command_settles_on_the_default_stance);
    RUN_TEST(test_command_below_deadband_does_not_start_the_cycle);
    RUN_TEST(test_phase_stays_normalized_over_a_long_walk);
    RUN_TEST(test_swing_lifts_the_foot_and_stance_keeps_it_down);
    RUN_TEST(test_swing_reaches_its_end_without_a_jump);
    RUN_TEST(test_stance_sweeps_the_foot_against_the_commanded_direction);
    RUN_TEST(test_stance_levels_the_foot_at_lift_off_in_every_direction);
    RUN_TEST(test_yaw_command_sweeps_feet_tangentially);
    RUN_TEST(test_gait_tables_cover_every_leg_once_per_cycle);
    RUN_TEST(test_inverse_kinematics_holds_the_nominal_stance);
    RUN_TEST(test_command_ramp_moves_toward_target_without_jumping);
    RUN_TEST(test_command_ramp_converges_on_the_command);
    RUN_TEST(test_command_ramp_is_independent_of_loop_rate);
    RUN_TEST(test_command_ramp_switches_cadence_mode_outright);
    RUN_TEST(test_command_ramp_leaves_the_gait_schedule_alone);
    RUN_TEST(test_command_ramp_lands_exactly_on_a_released_command);
    RUN_TEST(test_collapsed_ramp_takes_the_command_outright);
    RUN_TEST(test_released_command_stops_the_cycle);
    RUN_TEST(test_a_stop_eases_the_stroke_out_on_the_speed_law);
    RUN_TEST(test_yaw_below_deadband_does_not_start_the_cycle);
    RUN_TEST(test_yaw_above_deadband_still_steps);
    RUN_TEST(test_stance_change_walks_the_feet_to_the_new_target);
    RUN_TEST(test_settled_legs_hold_station_while_others_reposition);
    RUN_TEST(test_a_repositioning_swing_once_begun_is_finished);
    RUN_TEST(test_a_walk_after_a_stance_change_starts_its_swings_from_the_new_stance);
    RUN_TEST(test_walking_still_swings_every_foot);
    RUN_TEST(test_nominal_stance_is_inside_joint_travel);
    RUN_TEST(test_walking_stays_inside_joint_travel);
    RUN_TEST(test_tuned_lift_trim_keeps_the_femur_inside_travel);
    RUN_TEST(test_tuned_gait_records_lateral_femur_cost);
    RUN_TEST(test_tuned_gait_is_feasible_on_single_axis_commands);
    RUN_TEST(test_tuned_gait_records_combined_command_overshoot);
    RUN_TEST(test_tuned_default_lift_trim_still_exceeds_femur_travel);
    RUN_TEST(test_auto_gait_picks_by_speed_and_resists_flapping);
    RUN_TEST(test_auto_gait_switch_adds_no_discontinuity);
    RUN_TEST(test_walk_entered_from_stand_moves_no_joint_faster_than_walking);
    RUN_TEST(test_gait_swapped_at_a_standstill_moves_no_joint_faster_than_walking);
    RUN_TEST(test_feet_come_home_from_a_stopped_walk_without_a_jolt);
    RUN_TEST(test_walk_resumed_while_the_feet_come_home_moves_no_joint_faster_than_walking);
    RUN_TEST(test_feet_moved_while_standing_are_eased_back);
    RUN_TEST(test_gait_trace_ends_at_rest_on_the_stance);
    return UNITY_END();
}
