// Host-side tests for the gait engine. GaitController is header-only float math with no ESP
// dependency, so the coordination invariants the robot depends on are checkable off-device.
//
// The previous test in this directory timed 1000 steps and asserted a wall-clock budget. Host
// timing says nothing about the 5 ms control loop on the ESP32-S3, so it is replaced by the
// invariants that actually break when the gait tables or stroke signs are edited.

#include <unity.h>

#include <cmath>
#include <cstdio>
#include <gait.h>

namespace {

constexpr float STAND[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                               {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

constexpr float DT = 0.005f; // the control-task period

gait_state_t makeGait(GaitType type, float stepX, float stepZ, float stepAngle) {
    gait_state_t gait{};
    gait.gait_type = type;
    gait.step_x = stepX;
    gait.step_z = stepZ;
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
    // step() treats |step_x| < 2 and |step_z| < 2 with no yaw as standing still; a joystick at
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
    target.step_z = 100.0f;

    approachGaitCommand(cur, target, 0.005f, 0.15f);
    TEST_ASSERT_TRUE_MESSAGE(cur.step_z > 0.0f, "one tick should make progress");
    TEST_ASSERT_TRUE_MESSAGE(cur.step_z < 10.0f, "one 5 ms tick must not jump most of the way");
    TEST_ASSERT_TRUE_MESSAGE(cur.step_z <= target.step_z, "ramp must not overshoot the command");
}

void test_command_ramp_converges_on_the_command() {
    gait_state_t cur = rampState();
    gait_state_t target = rampState();
    target.step_x = -70.0f;
    target.step_z = 100.0f;
    target.step_angle = 0.4f;

    for (int i = 0; i < 200; i++) approachGaitCommand(cur, target, 0.005f, 0.15f);  // 1 s

    TEST_ASSERT_FLOAT_WITHIN(0.5f, target.step_x, cur.step_x);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, target.step_z, cur.step_z);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, target.step_angle, cur.step_angle);
}

void test_command_ramp_is_independent_of_loop_rate() {
    // Same elapsed time in different sized steps must land in the same place, or control-loop
    // jitter would silently change how the robot accelerates.
    gait_state_t fine = rampState(), coarse = rampState();
    gait_state_t target = rampState();
    target.step_z = 100.0f;

    for (int i = 0; i < 20; i++) approachGaitCommand(fine, target, 0.005f, 0.15f);  // 20 x 5 ms
    approachGaitCommand(coarse, target, 0.100f, 0.15f);                             // 1 x 100 ms

    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, fine.step_z, coarse.step_z, "ramp depends on tick size");
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
    target.step_z = 100.0f;

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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_tripod_keeps_three_feet_loaded);
    RUN_TEST(test_idle_command_settles_on_the_default_stance);
    RUN_TEST(test_command_below_deadband_does_not_start_the_cycle);
    RUN_TEST(test_phase_stays_normalized_over_a_long_walk);
    RUN_TEST(test_swing_lifts_the_foot_and_stance_keeps_it_down);
    RUN_TEST(test_stance_sweeps_the_foot_against_the_commanded_direction);
    RUN_TEST(test_yaw_command_sweeps_feet_tangentially);
    RUN_TEST(test_gait_tables_cover_every_leg_once_per_cycle);
    RUN_TEST(test_inverse_kinematics_holds_the_nominal_stance);
    RUN_TEST(test_command_ramp_moves_toward_target_without_jumping);
    RUN_TEST(test_command_ramp_converges_on_the_command);
    RUN_TEST(test_command_ramp_is_independent_of_loop_rate);
    RUN_TEST(test_command_ramp_switches_cadence_mode_outright);
    RUN_TEST(test_command_ramp_leaves_the_gait_schedule_alone);
    RUN_TEST(test_stance_change_walks_the_feet_to_the_new_target);
    RUN_TEST(test_settled_legs_hold_station_while_others_reposition);
    RUN_TEST(test_walking_still_swings_every_foot);
    return UNITY_END();
}
