// The firmware's Hexapod run off the robot (firmware/host/virtual_robot.h): commands reach the motion
// through its own EventBus subscriptions, and the clock is the control loop's 5 ms tick.

#define USE_POLICY 1
#define USE_MPU6050 1

#include <unity.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>
#include <gait_speed.h>
#include <virtual_robot.h>

namespace {

constexpr int TICKS_PER_SECOND = 200;
constexpr int COMMAND_PERIOD_TICKS = 4;  // the app's 20 ms stick period
constexpr float STEPPING_DEG_PER_TICK = 0.05f;

int ticksFor(float seconds) { return static_cast<int>(std::lround(seconds * TICKS_PER_SECOND)); }

// The largest joint step over the ticks run, in degrees: what a moving gait produces and a resting one does not.
float largestStep(VirtualRobot &robot, int ticks, const CommandMsg *heartbeat = nullptr) {
    float previous[18];
    std::copy_n(robot.angles(), 18, previous);
    float largest = 0.0f;
    for (int t = 0; t < ticks; ++t) {
        if (heartbeat && t % COMMAND_PERIOD_TICKS == 0) robot.command(*heartbeat);
        robot.tick();
        for (int j = 0; j < 18; ++j) largest = std::max(largest, std::fabs(robot.angles()[j] - previous[j]));
        std::copy_n(robot.angles(), 18, previous);
    }
    return largest;
}

void standUp(VirtualRobot &robot) {
    robot.setMode(MOTION_STATE::STAND);
    largestStep(robot, ticksFor(1.0f));
}

const CommandMsg FORWARD{0, 1, 0, 0, 0, 0, 0, 0};

}  // namespace

void setUp() {}
void tearDown() {}

void test_a_new_robot_is_deactivated() {
    VirtualRobot robot;
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, robot.mode());
}

// What a simulated robot's actuators read (servosDriven, servoAngles): what ServoController was last told.
void test_a_new_robot_drives_no_servo() {
    VirtualRobot robot;
    robot.tick();
    TEST_ASSERT_FALSE(robot.servosDriven());
}

void test_idle_from_boot_wakes_the_boards_but_sends_no_pose() {
    VirtualRobot robot;
    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::IDLE, robot.mode());
    TEST_ASSERT_FALSE(robot.servosDriven());
    TEST_ASSERT_EQUAL_UINT(0, robot.servoWrites());
}

void test_a_standing_robot_drives_its_servos_to_the_motions_pose() {
    VirtualRobot robot;
    standUp(robot);
    TEST_ASSERT_TRUE(robot.servosDriven());
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(robot.angles(), robot.servoAngles(), 18);
    robot.setMode(MOTION_STATE::DEACTIVATED);
    robot.tick();
    TEST_ASSERT_FALSE(robot.servosDriven());
}

void test_a_mode_request_reaches_the_motion_through_the_bus() {
    VirtualRobot robot;
    robot.setMode(MOTION_STATE::STAND);
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
}

void test_a_standing_robot_holds_still() {
    VirtualRobot robot;
    standUp(robot);
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, largestStep(robot, ticksFor(0.5f)));
}

void test_a_stick_held_in_walk_steps_the_legs() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    TEST_ASSERT_GREATER_THAN_FLOAT(STEPPING_DEG_PER_TICK, largestStep(robot, ticksFor(1.0f), &FORWARD));
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, robot.mode());
}

// Joint angles per tick of a walk whose heartbeat of `command` stops at `cut` ticks, or never when `cut` is negative.
std::vector<std::array<float, 18>> walkWithHeartbeatUntil(const CommandMsg &command, int cut, int ticks) {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    std::vector<std::array<float, 18>> trace;
    for (int t = 0; t < ticks; ++t) {
        if ((cut < 0 || t < cut) && t % COMMAND_PERIOD_TICKS == 0) robot.command(command);
        robot.tick();
        std::array<float, 18> angles;
        std::copy_n(robot.angles(), 18, angles.begin());
        trace.push_back(angles);
    }
    return trace;
}

// How long after its last command a walk that stops hearing `command` still moves exactly as one that hears it, in ms.
float msObeyedAfterSilence(const CommandMsg &command, float seconds) {
    const int lastCommand = ticksFor(1.0f);
    const std::vector<std::array<float, 18>> heard = walkWithHeartbeatUntil(command, -1, ticksFor(seconds));
    const std::vector<std::array<float, 18>> cutOff =
        walkWithHeartbeatUntil(command, lastCommand + 1, ticksFor(seconds));
    size_t departed = heard.size();
    for (size_t t = 0; t < heard.size() && departed == heard.size(); ++t) {
        if (heard[t] != cutOff[t]) departed = t;
    }
    return (departed - lastCommand) * 1000.0f / TICKS_PER_SECOND;
}

// The steering dead-man (motion.h STEERING_TIMEOUT_MS): a stick driving the walk is obeyed for 600 ms after the last
// command and no longer.
void test_a_walk_obeys_its_last_command_for_600_ms_and_no_longer() {
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 600.0f, msObeyedAfterSilence(FORWARD, 2.0f));
}

// A command that drives no locomotion, such as the height alone, keeps the 2 s link timeout (COMMAND_TIMEOUT_MS).
void test_a_height_only_command_is_obeyed_for_2_s_after_silence() {
    const CommandMsg raised {0, 0, 0, 0, 0.5f, 0, 0, 0};
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 2000.0f, msObeyedAfterSilence(raised, 3.5f));
}

// The robot's ServoController holds every joint within its travel, whatever the motion asks: a stand pushed to every
// extreme at once asks some joint for more, and no servo is driven past its limit.
void test_no_servo_is_driven_past_its_joint_travel() {
    VirtualRobot robot;
    standUp(robot);
    const CommandMsg extreme {1, 1, 1, 1, 1, 0, 0, 1};
    float asked = 0.0f;  // the furthest beyond its travel the motion asks a joint to go, in degrees
    float driven = 0.0f; // and a servo is driven
    for (int t = 0; t < ticksFor(2.0f); ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(extreme);
        robot.tick();
        for (int j = 0; j < 18; ++j) {
            const float limit = JOINT_LIMIT_DEG[j % 3];
            asked = std::max(asked, std::fabs(robot.angles()[j]) - limit);
            driven = std::max(driven, std::fabs(robot.servoAngles()[j]) - limit);
        }
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(1.0f, asked);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, driven);
}

void test_a_walk_without_commands_comes_to_rest() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    largestStep(robot, ticksFor(1.0f), &FORWARD);
    largestStep(robot, ticksFor(0.6f + 1.1f));  // the timeout, then the ramp's settle and the feet's ease
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, largestStep(robot, ticksFor(0.5f)));
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, robot.mode());
}

void test_deactivating_mid_walk_freezes_the_joints() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    largestStep(robot, ticksFor(0.5f), &FORWARD);
    robot.setMode(MOTION_STATE::DEACTIVATED);
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, robot.mode());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, largestStep(robot, ticksFor(0.5f), &FORWARD));
}

void test_a_gait_selected_while_standing_is_applied() {
    VirtualRobot robot;
    standUp(robot);
    robot.setGait(GaitType::BI_GATE);
    robot.tick();
    TEST_ASSERT_EQUAL(GaitType::BI_GATE, robot.gait());
}

namespace {

constexpr GaitType CLASSIC_GAITS[4] = {GaitType::TRI_GATE, GaitType::BI_GATE, GaitType::WAVE, GaitType::RIPPLE};

struct HeldRun {
    int clamped = 0;          // joint commands beyond travel, each one ServoController's withinTravel would change
    float largestStep = 0.0f; // the largest joint step in a tick, in degrees
};

// Holds `command` at the app's stick period for `ticks`.
HeldRun hold(VirtualRobot &robot, const CommandMsg &command, int ticks) {
    HeldRun run;
    float previous[18];
    std::copy_n(robot.angles(), 18, previous);
    for (int t = 0; t < ticks; ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(command);
        robot.tick();
        for (int j = 0; j < 18; ++j) {
            const float angle = robot.angles()[j];
            if (withinTravel(angle, j % 3) != angle) run.clamped++;
            run.largestStep = std::max(run.largestStep, std::fabs(angle - previous[j]));
            previous[j] = angle;
        }
    }
    return run;
}

void walkIn(VirtualRobot &robot, GaitType gait) {
    standUp(robot);
    robot.setGait(gait);
    robot.tick();
    robot.setMode(MOTION_STATE::WALK);
}

// lx, ly, rx at s = +1, the fastest the stick can ask for.
CommandMsg fullStick(float lx, float ly, float rx) { return {lx, ly, rx, 0, 0, 1, 0, 0}; }

const CommandMsg FULL_STICKS[] = {fullStick(0, 1, 0), fullStick(0, -1, 0), fullStick(1, 0, 0), fullStick(-1, 0, 0),
                                  fullStick(0, 0, 1), fullStick(0, 0, -1), fullStick(1, 1, 1), fullStick(-1, -1, -1)};

void describe(char *msg, size_t size, GaitType gait, const CommandMsg &c, const HeldRun &run) {
    std::snprintf(msg, size, "gait %d stick (%.0f %.0f %.0f) h %.2f fd %.2f: %d clamped, largest step %.2f deg",
                  (int)gait, c.lx, c.ly, c.rx, c.h, c.fd, run.clamped, run.largestStep);
}

// Joint angles per tick of `ticks` with `command` held from a stand, in `mode`.
std::vector<std::array<float, 18>> traceHeld(MOTION_STATE mode, const CommandMsg &command, int ticks) {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(mode);
    std::vector<std::array<float, 18>> trace;
    for (int t = 0; t < ticks; ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(command);
        robot.tick();
        std::array<float, 18> angles;
        std::copy_n(robot.angles(), 18, angles.begin());
        trace.push_back(angles);
    }
    return trace;
}

} // namespace

// The stick is a share of the top speed the envelope measured, so no full deflection, alone or combined, asks a
// joint for more than its travel, in any classic gait.
void test_full_stick_walk_stays_in_travel_for_every_classic_gait() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (const CommandMsg &stick : FULL_STICKS) {
            VirtualRobot robot;
            walkIn(robot, gait);
            const HeldRun run = hold(robot, stick, ticksFor(3.0f));
            char msg[160];
            describe(msg, sizeof(msg), gait, stick, run);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(STEPPING_DEG_PER_TICK, run.largestStep, msg);
        }
    }
}

// AUTO picks the gait in the ramp's window at the start of a walk, after the command was already turned into a
// stroke and cadence for the gait it left; WAVE-like and TRI-like top speeds differ several times, so the target
// is recomputed for the new gait on the same tick, from the stick already held.
void test_an_auto_gait_switch_retargets_the_held_stick() {
    struct Case {
        float s;
        GaitType selected;
    };
    for (const Case &k : {Case {0.0f, GaitType::TRI_GATE}, Case {1.0f, GaitType::BI_GATE}}) {
        VirtualRobot robot;
        standUp(robot);
        robot.setGait(GaitType::AUTO);
        robot.tick();
        TEST_ASSERT_EQUAL(GaitType::RIPPLE, robot.gait());
        robot.setMode(MOTION_STATE::WALK);
        robot.tick();

        robot.command({0, 1, 0, 0, 0, k.s, 0, 0});
        robot.tick();
        TEST_ASSERT_EQUAL(k.selected, robot.gait());
        const WalkTarget left = classicWalkTarget(GaitType::RIPPLE, 0, 1, 0, k.s, 0, 0, 0);
        const WalkTarget expected = classicWalkTarget(k.selected, 0, 1, 0, k.s, 0, 0, 0);
        TEST_ASSERT_TRUE(std::fabs(expected.phaseRate - left.phaseRate) > 0.1f * expected.phaseRate);
        const gait_state_t &target = robot.gaitCommand();
        TEST_ASSERT_EQUAL_FLOAT(expected.stepX, target.step_x);
        TEST_ASSERT_EQUAL_FLOAT(expected.stepY, target.step_y);
        TEST_ASSERT_EQUAL_FLOAT(expected.stepAngle, target.step_angle);
        TEST_ASSERT_EQUAL_FLOAT(expected.phaseRate, target.phase_rate);
    }
}

// Narrowing the stance at full stick retargets the stroke to the narrow stance at once, while the feet only reach it
// swing by swing; the walk through that transition stays within travel.
void test_feet_distance_change_while_walking_stays_in_travel() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (const CommandMsg &stick : {fullStick(0, 1, 0), fullStick(1, 1, 1)}) {
            VirtualRobot robot;
            walkIn(robot, gait);
            CommandMsg c = stick;
            HeldRun run = hold(robot, c, ticksFor(1.0f));
            c.fd = -1.0f;
            const HeldRun narrowed = hold(robot, c, ticksFor(2.0f));
            run.clamped += narrowed.clamped;
            run.largestStep = narrowed.largestStep;
            char msg[160];
            describe(msg, sizeof(msg), gait, c, run);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(STEPPING_DEG_PER_TICK, run.largestStep, msg);
        }
    }
}

// A posture outside the walkable region drives the robot exactly as the walkable posture it clamps to, standing and
// walking: the body height, the stance and the walk's envelope all come from the clamped one.
void test_unwalkable_posture_is_clamped() {
    float s1 = 0.0f, h = 1.0f, fd = -1.0f;
    TEST_ASSERT_FALSE(postureWalkable(s1, h, fd));
    clampPosture(s1, h, fd);
    TEST_ASSERT_TRUE(postureWalkable(s1, h, fd));
    TEST_ASSERT_TRUE(h < 1.0f && fd > -1.0f);

    for (const MOTION_STATE mode : {MOTION_STATE::STAND, MOTION_STATE::WALK}) {
        const float ly = mode == MOTION_STATE::WALK ? 1.0f : 0.0f;
        const auto asked = traceHeld(mode, {0, ly, 0, 0, 1.0f, 0, 0, -1.0f}, ticksFor(3.0f));
        const auto clamped = traceHeld(mode, {0, ly, 0, 0, h, 0, s1, fd}, ticksFor(3.0f));
        TEST_ASSERT_TRUE_MESSAGE(asked == clamped, mode == MOTION_STATE::WALK ? "walking" : "standing");
    }
}

// The dead-man zeroes the stick it last heard, not just the stroke: an AUTO walk that sped up while stepping, and so
// still holds its slow gait, must not have the faster gait chosen and walked once the timeout has stopped it.
void test_a_timed_out_auto_walk_stays_at_rest() {
    VirtualRobot robot;
    standUp(robot);
    robot.setGait(GaitType::AUTO);
    robot.tick();
    robot.setMode(MOTION_STATE::WALK);
    hold(robot, {0, 0.3f, 0, 0, 0, 0, 0, 0}, ticksFor(1.0f));
    hold(robot, fullStick(0, 1, 0), ticksFor(1.0f));
    TEST_ASSERT_EQUAL(GaitType::RIPPLE, robot.gait());

    largestStep(robot, ticksFor(0.6f + 2.0f)); // the timeout, then the ramp's settle and the feet's ease
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, largestStep(robot, ticksFor(1.0f)));
}

// A classic gait loaded after the searched one walks the stick held now, not one a classic command left behind: the
// switch to TUNED here lands in the ramp window right after a slow stick, whose first ramp tick stays inside the
// stride deadband, and the stick is released before TRI.
void test_a_classic_gait_loaded_after_tuned_walks_no_stale_stick() {
    VirtualRobot robot;
    walkIn(robot, GaitType::TRI_GATE);
    robot.tick();
    robot.command({0, 0.3f, 0, 0, 0, 0, 0, 0});
    robot.tick();
    robot.setGait(GaitType::TUNED);
    robot.tick();
    TEST_ASSERT_EQUAL(GaitType::TUNED, robot.gait());
    hold(robot, FORWARD, ticksFor(1.0f));
    hold(robot, CommandMsg {}, ticksFor(1.5f));

    robot.setGait(GaitType::TRI_GATE);
    robot.tick();
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, robot.gait());
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, largestStep(robot, ticksFor(1.0f)));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_new_robot_is_deactivated);
    RUN_TEST(test_a_new_robot_drives_no_servo);
    RUN_TEST(test_idle_from_boot_wakes_the_boards_but_sends_no_pose);
    RUN_TEST(test_a_standing_robot_drives_its_servos_to_the_motions_pose);
    RUN_TEST(test_a_mode_request_reaches_the_motion_through_the_bus);
    RUN_TEST(test_a_standing_robot_holds_still);
    RUN_TEST(test_a_stick_held_in_walk_steps_the_legs);
    RUN_TEST(test_a_walk_obeys_its_last_command_for_600_ms_and_no_longer);
    RUN_TEST(test_a_height_only_command_is_obeyed_for_2_s_after_silence);
    RUN_TEST(test_no_servo_is_driven_past_its_joint_travel);
    RUN_TEST(test_a_walk_without_commands_comes_to_rest);
    RUN_TEST(test_deactivating_mid_walk_freezes_the_joints);
    RUN_TEST(test_a_gait_selected_while_standing_is_applied);
    RUN_TEST(test_full_stick_walk_stays_in_travel_for_every_classic_gait);
    RUN_TEST(test_an_auto_gait_switch_retargets_the_held_stick);
    RUN_TEST(test_feet_distance_change_while_walking_stays_in_travel);
    RUN_TEST(test_unwalkable_posture_is_clamped);
    RUN_TEST(test_a_timed_out_auto_walk_stays_at_rest);
    RUN_TEST(test_a_classic_gait_loaded_after_tuned_walks_no_stale_stick);
    return UNITY_END();
}
