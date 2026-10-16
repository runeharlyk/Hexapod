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

// Per tick of a walk whose heartbeat of `command` stops at `cut` ticks, or never when `cut` is negative: the joint
// angles, the body height the motion eases toward and the lift the gait eases toward. A dead-man that takes the neutral
// posture once the walk is still starts by lowering the lift, which moves no joint at rest.
using MotionSample = std::array<float, 20>;
std::vector<MotionSample> walkWithHeartbeatUntil(const CommandMsg &command, int cut, int ticks) {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    std::vector<MotionSample> trace;
    for (int t = 0; t < ticks; ++t) {
        if ((cut < 0 || t < cut) && t % COMMAND_PERIOD_TICKS == 0) robot.command(command);
        robot.tick();
        MotionSample sample;
        std::copy_n(robot.angles(), 18, sample.begin());
        sample[18] = robot.bodyCommand().zm;
        sample[19] = robot.gaitCommand().step_height;
        trace.push_back(sample);
    }
    return trace;
}

// How long after its last command a walk that stops hearing `command` still moves and aims exactly as one that hears
// it, in ms.
float msObeyedAfterSilence(const CommandMsg &command, float seconds) {
    const int lastCommand = ticksFor(1.0f);
    const std::vector<MotionSample> heard = walkWithHeartbeatUntil(command, -1, ticksFor(seconds));
    const std::vector<MotionSample> cutOff = walkWithHeartbeatUntil(command, lastCommand + 1, ticksFor(seconds));
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
    largestStep(robot, ticksFor(0.6f + 1.5f)); // the timeout, then the stop's two-stage ramp and the feet's ease
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

// A zero stick held in WALK long enough for the walk to take its posture, so the next stick walks at once.
void settleWalk(VirtualRobot &robot) { hold(robot, CommandMsg {}, ticksFor(1.0f)); }

struct Posture3 {
    float s1, h, fd;
};

CommandMsg withPosture(CommandMsg c, const Posture3 &p) {
    c.s1 = p.s1;
    c.h = p.h;
    c.fd = p.fd;
    return c;
}

// The walk is driving a stroke at a cadence, not only stepping a stance change in.
bool walkResumed(VirtualRobot &robot) {
    const gait_state_t &target = robot.gaitCommand();
    return target.phase_rate > 0.0f && std::hypot(target.step_x, target.step_y) + std::fabs(target.step_angle) > 0.0f;
}

// The steady full-stick walk moves a joint up to about 3.9 deg in a tick, just past the servo's 3.75 deg (13.1 rad/s
// over 5 ms); a gait swapped under a swinging foot jumped 22-29 deg. 6 deg tells the two apart.
constexpr float SWAP_STEP_BOUND_DEG = 6.0f;

// The lift a stand steps its stance at (motion.h DEFAULT_STEP_HEIGHT_MM), which a posture settle starts by taking.
constexpr float DEFAULT_LIFT_MM = 15.0f;

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
    for (const Case &k : {Case {-0.5f, GaitType::TRI_GATE}, Case {1.0f, GaitType::BI_GATE}}) {
        VirtualRobot robot;
        standUp(robot);
        robot.setGait(GaitType::AUTO);
        robot.tick();
        TEST_ASSERT_EQUAL(GaitType::RIPPLE, robot.gait());
        robot.setMode(MOTION_STATE::WALK);
        settleWalk(robot);
        TEST_ASSERT_EQUAL(GaitType::RIPPLE, robot.gait());

        robot.command({0, 1, 0, 0, 0, k.s, 0, 0});
        robot.tick();
        TEST_ASSERT_EQUAL(k.selected, robot.gait());
        const AutoDemand demand = autoDemand(0, 1, 0, k.s, 0, 0, 0);
        const WalkTarget left = autoWalkTarget(GaitType::RIPPLE, demand, 0, 1, 0, 0, 0, 0);
        const WalkTarget expected = autoWalkTarget(k.selected, demand, 0, 1, 0, 0, 0, 0);
        TEST_ASSERT_TRUE(std::fabs(expected.phaseRate - left.phaseRate) > 0.1f * expected.phaseRate);
        const gait_state_t &target = robot.gaitCommand();
        TEST_ASSERT_EQUAL_FLOAT(expected.stepX, target.step_x);
        TEST_ASSERT_EQUAL_FLOAT(expected.stepY, target.step_y);
        TEST_ASSERT_EQUAL_FLOAT(expected.stepAngle, target.step_angle);
        TEST_ASSERT_EQUAL_FLOAT(expected.phaseRate, target.phase_rate);
    }
}

// Narrowing the stance at full stick stops the walk, steps the stance in at rest and walks on: no joint leaves its
// travel on the way, and the walk is back on its own, with a stroke, by the end.
void test_feet_distance_change_while_walking_stays_in_travel() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (const CommandMsg &stick : {fullStick(0, 1, 0), fullStick(1, 1, 1)}) {
            VirtualRobot robot;
            walkIn(robot, gait);
            CommandMsg c = stick;
            HeldRun run = hold(robot, c, ticksFor(1.0f));
            c.fd = -1.0f;
            const HeldRun narrowed = hold(robot, c, ticksFor(5.0f));
            run.clamped += narrowed.clamped;
            run.largestStep = narrowed.largestStep;
            char msg[160];
            describe(msg, sizeof(msg), gait, c, run);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
            TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
        }
    }
}

// A posture outside the walkable region walks the robot exactly as the walkable posture it clamps to: the body
// height, the stance and the walk's envelope all come from the clamped one.
void test_unwalkable_posture_is_clamped() {
    float s1 = 0.0f, h = 1.0f, fd = -1.0f;
    TEST_ASSERT_FALSE(postureWalkable(s1, h, fd));
    clampPosture(s1, h, fd);
    TEST_ASSERT_TRUE(postureWalkable(s1, h, fd));
    TEST_ASSERT_TRUE(h < 1.0f && fd > -1.0f);

    const auto asked = traceHeld(MOTION_STATE::WALK, {0, 1, 0, 0, 1.0f, 0, 0, -1.0f}, ticksFor(3.0f));
    const auto clamped = traceHeld(MOTION_STATE::WALK, {0, 1, 0, 0, h, 0, s1, fd}, ticksFor(3.0f));
    TEST_ASSERT_TRUE(asked == clamped);
}

// The dead-man zeroes the stick it last heard, not just the stroke: an AUTO walk that sped up while stepping, and so
// still holds its slow gait, must not have the faster gait chosen and walked once the timeout has stopped it.
void test_a_timed_out_auto_walk_stays_at_rest() {
    VirtualRobot robot;
    standUp(robot);
    robot.setGait(GaitType::AUTO);
    robot.tick();
    robot.setMode(MOTION_STATE::WALK);
    hold(robot, {0, 0.1f, 0, 0, 0, 0, 0, 0}, ticksFor(1.0f));
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
    settleWalk(robot);
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
    // Long enough to take TRI's posture from TUNED's and settle in it, short of the 2 s link timeout.
    largestStep(robot, ticksFor(1.5f));
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, largestStep(robot, ticksFor(0.35f)));
}

// A posture change while walking stops the walk, takes the posture at rest and walks on from there, so the mixed
// postures its parts pass through on the way, which can leave the joints' travel, are never walked. Entering the walk
// from a stand posture the walk clamps is such a change too. These are the worst transitions of a 2688-case probe
// (4 gaits, 8 full sticks, 7 x 12 postures) of the walk that stepped through them.
void test_a_posture_change_while_walking_stays_in_travel() {
    const Posture3 changes[][2] = {{{0, 0, 0}, {1, 1, 1}},    {{0, 0, -1}, {1, 1, 1}}, {{0, 0, -1}, {0, 1, 0}},
                                   {{0, 1, 0}, {-1, -1, -1}}, {{1, 0, 0}, {0, 0, -1}}, {{0, -1, 0}, {1, 1, 1}}};
    for (const GaitType gait : {GaitType::TRI_GATE, GaitType::WAVE}) {
        for (const CommandMsg &stick : {fullStick(0, 1, 0), fullStick(1, 1, 1)}) {
            for (const auto &change : changes) {
                VirtualRobot robot;
                standUp(robot);
                robot.setGait(gait);
                robot.tick();
                HeldRun run = hold(robot, withPosture(CommandMsg {}, change[0]), ticksFor(2.0f));
                robot.setMode(MOTION_STATE::WALK);
                run.clamped += hold(robot, withPosture(stick, change[0]), ticksFor(2.0f)).clamped;
                // The slowest pause, WAVE stepping a wide stance under a body that then rises, takes about 4 s.
                run.clamped += hold(robot, withPosture(stick, change[1]), ticksFor(5.0f)).clamped;
                const HeldRun walking = hold(robot, withPosture(stick, change[1]), ticksFor(1.0f));
                run.clamped += walking.clamped;
                run.largestStep = walking.largestStep;
                char msg[200];
                std::snprintf(msg, sizeof(msg),
                              "gait %d stick (%.0f %.0f %.0f) (%.0f %.0f %.0f) -> (%.0f %.0f %.0f): %d clamped, "
                              "then largest step %.2f deg",
                              (int)gait, stick.lx, stick.ly, stick.rx, change[0].s1, change[0].h, change[0].fd,
                              change[1].s1, change[1].h, change[1].fd, run.clamped, run.largestStep);
                TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
                TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(STEPPING_DEG_PER_TICK, run.largestStep, msg);
                TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
            }
        }
    }
}

// The walk waits out a posture change by itself: with the stick held throughout, the stroke drops to zero at the
// change and comes back as the new posture's own, without a new stick.
void test_a_walk_resumes_by_itself_after_a_posture_change() {
    VirtualRobot robot;
    walkIn(robot, GaitType::TRI_GATE);
    const CommandMsg stick = fullStick(0, 1, 0);
    hold(robot, stick, ticksFor(2.0f));
    TEST_ASSERT_EQUAL_FLOAT(classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, 0, 0, 0).stepY,
                            robot.gaitCommand().step_y);

    CommandMsg raised = stick;
    raised.h = 1.0f;
    robot.command(raised);
    robot.tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.gaitCommand().step_y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.gaitCommand().phase_rate);

    hold(robot, raised, ticksFor(3.0f));
    float s1 = raised.s1, h = raised.h, fd = raised.fd;
    clampPosture(s1, h, fd);
    const WalkTarget expected = classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, s1, h, fd);
    TEST_ASSERT_EQUAL_FLOAT(expected.stepY, robot.gaitCommand().step_y);
    TEST_ASSERT_EQUAL_FLOAT(expected.phaseRate, robot.gaitCommand().phase_rate);
    TEST_ASSERT_GREATER_THAN_FLOAT(STEPPING_DEG_PER_TICK, hold(robot, raised, ticksFor(1.0f)).largestStep);
}

// A posture change within the 1/32 deadband does not stop the walk; one beyond it does.
void test_a_posture_change_below_the_deadband_does_not_pause() {
    for (const float dh : {1.0f / 64.0f, 1.0f / 16.0f}) {
        VirtualRobot robot;
        walkIn(robot, GaitType::TRI_GATE);
        const CommandMsg stick = fullStick(0, 1, 0);
        hold(robot, stick, ticksFor(2.0f));
        CommandMsg nudged = stick;
        nudged.h = dh;
        bool paused = false;
        for (int t = 0; t < ticksFor(1.0f); ++t) {
            if (t % COMMAND_PERIOD_TICKS == 0) robot.command(nudged);
            robot.tick();
            paused |= robot.gaitCommand().step_y == 0.0f;
        }
        TEST_ASSERT_EQUAL_MESSAGE(dh > 1.0f / 32.0f, paused, dh > 1.0f / 32.0f ? "beyond" : "within");
    }
}

// A stand takes postures a walk would clamp, as long as the legs can stand there: h = +1 over the neutral stance
// stands at the full height.
void test_a_stand_accepts_a_posture_the_walk_would_clamp() {
    Kinematics kin;
    TEST_ASSERT_FALSE(postureWalkable(0, 1, 0));
    TEST_ASSERT_TRUE(standFeasible(kin, 1, 0));

    VirtualRobot robot;
    standUp(robot);
    const HeldRun run = hold(robot, {0, 0, 0, 0, 1, 0, 0, 0}, ticksFor(3.0f));
    BodyStateMsg raised {};
    raised.zm = bodyHeightMm(1);
    scaleStance(feetScale(0), raised.feet);
    float expected[18];
    kin.inverseKinematics(raised, expected);
    TEST_ASSERT_EQUAL_INT(0, run.clamped);
    for (int j = 0; j < 18; ++j) TEST_ASSERT_FLOAT_WITHIN(1e-2f, expected[j], robot.angles()[j]);
}

// A posture the legs cannot stand in is pulled in to the outermost one they can, and stood in exactly as if that
// one had been commanded.
void test_an_infeasible_stand_posture_is_pulled_in() {
    Kinematics kin;
    float h = 1.0f, fd = -1.0f;
    TEST_ASSERT_FALSE(standFeasible(kin, h, fd));
    clampStandPosture(kin, h, fd);
    TEST_ASSERT_TRUE(standFeasible(kin, h, fd));
    TEST_ASSERT_TRUE(h < 1.0f && fd > -1.0f);

    const auto asked = traceHeld(MOTION_STATE::STAND, {0, 0, 0, 0, 1.0f, 0, 0, -1.0f}, ticksFor(4.0f));
    const auto pulled = traceHeld(MOTION_STATE::STAND, {0, 0, 0, 0, h, 0, 0, fd}, ticksFor(4.0f));
    TEST_ASSERT_TRUE(asked == pulled);
    // Settled, the stand is within travel; the stance change on the way may brush it, as any stand's does.
    for (int j = 0; j < 18; ++j) TEST_ASSERT_EQUAL_FLOAT(withinTravel(asked.back()[j], j % 3), asked.back()[j]);
}

// A gait change during a posture settle waits for it: the settle steps the stance with a zero stroke, so the gait is
// not commanded while a foot swings, and a swap there reassigns that foot's phase. The swap comes once the settle is
// done, manual or AUTO.
void test_a_gait_change_during_a_settle_waits_for_it() {
    const Posture3 moved {0.0f, -0.5f, -1.0f};
    for (const float delay : {0.5f, 1.0f, 1.3f, 1.6f}) {
        VirtualRobot robot;
        walkIn(robot, GaitType::WAVE);
        const CommandMsg stick = fullStick(0, 1, 0);
        hold(robot, stick, ticksFor(3.0f));
        hold(robot, withPosture(stick, moved), ticksFor(delay));
        robot.setGait(GaitType::TRI_GATE);
        const HeldRun run = hold(robot, withPosture(stick, moved), ticksFor(4.0f));
        char msg[96];
        std::snprintf(msg, sizeof(msg), "manual, %.1f s into the settle: largest step %.2f deg", delay,
                      run.largestStep);
        TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(SWAP_STEP_BOUND_DEG, run.largestStep, msg);
        TEST_ASSERT_EQUAL_MESSAGE(GaitType::TRI_GATE, robot.gait(), msg);
        TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
    }
    for (const float delay : {0.3f, 0.9f}) {
        VirtualRobot robot;
        standUp(robot);
        robot.setGait(GaitType::AUTO);
        robot.tick();
        robot.setMode(MOTION_STATE::WALK);
        const CommandMsg slow {0, 0.1f, 0, 0, 0, 0, 0, 0};
        hold(robot, slow, ticksFor(3.0f));
        TEST_ASSERT_EQUAL(GaitType::RIPPLE, robot.gait());
        hold(robot, withPosture(slow, moved), ticksFor(delay));
        const HeldRun run = hold(robot, withPosture(fullStick(0, 1, 0), moved), ticksFor(4.0f));
        char msg[96];
        std::snprintf(msg, sizeof(msg), "AUTO, pushed %.1f s into the settle: largest step %.2f deg", delay,
                      run.largestStep);
        TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(SWAP_STEP_BOUND_DEG, run.largestStep, msg);
        TEST_ASSERT_EQUAL_MESSAGE(GaitType::BI_GATE, robot.gait(), msg);
        TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
    }
}

// Leaving a full-stick walk for STAND on the tick the stick is released: the stand's posture (h = 1, which the walk
// clamped) waits until the walk has stopped and its feet are down, so the body never rises over a stepping gait.
void test_leaving_a_walk_for_stand_mid_stride_stays_in_travel() {
    for (const GaitType gait : {GaitType::TRI_GATE, GaitType::BI_GATE, GaitType::WAVE}) {
        VirtualRobot robot;
        standUp(robot);
        robot.setGait(gait);
        robot.tick();
        const CommandMsg raised {0, 0, 0, 0, 1.0f, 1, 0, 0};
        hold(robot, raised, ticksFor(2.0f));
        robot.setMode(MOTION_STATE::WALK);
        CommandMsg walking = raised;
        walking.ly = 1.0f;
        HeldRun run = hold(robot, walking, ticksFor(4.0f));
        robot.command(raised);
        robot.setMode(MOTION_STATE::STAND);
        run.clamped += hold(robot, raised, ticksFor(3.0f)).clamped;
        char msg[48];
        std::snprintf(msg, sizeof(msg), "gait %d", (int)gait);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
        TEST_ASSERT_EQUAL_MESSAGE(MOTION_STATE::STAND, robot.mode(), msg);
    }
}

// WALK_NN moves the ride height and the feet without a command the walk hears, so a walk entered from it starts from
// a zero stroke and takes its posture again before it steps, even within the dead-man's 600 ms.
void test_a_walk_after_walk_nn_takes_its_posture_again() {
    VirtualRobot robot;
    walkIn(robot, GaitType::TRI_GATE);
    const CommandMsg stick = fullStick(0, 1, 0);
    hold(robot, stick, ticksFor(2.0f));
    TEST_ASSERT_TRUE(walkResumed(robot));

    robot.setMode(MOTION_STATE::WALK_NN);
    largestStep(robot, ticksFor(0.4f));
    robot.setMode(MOTION_STATE::WALK);
    robot.tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.gaitCommand().step_y);
    robot.command(stick);
    robot.tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.gaitCommand().step_y);
    const HeldRun run = hold(robot, stick, ticksFor(3.0f));
    TEST_ASSERT_EQUAL_INT(0, run.clamped);
    TEST_ASSERT_TRUE(walkResumed(robot));
}

// A settle cut short by WALK_NN starts over on the way back. Cut in its last stage, raising the lift, which checks
// nothing else, a stale settle would release the walk as soon as the lift arrived, over whatever height and stance the
// policy left; a fresh one first takes the stand's lift and the posture again.
void test_an_interrupted_settle_starts_over() {
    VirtualRobot robot;
    walkIn(robot, GaitType::TRI_GATE);
    const CommandMsg stick = fullStick(0, 1, 0);
    hold(robot, stick, ticksFor(2.0f));
    const CommandMsg moved = withPosture(stick, {1.0f, -0.5f, -1.0f});
    float s1 = moved.s1, h = moved.h, fd = moved.fd;
    clampPosture(s1, h, fd);
    const float walkLift = (s1 + 1.0f) * 20.0f;
    TEST_ASSERT_TRUE(walkLift > DEFAULT_LIFT_MM + 1.0f);
    bool lowered = false;
    int t = 0;
    for (; t < ticksFor(5.0f); ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(moved);
        robot.tick();
        lowered |= robot.gaitCommand().step_height == DEFAULT_LIFT_MM;
        if (lowered && robot.gaitCommand().step_height == walkLift && robot.gaitCommand().step_y == 0.0f) break;
    }
    TEST_ASSERT_TRUE_MESSAGE(t < ticksFor(5.0f), "the settle never reached its last stage");

    robot.setMode(MOTION_STATE::WALK_NN);
    largestStep(robot, ticksFor(0.4f));
    robot.setMode(MOTION_STATE::WALK);
    bool liftTakenAgain = false;
    HeldRun run;
    for (t = 0; t < ticksFor(5.0f) && !walkResumed(robot); ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(moved);
        robot.tick();
        liftTakenAgain |= robot.gaitCommand().step_height == DEFAULT_LIFT_MM;
        for (int j = 0; j < 18; ++j) run.clamped += withinTravel(robot.angles()[j], j % 3) != robot.angles()[j];
    }
    TEST_ASSERT_TRUE(liftTakenAgain);
    TEST_ASSERT_EQUAL_INT(0, run.clamped);
    const WalkTarget expected = classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, s1, h, fd);
    TEST_ASSERT_EQUAL_FLOAT(expected.stepY, robot.gaitCommand().step_y);
    TEST_ASSERT_EQUAL_FLOAT(expected.phaseRate, robot.gaitCommand().phase_rate);
}

// A walk entered from a stand at rest steps on its first command when the command keeps the stand's height and stance:
// nothing has to settle, and the lift rises to the walk's alongside the stroke.
void test_a_walk_from_a_stand_at_rest_steps_at_once() {
    for (const Posture3 posture : {Posture3 {0, 0, 0}, Posture3 {0.5f, 0.25f, 0.25f}}) {
        TEST_ASSERT_TRUE(postureWalkable(posture.s1, posture.h, posture.fd));
        VirtualRobot robot;
        standUp(robot);
        hold(robot, withPosture(CommandMsg {}, {0, posture.h, posture.fd}), ticksFor(2.0f));
        robot.setMode(MOTION_STATE::WALK);
        const CommandMsg stick = withPosture(fullStick(0, 1, 0), posture);
        robot.command(stick);
        robot.tick();
        const WalkTarget expected =
            classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, posture.s1, posture.h, posture.fd);
        TEST_ASSERT_EQUAL_FLOAT(expected.stepY, robot.gaitCommand().step_y);
        TEST_ASSERT_EQUAL_FLOAT(expected.phaseRate, robot.gaitCommand().phase_rate);
        TEST_ASSERT_EQUAL_FLOAT((posture.s1 + 1.0f) * 20.0f, robot.gaitCommand().step_height);
        TEST_ASSERT_GREATER_THAN_FLOAT(STEPPING_DEG_PER_TICK, largestStep(robot, ticksFor(0.1f), &stick));
        TEST_ASSERT_EQUAL_INT(0, hold(robot, stick, ticksFor(3.0f)).clamped);
    }
}

// From the same stand, a command at another height, or one that lowers the lift below the stand's, settles first.
void test_a_walk_from_a_stand_at_rest_settles_a_new_posture_first() {
    for (const Posture3 posture : {Posture3 {0, 0.25f, 0}, Posture3 {-1, 0, 0}}) {
        VirtualRobot robot;
        standUp(robot);
        robot.setMode(MOTION_STATE::WALK);
        const CommandMsg stick = withPosture(fullStick(0, 1, 0), posture);
        robot.command(stick);
        robot.tick();
        TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.gaitCommand().step_y);
        const HeldRun run = hold(robot, stick, ticksFor(3.0f));
        TEST_ASSERT_EQUAL_INT(0, run.clamped);
        TEST_ASSERT_TRUE(walkResumed(robot));
    }
}

// A TUNED walk between the stand and the first classic command moves the body to its own ride height and lift, so the
// classic walk cannot start in the stand's posture: it settles first and walks at the classic height.
void test_a_tuned_walk_after_the_stand_spends_its_rest_posture() {
    VirtualRobot robot;
    standUp(robot);
    hold(robot, CommandMsg {}, ticksFor(1.0f));
    robot.setMode(MOTION_STATE::WALK);
    robot.setGait(GaitType::TUNED);
    robot.tick();
    TEST_ASSERT_EQUAL(GaitType::TUNED, robot.gait());
    hold(robot, fullStick(0, 1, 0), ticksFor(2.0f));
    TEST_ASSERT_TRUE(robot.bodyCommand().zm < -1.0f);
    hold(robot, CommandMsg {}, ticksFor(2.0f));
    // The stick follows the swap within the tick, as a heartbeat can, before the control task has seen the swap.
    robot.setGait(GaitType::TRI_GATE);
    const CommandMsg stick = fullStick(0, 1, 0);
    robot.command(stick);
    robot.tick();
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, robot.gait());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.gaitCommand().step_y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, robot.gaitCommand().phase_rate);
    HeldRun run;
    for (int t = 0; t < ticksFor(5.0f) && !walkResumed(robot); ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(stick);
        robot.tick();
        for (int j = 0; j < 18; ++j) run.clamped += withinTravel(robot.angles()[j], j % 3) != robot.angles()[j];
    }
    TEST_ASSERT_TRUE(walkResumed(robot));
    TEST_ASSERT_EQUAL_FLOAT(bodyHeightMm(0), robot.bodyCommand().zm);
    TEST_ASSERT_EQUAL_INT(0, run.clamped + hold(robot, stick, ticksFor(2.0f)).clamped);
}

namespace {

enum class Stop { RELEASE, POSTURE_PAUSE, STAND, DEAD_MAN };

const char *stopName(Stop stop) {
    switch (stop) {
        case Stop::RELEASE: return "release";
        case Stop::POSTURE_PAUSE: return "posture pause";
        case Stop::STAND: return "stand";
        case Stop::DEAD_MAN: return "dead-man";
    }
    return "";
}

struct StopRun {
    float walkPeak = 0.0f; // the largest joint step in a tick of the steady walk
    HeldRun stop;          // the stop, from its first tick until the feet are home
    bool onSpeedLaw = true;   // the cadence kept in proportion to the stroke, as a slower command's
    bool cadenceLeft = false; // the gait stopped stepping with an explicit cadence still set
};

// Walks `stick` at its posture from a stand in that posture, then stops it the way `stop` does, `phase` ticks later in
// the cycle. The stop is followed until the feet have eased home after the gait stopped stepping, or until the motion
// moves the body to another posture, which a posture pause and the dead-man do once the gait is still.
StopRun walkThenStop(GaitType gait, const CommandMsg &stick, Stop stop, int phase) {
    VirtualRobot robot;
    standUp(robot);
    robot.setGait(gait);
    robot.tick();
    hold(robot, withPosture(CommandMsg {}, {0, stick.h, stick.fd}), ticksFor(2.0f));
    robot.setMode(MOTION_STATE::WALK);
    hold(robot, stick, ticksFor(2.5f));
    StopRun run;
    run.walkPeak = hold(robot, stick, ticksFor(1.5f) + phase).largestStep;

    CommandMsg after = stick;
    after.lx = after.ly = after.rx = 0;
    if (stop == Stop::POSTURE_PAUSE) {
        after = stick;
        after.s1 = stick.s1 > 0.0f ? stick.s1 - 0.5f : stick.s1 + 0.5f;
    }
    if (stop == Stop::STAND) {
        robot.command(after);
        robot.setMode(MOTION_STATE::STAND);
    }
    const gait_state_t walked = robot.gaitState();
    // The stroke along the axis the walk drove furthest past its deadband.
    const auto stroke = [&walked](const gait_state_t &g) {
        const float x = std::fabs(walked.step_x) / GAIT_STRIDE_DEADBAND_MM;
        const float y = std::fabs(walked.step_y) / GAIT_STRIDE_DEADBAND_MM;
        const float yaw = std::fabs(walked.step_angle) / GAIT_YAW_DEADBAND_RAD;
        return std::fabs(x >= y && x >= yaw ? g.step_x : y >= yaw ? g.step_y : g.step_angle);
    };
    const float zm = robot.bodyCommand().zm;
    float previous[18];
    std::copy_n(robot.angles(), 18, previous);
    int still = 0;
    for (int t = 0; t < ticksFor(5.0f) && still < ticksFor(FEET_EASE_S); ++t) {
        if (stop != Stop::DEAD_MAN && t % COMMAND_PERIOD_TICKS == 0) robot.command(after);
        robot.tick();
        if (robot.bodyCommand().zm != zm) break;
        for (int j = 0; j < 18; ++j) {
            const float angle = robot.angles()[j];
            if (withinTravel(angle, j % 3) != angle) run.stop.clamped++;
            run.stop.largestStep = std::max(run.stop.largestStep, std::fabs(angle - previous[j]));
            previous[j] = angle;
        }
        const gait_state_t &live = robot.gaitState();
        if (gaitIsCommanded(live)) {
            run.onSpeedLaw &= std::fabs(live.phase_rate * stroke(walked) - walked.phase_rate * stroke(live)) <=
                              1e-4f * walked.phase_rate * stroke(walked);
        } else {
            run.cadenceLeft |= still == 0 && live.phase_rate != 0.0f;
            ++still;
        }
    }
    return run;
}

} // namespace

// A stop ramps the stroke out over several cycles. Stepped at the stride law's cadence, up to 1.5 cycles/s, that stroke
// moved a joint up to 10.4 deg in a tick, where WAVE and RIPPLE walk theirs below 1 cycle/s. At the walk's own cadence
// a stroke shrinking at its fastest from the first tick moved a foot at the edge of its reach up to 6.7 deg in a tick,
// depending on where in the cycle the stop came, and a stroke near zero lifted the feet as fast as the walk did, up to
// 4.5 deg. Eased out from rest, with the cadence shrinking alongside, a stop passes only through walks a slower command
// would take. Every way a classic walk stops is checked, at a slow and a fast stick, in postures from neutral to the
// envelope's edges, a release at eight points of the cycle, and no stop may move a joint faster than
// STOP_STEP_BOUND_DEG, leave travel, leave the speed law, or leave a cadence set once the gait has stopped.
void test_every_stop_ramps_down_on_the_speed_law() {
    // The steady walks below peak at 3.95 deg in a tick (measured), just past the servo's 3.75 deg (13.1 rad/s over 5
    // ms), and their stops at 3.96 deg.
    constexpr float STOP_STEP_BOUND_DEG = 4.0f;
    constexpr int PHASES = 8, PHASE_STEP_TICKS = 7; // 0.28 s, half a fast gait's cycle
    const Posture3 postures[] = {{0, 0, 0}, {1, -1, 0}, {1, -1, -1}, {-1, -1, 0}, {1, 0.5f, 0.5f}, {1, 1, 1}};
    float walkPeak = 0.0f, stopPeak = 0.0f;
    for (const GaitType gait : CLASSIC_GAITS) {
        for (Posture3 posture : postures) {
            clampPosture(posture.s1, posture.h, posture.fd);
            for (const float s : {0.0f, 1.0f}) {
                for (CommandMsg stick : FULL_STICKS) {
                    stick.s = s;
                    stick = withPosture(stick, posture);
                    for (int k = 0; k < PHASES + 3; ++k) {
                        const Stop others[] = {Stop::POSTURE_PAUSE, Stop::STAND, Stop::DEAD_MAN};
                        const Stop stop = k < PHASES ? Stop::RELEASE : others[k - PHASES];
                        const int phase = k < PHASES ? k * PHASE_STEP_TICKS : 0;
                        const StopRun run = walkThenStop(gait, stick, stop, phase);
                        walkPeak = std::max(walkPeak, run.walkPeak);
                        stopPeak = std::max(stopPeak, run.stop.largestStep);
                        char msg[200];
                        std::snprintf(msg, sizeof(msg),
                                      "%s %d ticks on, gait %d stick (%.0f %.0f %.0f) s %.0f posture (%.2f %.2f %.2f): "
                                      "walk peak %.2f, stop peak %.2f deg, %d clamped",
                                      stopName(stop), phase, (int)gait, stick.lx, stick.ly, stick.rx, s, posture.s1,
                                      posture.h, posture.fd, run.walkPeak, run.stop.largestStep, run.stop.clamped);
                        TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(STOP_STEP_BOUND_DEG, run.stop.largestStep, msg);
                        TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.stop.clamped, msg);
                        TEST_ASSERT_TRUE_MESSAGE(run.onSpeedLaw, msg);
                        TEST_ASSERT_FALSE_MESSAGE(run.cadenceLeft, msg);
                    }
                }
            }
        }
    }
    char summary[96];
    std::snprintf(summary, sizeof(summary), "walk peak %.2f, stop peak %.2f deg per tick", walkPeak, stopPeak);
    TEST_MESSAGE(summary);
}

namespace {

// Whether the robot stands still in the neutral posture, h = fd = 0 and the body level and centred, as a stand there
// would. A stance stepped there ends within 0.5 mm of it (GaitController::hasPendingStanceChange), up to half a degree
// (measured); a quarter of the feet distance away is 9 deg.
bool standsInNeutralPosture(VirtualRobot &robot) {
    Kinematics kin;
    BodyStateMsg neutral {};
    neutral.zm = bodyHeightMm(0);
    scaleStance(feetScale(0), neutral.feet);
    float expected[18];
    kin.inverseKinematics(neutral, expected);
    for (int j = 0; j < 18; ++j)
        if (std::fabs(expected[j] - robot.angles()[j]) > 1.0f) return false;
    return largestStep(robot, ticksFor(0.5f)) < 1e-3f;
}

} // namespace

// The dead-man zeroes the posture as well as the stick. Written into the body height while the walk still stepped, it
// took the legs through the mixed postures between and out of travel (44 of 960 probed stops). The zero posture is
// taken like any other instead: after the walk has stopped, settled in stages.
void test_a_dead_man_stop_takes_the_neutral_posture_once_stopped() {
    const Posture3 postures[] = {{1, -1, -1}, {-1, -1, 0}, {-0.75f, -0.75f, 0.375f}, {0, 1, 0}, {1, 1, 1}, {0, 0, -1}};
    for (const GaitType gait : CLASSIC_GAITS) {
        for (Posture3 posture : postures) {
            clampPosture(posture.s1, posture.h, posture.fd);
            for (const CommandMsg &full :
                 {fullStick(0, 1, 0), fullStick(1, 0, 0), fullStick(0, 0, 1), fullStick(1, 1, 1)}) {
                const CommandMsg stick = withPosture(full, posture);
                VirtualRobot robot;
                standUp(robot);
                robot.setGait(gait);
                robot.tick();
                hold(robot, withPosture(CommandMsg {}, {0, posture.h, posture.fd}), ticksFor(2.0f));
                robot.setMode(MOTION_STATE::WALK);
                hold(robot, stick, ticksFor(3.0f));
                HeldRun run;
                float previous[18];
                std::copy_n(robot.angles(), 18, previous);
                for (int t = 0; t < ticksFor(6.0f); ++t) {
                    robot.tick();
                    for (int j = 0; j < 18; ++j) {
                        const float angle = robot.angles()[j];
                        if (withinTravel(angle, j % 3) != angle) run.clamped++;
                        run.largestStep = std::max(run.largestStep, std::fabs(angle - previous[j]));
                        previous[j] = angle;
                    }
                }
                char msg[160];
                describe(msg, sizeof(msg), gait, stick, run);
                TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
                TEST_ASSERT_EQUAL_MESSAGE(MOTION_STATE::WALK, robot.mode(), msg);
                TEST_ASSERT_TRUE_MESSAGE(standsInNeutralPosture(robot), msg);
            }
        }
    }
}

// The same in a stand: the link lost as the operator leaves a walk for STAND, so the dead-man fires while the walk
// still ramps out under the stand. The zero posture waits until the feet are still, as any stand posture does.
void test_a_dead_man_stop_in_a_stand_waits_for_the_feet() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (Posture3 posture : {Posture3 {1, -1, -1}, Posture3 {-1, -1, 0}, Posture3 {0, 1, 0}}) {
            clampPosture(posture.s1, posture.h, posture.fd);
            const CommandMsg stick = withPosture(fullStick(0, 1, 0), posture);
            VirtualRobot robot;
            standUp(robot);
            robot.setGait(gait);
            robot.tick();
            hold(robot, withPosture(CommandMsg {}, {0, posture.h, posture.fd}), ticksFor(2.0f));
            robot.setMode(MOTION_STATE::WALK);
            hold(robot, stick, ticksFor(3.0f));
            robot.setMode(MOTION_STATE::STAND);
            HeldRun run;
            for (int t = 0; t < ticksFor(6.0f); ++t) {
                robot.tick();
                for (int j = 0; j < 18; ++j) run.clamped += withinTravel(robot.angles()[j], j % 3) != robot.angles()[j];
            }
            char msg[160];
            describe(msg, sizeof(msg), gait, stick, run);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
            TEST_ASSERT_EQUAL_MESSAGE(MOTION_STATE::STAND, robot.mode(), msg);
            TEST_ASSERT_TRUE_MESSAGE(standsInNeutralPosture(robot), msg);
        }
    }
}

// The envelope measured a level, centred body, so a classic walk eases the tilt stick and the stand's offsets out: a
// full stick under a full tilt clamped 119-646 joint-ticks in 3 s of TRI or BI.
void test_a_classic_walk_ignores_the_tilt_stick() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (Posture3 posture : {Posture3 {0, 0, 0}, Posture3 {1, 0, 0}, Posture3 {0, 1, 0}, Posture3 {0, 0, -1}}) {
            clampPosture(posture.s1, posture.h, posture.fd);
            for (const float ry : {1.0f, -1.0f}) {
                for (const CommandMsg &full :
                     {fullStick(0, 1, 0), fullStick(1, 0, 0), fullStick(0, 0, 1), fullStick(1, 1, 1)}) {
                    CommandMsg stick = withPosture(full, posture);
                    stick.ry = ry;
                    VirtualRobot robot;
                    walkIn(robot, gait);
                    const HeldRun run = hold(robot, stick, ticksFor(4.0f));
                    char msg[160];
                    describe(msg, sizeof(msg), gait, stick, run);
                    TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
                    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, robot.bodyCommand().omega, msg);
                    TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
                }
            }
        }
    }
}

// A shifted and tilted stand hands a walk a body the envelope did not measure; the walk eases it back to level and
// centred. The stands here are within travel themselves.
void test_a_classic_walk_after_a_shifted_stand_stays_in_travel() {
    const CommandMsg shifts[] = {{1, 0, 0, 0, 0, 0, 0, 0},       {0, -1, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 1, 0, 0, 0, 0, 0},       {0, 0, 0, -1, 0, 0, 0, 0},
                                 {0.5f, 0.5f, 0, 0, 0, 0, 0, 0}, {0.3f, -0.3f, 0.3f, -0.3f, 0, 0, 0, 0}};
    for (const GaitType gait : CLASSIC_GAITS) {
        for (const CommandMsg &shift : shifts) {
            for (const CommandMsg &stick :
                 {fullStick(0, 1, 0), fullStick(1, 0, 0), fullStick(0, 0, 1), fullStick(1, 1, 1)}) {
                VirtualRobot robot;
                standUp(robot);
                robot.setGait(gait);
                robot.tick();
                hold(robot, shift, ticksFor(2.0f));
                char msg[160];
                const HeldRun standing = hold(robot, shift, 1);
                describe(msg, sizeof(msg), gait, shift, standing);
                TEST_ASSERT_EQUAL_INT_MESSAGE(0, standing.clamped, msg);
                robot.setMode(MOTION_STATE::WALK);
                const HeldRun run = hold(robot, stick, ticksFor(3.0f));
                describe(msg, sizeof(msg), gait, stick, run);
                TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
                const BodyStateMsg &body = robot.bodyCommand();
                TEST_ASSERT_TRUE_MESSAGE(body.xm == 0.0f && body.ym == 0.0f && body.phi == 0.0f && body.omega == 0.0f,
                                         msg);
                TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
            }
        }
    }
}

// Self-levelling on the IMU readings measured on the robot on 2026-10-16 (imu_frame.h): with its right side 10 deg
// down the DMP's pitch reads -10 deg, and nose down its roll reads -10 deg. Levelling raises the low side, so the
// low side's legs reach down: a smaller femur angle than the high side's.
void test_self_levelling_reaches_down_with_the_low_side() {
    constexpr float TILT = 10.0f * static_cast<float>(M_PI) / 180.0f;
    auto femur = [](VirtualRobot &robot, int leg) { return robot.angles()[leg * 3 + 1]; };
    {
        VirtualRobot robot;
        standUp(robot);
        robot.setImu({{0, -TILT, 0}, {-std::sin(TILT), 0, std::cos(TILT)}, {0, 0, 0}});
        largestStep(robot, ticksFor(1.0f));
        for (int leg = 0; leg < 3; ++leg) TEST_ASSERT_LESS_THAN_FLOAT(femur(robot, leg + 3) - 5.0f, femur(robot, leg));
    }
    {
        VirtualRobot robot;
        standUp(robot);
        robot.setImu({{-TILT, 0, 0}, {0, -std::sin(TILT), std::cos(TILT)}, {0, 0, 0}});
        largestStep(robot, ticksFor(1.0f));
        for (int side = 0; side < 6; side += 3)
            TEST_ASSERT_LESS_THAN_FLOAT(femur(robot, side + 2) - 5.0f, femur(robot, side));
    }
    Peripherals::setImu({});
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
    RUN_TEST(test_a_posture_change_while_walking_stays_in_travel);
    RUN_TEST(test_a_walk_resumes_by_itself_after_a_posture_change);
    RUN_TEST(test_a_posture_change_below_the_deadband_does_not_pause);
    RUN_TEST(test_a_stand_accepts_a_posture_the_walk_would_clamp);
    RUN_TEST(test_an_infeasible_stand_posture_is_pulled_in);
    RUN_TEST(test_a_gait_change_during_a_settle_waits_for_it);
    RUN_TEST(test_leaving_a_walk_for_stand_mid_stride_stays_in_travel);
    RUN_TEST(test_a_walk_after_walk_nn_takes_its_posture_again);
    RUN_TEST(test_an_interrupted_settle_starts_over);
    RUN_TEST(test_a_walk_from_a_stand_at_rest_steps_at_once);
    RUN_TEST(test_a_walk_from_a_stand_at_rest_settles_a_new_posture_first);
    RUN_TEST(test_a_tuned_walk_after_the_stand_spends_its_rest_posture);
    RUN_TEST(test_every_stop_ramps_down_on_the_speed_law);
    RUN_TEST(test_a_dead_man_stop_takes_the_neutral_posture_once_stopped);
    RUN_TEST(test_a_dead_man_stop_in_a_stand_waits_for_the_feet);
    RUN_TEST(test_a_classic_walk_ignores_the_tilt_stick);
    RUN_TEST(test_a_classic_walk_after_a_shifted_stand_stays_in_travel);
    RUN_TEST(test_self_levelling_reaches_down_with_the_low_side);
    return UNITY_END();
}
