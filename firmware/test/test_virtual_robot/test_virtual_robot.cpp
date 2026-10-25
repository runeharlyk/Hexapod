// The firmware's Hexapod run off the robot (firmware/host/virtual_robot.h): commands reach the motion
// through its own EventBus subscriptions, and the clock is the control loop's 5 ms tick.

// The animation store reads the parity fixtures, whose "fx_single" holds its last keyframe until stopped.
#define ANIMATION_DIRECTORY "animations/fixtures"
#define USE_POLICY 1
#define USE_MPU6050 1

#include <unity.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>
#include <gait_speed.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <servo_angles_wire.h>
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

constexpr int HANDHELD_PERIOD_TICKS = 4; // the handheld's 50 Hz while a stick is off centre
constexpr int APP_HEARTBEAT_TICKS = 40;  // app/src/lib/control-link.ts COMMAND_HEARTBEAT_MS

// The handheld controller's frame, its axes as shares of full scale.
controller_packet_t handheldFrame(float lx, float ly, float rx = 0, float ry = 0) {
    const auto axis = [](float v) { return static_cast<int16_t>(std::lround(v * AXIS_FULL_SCALE)); };
    return {CONTROLLER_PACKET_VERSION, 0, axis(lx), axis(ly), axis(rx), axis(ry), 0};
}

}  // namespace

void setUp() {}
// The IMU stand-in is process-wide, so a test that tilts it must not hand the tilt on, even when it fails.
void tearDown() { Peripherals::setImu({}); }

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

// The boards wake with every channel off, so IDLE after a deactivation leaves the robot lying limp where it sagged,
// rather than restarting the stand it held before.
void test_idle_after_a_deactivated_stand_leaves_every_servo_limp() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::DEACTIVATED);
    robot.tick();
    robot.setMode(MOTION_STATE::IDLE);
    for (int t = 0; t < ticksFor(0.5f); ++t) robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::IDLE, robot.mode());
    TEST_ASSERT_FALSE(robot.servosDriven());
    TEST_ASSERT_EQUAL_HEX32(0, robot.drivenServos());
    robot.setJoint(4, 20.0f);
    robot.tick();
    TEST_ASSERT_EQUAL_HEX32(1u << 4, robot.drivenServos());
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

// The robot reports a mode only once the control task has applied it: each mode a request applies is published as
// APPLIED in the tick that applies it, and not when the request is delivered. The observers that show the robot's mode
// forward these alone.
void test_a_mode_is_reported_applied_in_the_tick_that_applies_it() {
    VirtualRobot robot;
    robot.tick();
    std::vector<MOTION_STATE> applied;
    auto listener = EventBus<ModeMsg>::subscribe([&applied](const ModeMsg &m) {
        if (m.kind == ModeMsgKind::APPLIED) applied.push_back(m.mode);
    });
    for (const MOTION_STATE mode : {MOTION_STATE::STAND, MOTION_STATE::WALK, MOTION_STATE::IDLE, MOTION_STATE::STAND,
                                    MOTION_STATE::DEACTIVATED}) {
        applied.clear();
        robot.setMode(mode);
        host_bus::drainAll();
        TEST_ASSERT_TRUE(applied.empty());
        robot.tick();
        TEST_ASSERT_EQUAL(mode, robot.mode());
        TEST_ASSERT_EQUAL_size_t(1, applied.size());
        TEST_ASSERT_EQUAL(mode, applied[0]);
    }
}

// The modes a joint drag or a centring must never move the robot in.
constexpr MOTION_STATE NOT_IDLE[] = {MOTION_STATE::DEACTIVATED, MOTION_STATE::STAND, MOTION_STATE::WALK,
                                     MOTION_STATE::WALK_NN, MOTION_STATE::ANIMATE};

void idleFromStand(VirtualRobot &robot) {
    standUp(robot);
    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::IDLE, robot.mode());
}

// IDLE stops the motion where it is: the servos keep the stand's angles and no pose is written over them, so a joint
// dragged first moves from where the robot stands.
void test_entering_idle_holds_the_servos_angles() {
    VirtualRobot robot;
    standUp(robot);
    float stood[18];
    std::copy_n(robot.servoAngles(), 18, stood);
    const unsigned writes = robot.servoWrites();
    robot.setMode(MOTION_STATE::IDLE);
    for (int t = 0; t < ticksFor(1.0f); ++t) robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::IDLE, robot.mode());
    TEST_ASSERT_EQUAL_UINT(writes, robot.servoWrites());
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(stood, robot.servoAngles(), 18);
}

// A joint dragged in IDLE moves that servo alone, from the next tick, and every other joint holds its angle. The tick
// reports the write of the joint's board, the right one for servo 4, and the tick after it none.
void test_a_joint_in_idle_moves_only_its_servo() {
    VirtualRobot robot;
    idleFromStand(robot);
    float held[18];
    std::copy_n(robot.servoAngles(), 18, held);
    robot.setJoint(4, 20.0f);
    TEST_ASSERT_EQUAL_FLOAT(held[4], robot.servoAngles()[4]);
    robot.tick();
    held[4] = 20.0f;
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(held, robot.servoAngles(), 18);
    TEST_ASSERT_EQUAL_UINT(1, robot.jointWrites());
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, robot.servoWrite());
    robot.tick();
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_NONE, robot.servoWrite());
}

void test_a_joint_in_idle_is_held_within_its_travel() {
    VirtualRobot robot;
    idleFromStand(robot);
    robot.setJoint(0, 80.0f);
    robot.setJoint(5, -170.0f);
    robot.tick();
    TEST_ASSERT_EQUAL_FLOAT(JOINT_LIMIT_DEG[0], robot.servoAngles()[0]);
    TEST_ASSERT_EQUAL_FLOAT(-JOINT_LIMIT_DEG[2], robot.servoAngles()[5]);
}

// Outside IDLE a joint is ignored: the motion owns the servos, and deactivated they are limp.
void test_a_joint_outside_idle_is_ignored() {
    for (const MOTION_STATE mode : NOT_IDLE) {
        VirtualRobot robot;
        standUp(robot);
        robot.setMode(mode);
        for (int t = 0; t < ticksFor(0.2f); ++t) robot.tick();
        TEST_ASSERT_EQUAL(mode, robot.mode());
        robot.setJoint(4, 20.0f);
        robot.tick();
        TEST_ASSERT_EQUAL_UINT(0, robot.jointWrites());
    }
}

// A joint dragged while a mode leaving IDLE is on its way finds that mode applied first, and moves nothing.
void test_a_joint_sent_behind_a_mode_out_of_idle_is_ignored() {
    VirtualRobot robot;
    idleFromStand(robot);
    robot.setMode(MOTION_STATE::STAND);
    robot.setJoint(4, 20.0f);
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
    TEST_ASSERT_EQUAL_UINT(0, robot.jointWrites());
}

// The calibration page's "Centre all servos": accepted, and carried out at the next tick, in IDLE alone.
void test_the_servos_are_centred_only_in_idle() {
    for (const MOTION_STATE mode : NOT_IDLE) {
        VirtualRobot robot;
        standUp(robot);
        robot.setMode(mode);
        for (int t = 0; t < ticksFor(0.2f); ++t) robot.tick();
        TEST_ASSERT_EQUAL_INT(409, robot.centreServos());
        robot.tick();
        TEST_ASSERT_EQUAL_UINT(0, robot.centrings());
    }
    VirtualRobot robot;
    idleFromStand(robot);
    TEST_ASSERT_EQUAL_INT(200, robot.centreServos());
    TEST_ASSERT_EQUAL_UINT(0, robot.centrings());
    robot.tick();
    TEST_ASSERT_EQUAL_UINT(1, robot.centrings());
}

// A centring answered 200 is carried out, in the order things arrived: before a mode that arrived after it, even one
// leaving IDLE, and over every joint dragged before it, which a joint dragged after it then moves again.
void test_a_centring_is_carried_out_in_arrival_order() {
    VirtualRobot robot;
    idleFromStand(robot);
    TEST_ASSERT_EQUAL_INT(200, robot.centreServos());
    robot.setMode(MOTION_STATE::STAND);
    robot.tick();
    TEST_ASSERT_EQUAL_UINT(1, robot.centrings());

    VirtualRobot joints;
    idleFromStand(joints);
    joints.setJoint(4, 20.0f);
    TEST_ASSERT_EQUAL_INT(200, joints.centreServos());
    joints.setJoint(7, 15.0f);
    joints.tick();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, joints.servoAngles()[4]);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, joints.servoAngles()[7]);
}

// A mode already on its way when the centring is asked for is applied first and may leave IDLE, so the centring is
// refused rather than answered 200 and dropped.
void test_a_centring_behind_a_queued_mode_is_refused() {
    VirtualRobot robot;
    idleFromStand(robot);
    robot.setMode(MOTION_STATE::STAND);
    host_bus::drainAll();
    TEST_ASSERT_EQUAL_INT(409, robot.centreServos());
    robot.tick();
    TEST_ASSERT_EQUAL_UINT(0, robot.centrings());
}

// The angles the robot publishes for the app (main.cpp bridges them to AnglesData), each publication in order.
using Published = std::vector<std::array<float, 18>>;

EventBus<ServoAnglesMsg>::Handle recordAngles(Published &heard) {
    return EventBus<ServoAnglesMsg>::subscribe([&heard](const ServoAnglesMsg &a) {
        std::array<float, 18> angles;
        std::copy_n(a.angles, 18, angles.begin());
        heard.push_back(angles);
    });
}

std::array<float, 18> servoAnglesOf(const VirtualRobot &robot) {
    std::array<float, 18> angles;
    std::copy_n(robot.servoAngles(), 18, angles.begin());
    return angles;
}

// IDLE publishes the angles its servos hold as it is entered, so the app draws the robot as it stands, and nothing
// more while nothing moves them: no publication on a timer. The telemetry reports the same angles.
void test_entering_idle_publishes_the_held_angles_once() {
    VirtualRobot robot;
    standUp(robot);
    const std::array<float, 18> stood = servoAnglesOf(robot);
    Published heard;
    auto listener = recordAngles(heard);
    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(stood.data(), heard[0].data(), 18);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(stood.data(), robot.angles(), 18);
    for (int t = 0; t < ticksFor(2.0f); ++t) robot.tick();
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
}

// Each tick that writes joints publishes the angles once, with the joints as written, within their travel, and every
// other joint as held.
void test_a_joint_write_in_idle_publishes_the_angles() {
    VirtualRobot robot;
    idleFromStand(robot);
    std::array<float, 18> expected = servoAnglesOf(robot);
    Published heard;
    auto listener = recordAngles(heard);
    robot.setJoint(4, 20.0f);
    robot.setJoint(13, -120.0f);
    robot.tick();
    expected[4] = 20.0f;
    expected[13] = -JOINT_LIMIT_DEG[1];
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected.data(), heard[0].data(), 18);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected.data(), robot.angles(), 18);
    for (int t = 0; t < ticksFor(1.0f); ++t) robot.tick();
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
}

// Centring publishes the centre pose in joint angles: the stand-in's factory calibration centres every coxa and femur
// at 0 and every tibia at -90, the servo's centre under the square knee.
void test_centring_in_idle_publishes_the_centre_pose() {
    VirtualRobot robot;
    idleFromStand(robot);
    Published heard;
    auto listener = recordAngles(heard);
    TEST_ASSERT_EQUAL_INT(200, robot.centreServos());
    robot.tick();
    std::array<float, 18> centre {};
    for (int leg = 0; leg < 6; ++leg) centre[leg * 3 + 2] = -90.0f;
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(centre.data(), heard[0].data(), 18);
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(centre.data(), robot.servoAngles(), 18);
    for (int t = 0; t < ticksFor(1.0f); ++t) robot.tick();
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
}

// The angles as the app receives them (servo_angles_wire.h), each publication in order.
EventBus<ServoAnglesMsg>::Handle recordWire(std::vector<socket_message_AnglesData> &heard) {
    return EventBus<ServoAnglesMsg>::subscribe([&heard](const ServoAnglesMsg &a) { heard.push_back(anglesData(a)); });
}

// A servo no pulse has driven since the boards woke holds no angle anyone knows. IDLE entered from boot or after a
// deactivation publishes every servo limp, which reaches the app as no angles at all rather than zeros; a joint written
// is reported at its angle from then on, the servos still limp marked so and given none, and the centring drives all.
// Entered from a stand, IDLE has no limp servo.
void test_idle_reports_its_limp_servos_as_limp() {
    for (const bool stoodFirst : {false, true}) {
        VirtualRobot robot;
        if (stoodFirst) {
            standUp(robot);
            robot.setMode(MOTION_STATE::DEACTIVATED);
            robot.tick();
        }
        std::vector<socket_message_AnglesData> heard;
        auto listener = recordWire(heard);
        robot.setMode(MOTION_STATE::IDLE);
        robot.tick();
        TEST_ASSERT_EQUAL_size_t(1, heard.size());
        TEST_ASSERT_EQUAL_size_t(0, heard[0].angles_count);

        robot.setJoint(4, 20.4f);
        robot.tick();
        TEST_ASSERT_EQUAL_size_t(2, heard.size());
        TEST_ASSERT_EQUAL_size_t(18, heard[1].angles_count);
        TEST_ASSERT_EQUAL_HEX32(ALL_SERVOS_LIMP & ~(1u << 4), heard[1].limp);
        TEST_ASSERT_EQUAL_INT32(20, heard[1].angles[4]);
        for (int servo = 0; servo < 18; ++servo)
            if (servo != 4) TEST_ASSERT_EQUAL_INT32(0, heard[1].angles[servo]);

        TEST_ASSERT_EQUAL_INT(200, robot.centreServos());
        robot.tick();
        TEST_ASSERT_EQUAL_size_t(3, heard.size());
        TEST_ASSERT_EQUAL_HEX32(0, heard[2].limp);
        TEST_ASSERT_EQUAL_INT32(-90, heard[2].angles[2]);
    }
    VirtualRobot robot;
    standUp(robot);
    std::vector<socket_message_AnglesData> heard;
    auto listener = recordWire(heard);
    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
    TEST_ASSERT_EQUAL_size_t(18, heard[0].angles_count);
    TEST_ASSERT_EQUAL_HEX32(0, heard[0].limp);
}

// The servo page's switch is applied by the control task at its next tick, as a mode is, never by the bus worker that
// delivers it, and published as the limp servos it leaves. Off in a stand, every servo is limp and stays so while the
// stand writes its pose to the sleeping boards; on again, every servo is limp until written. In IDLE it leaves every
// servo limp either way. It is taken before the modes that arrive with it, so a DEACTIVATED taken in the same tick has
// the last word.
void test_the_servo_pages_switch_is_applied_by_the_control_task() {
    VirtualRobot robot;
    standUp(robot);
    std::vector<socket_message_AnglesData> heard;
    auto listener = recordWire(heard);
    robot.servoSwitch(false);
    host_bus::drainAll();
    TEST_ASSERT_TRUE(robot.servosAwake());
    robot.tick();
    TEST_ASSERT_FALSE(robot.servosAwake());
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
    TEST_ASSERT_EQUAL_size_t(0, heard[0].angles_count);
    for (int t = 0; t < ticksFor(0.5f); ++t) robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
    TEST_ASSERT_EQUAL_HEX32(0, robot.drivenServos());
    robot.servoSwitch(true);
    robot.tick();
    TEST_ASSERT_TRUE(robot.servosDriven());

    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    robot.setJoint(4, 20.0f);
    robot.tick();
    heard.clear();
    robot.servoSwitch(false);
    robot.tick();
    robot.servoSwitch(true);
    robot.tick();
    TEST_ASSERT_TRUE(robot.servosAwake());
    TEST_ASSERT_EQUAL_HEX32(0, robot.drivenServos());
    TEST_ASSERT_EQUAL_size_t(2, heard.size());
    TEST_ASSERT_EQUAL_size_t(0, heard[1].angles_count);

    robot.setMode(MOTION_STATE::DEACTIVATED);
    robot.servoSwitch(true);
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, robot.mode());
    TEST_ASSERT_FALSE(robot.servosAwake());
}

// Deactivated, every servo is limp, so the angles a client subscribing then is answered with are none, rather than the
// pose the robot held before it sagged.
void test_deactivating_publishes_every_servo_limp() {
    VirtualRobot robot;
    standUp(robot);
    std::vector<socket_message_AnglesData> heard;
    auto listener = recordWire(heard);
    robot.setMode(MOTION_STATE::DEACTIVATED);
    robot.tick();
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
    TEST_ASSERT_EQUAL_size_t(0, heard[0].angles_count);
    ServoAnglesMsg latest;
    TEST_ASSERT_TRUE(EventBus<ServoAnglesMsg>::peek(latest));
    TEST_ASSERT_EQUAL_size_t(0, anglesData(latest).angles_count);
    for (int t = 0; t < ticksFor(1.0f); ++t) robot.tick();
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
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

// The telemetry says on which tick the dead-man stop fired: its flag rises on the tick the walk's stroke target drops,
// 600 ms after the last command and not a tick before, stays up while the link is silent, and falls with the next
// command.
void test_the_telemetry_flags_the_dead_man_stop_from_the_tick_it_fires() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    const int lastCommand = ticksFor(1.0f);
    int fired = -1;
    for (int t = 0; t < ticksFor(3.0f); ++t) {
        if (t <= lastCommand && t % COMMAND_PERIOD_TICKS == 0) robot.command(FORWARD);
        robot.tick();
        const bool stroked = robot.gaitCommand().step_y > 0.f;
        if (fired < 0 && robot.linkLost()) fired = t;
        if (fired < 0) {
            if (t > COMMAND_PERIOD_TICKS) TEST_ASSERT_TRUE_MESSAGE(stroked, "the walk stopped before the flag rose");
        } else {
            TEST_ASSERT_TRUE_MESSAGE(robot.linkLost(), "the flag fell without a command");
            TEST_ASSERT_FALSE_MESSAGE(stroked, "the flag rose before the stop");
        }
    }
    const int lastHeard = lastCommand - lastCommand % COMMAND_PERIOD_TICKS;
    TEST_ASSERT_INT_WITHIN(1, lastHeard + ticksFor(0.6f), fired);
    robot.command(FORWARD);
    robot.tick();
    TEST_ASSERT_FALSE(robot.linkLost());
}

// The dead-man's window runs from the command's arrival, not from the tick that takes it: a command delivered and then
// left in the inbox for 100 ms, as an overrun of the tick in progress leaves it, still stops the walk 600 ms after it
// arrived.
void test_the_dead_man_window_runs_from_the_commands_arrival_not_its_take() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    largestStep(robot, ticksFor(0.5f), &FORWARD);
    robot.command(FORWARD);
    host_bus::drainAll();
    constexpr int64_t HELD_US = 100000;
    host_clock::advance(HELD_US);
    robot.tick();
    TEST_ASSERT_FALSE(robot.linkLost());
    int ticks = 0;
    while (!robot.linkLost() && ticks < ticksFor(2.0f)) {
        robot.tick();
        ++ticks;
    }
    const int sinceArrival = (HELD_US + VirtualRobot::TICK_US * (ticks + 1)) / VirtualRobot::TICK_US;
    TEST_ASSERT_EQUAL_INT(ticksFor(0.6f), sinceArrival);
}

// A walk on the handheld's stick at its 50 Hz, the app's heartbeat `heartbeat` arriving between its frames when given:
// per tick, as walkWithHeartbeatUntil samples it.
std::vector<MotionSample> handheldWalk(const CommandMsg *heartbeat, float seconds) {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    std::vector<MotionSample> trace;
    for (int t = 0; t < ticksFor(seconds); ++t) {
        if (t % HANDHELD_PERIOD_TICKS == 0) robot.handheld(handheldFrame(0, 0.8f));
        if (heartbeat && t % APP_HEARTBEAT_TICKS == 2) robot.command(*heartbeat);
        robot.tick();
        MotionSample sample;
        std::copy_n(robot.angles(), 18, sample.begin());
        sample[18] = robot.bodyCommand().zm;
        sample[19] = robot.gaitCommand().step_y;
        trace.push_back(sample);
    }
    return trace;
}

// The handheld has priority (MotionInbox::HANDHELD_PRIORITY_MS): the app re-sends its last command every 200 ms, its
// sticks centred and its sliders' posture while untouched, and while the handheld walks the robot those heartbeats
// change no joint angle, no stroke and no body height, where each one used to stop the walk for a tick and move the
// posture.
void test_the_apps_heartbeat_leaves_a_handheld_walk_alone() {
    const CommandMsg heartbeat {0, 0, 0, 0, 0.3f, 0, 0, 0, 0};
    const std::vector<MotionSample> alone = handheldWalk(nullptr, 3.0f);
    const std::vector<MotionSample> heard = handheldWalk(&heartbeat, 3.0f);
    TEST_ASSERT_GREATER_THAN_FLOAT(GAIT_STRIDE_DEADBAND_MM, alone.back()[19]);
    for (size_t t = 0; t < alone.size(); ++t) {
        char msg[48];
        std::snprintf(msg, sizeof(msg), "tick %u", (unsigned)t);
        TEST_ASSERT_EQUAL_FLOAT_ARRAY_MESSAGE(alone[t].data(), heard[t].data(), 20, msg);
    }
}

// Once the handheld lets go, its centred frame stops it at once, and the app is heard again 600 ms after the
// handheld's last frame with a stick off centre: the app's heartbeat carries another height, which the stand takes
// from then on and not a tick before.
void test_the_app_is_heard_again_600_ms_after_the_handheld_lets_go() {
    VirtualRobot robot;
    standUp(robot);
    const CommandMsg heartbeat {0, 0, 0, 0, 0.5f, 0, 0, 0, 0};
    const float standingZm = robot.bodyCommand().zm;
    const int release = ticksFor(1.0f);
    const int lastMoving = release - HANDHELD_PERIOD_TICKS;
    int heightTaken = -1;
    for (int t = 0; t < ticksFor(2.5f); ++t) {
        if (t < release && t % HANDHELD_PERIOD_TICKS == 0) robot.handheld(handheldFrame(0.5f, 0));
        if (t == release) robot.handheld(handheldFrame(0, 0));
        if (t % APP_HEARTBEAT_TICKS == 3) robot.command(heartbeat);
        robot.tick();
        if (t < release) TEST_ASSERT_LESS_THAN_FLOAT(0, robot.bodyCommand().xm); // lx > 0 shifts the body right
        if (t == release) TEST_ASSERT_EQUAL_FLOAT(0, robot.bodyCommand().xm);
        if (heightTaken < 0 && robot.bodyCommand().zm != standingZm) heightTaken = t;
    }
    int firstHeard = lastMoving + ticksFor(0.6f);
    while (firstHeard % APP_HEARTBEAT_TICKS != 3) ++firstHeard;
    TEST_ASSERT_EQUAL_INT(firstHeard, heightTaken);
    TEST_ASSERT_EQUAL_FLOAT(bodyHeightMm(0.5f), robot.bodyCommand().zm);
}

// A handheld whose link drops mid-walk sends no stop: the dead-man stops the walk 600 ms after its last frame, and the
// telemetry reports the link lost, though the app's heartbeat goes on arriving. A dropped heartbeat is no
// command to the dead-man, so it neither keeps the handheld's last stick walking nor hides the loss.
void test_the_dead_man_stops_a_handheld_walk_whose_link_drops_under_the_apps_heartbeat() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    const CommandMsg heartbeat {0, 0, 0, 0, 0, 0, 0, 0, 0};
    const int lastFrame = ticksFor(1.0f);
    int fired = -1;
    for (int t = 0; t <= lastFrame + ticksFor(1.0f); ++t) {
        if (t <= lastFrame && t % HANDHELD_PERIOD_TICKS == 0) robot.handheld(handheldFrame(0, 0.8f));
        if (t > lastFrame && (t - lastFrame) % APP_HEARTBEAT_TICKS == 10) robot.command(heartbeat);
        robot.tick();
        if (fired < 0 && robot.linkLost()) fired = t;
        if (t > HANDHELD_PERIOD_TICKS && fired < 0)
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0, robot.gaitCommand().step_y,
                                                   "the walk stopped before the dead-man");
    }
    TEST_ASSERT_EQUAL_INT(lastFrame + ticksFor(0.6f), fired);
    TEST_ASSERT_EQUAL_FLOAT(0, robot.gaitCommand().step_y);
}

// A script walking the robot when the handheld takes over keeps re-sending its stick, as the facade's keep-alive does
// every 200 ms: the robot stays stopped after the handheld lets go, and walks the script's stick again only after the
// script has sent one centred command.
void test_a_stale_stick_stream_does_not_resume_the_walk_after_the_handheld() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    const CommandMsg centred {0, 0, 0, 0, 0, 0, 0, 0, 0};
    const int takeover = ticksFor(1.0f), release = ticksFor(2.0f), centredAt = ticksFor(4.0f);
    for (int t = 0; t < ticksFor(5.0f); ++t) {
        if (t >= takeover && t < release && t % HANDHELD_PERIOD_TICKS == 0) robot.handheld(handheldFrame(0, 0, 0.6f));
        if (t == release) robot.handheld(handheldFrame(0, 0));
        if (t % APP_HEARTBEAT_TICKS == 2) robot.command(t == centredAt + 2 ? centred : FORWARD);
        robot.tick();
        if (t == takeover - 1)
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(GAIT_STRIDE_DEADBAND_MM, robot.gaitCommand().step_y,
                                                   "the script did not walk the robot");
        if (t > release && t <= centredAt + 2)
            TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0, robot.gaitCommand().step_y, "the stale stick walked again");
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(GAIT_STRIDE_DEADBAND_MM, robot.gaitCommand().step_y);
}

// The robot reports whether the handheld drives with its mode (ModeData.handheld): on each change of it the control
// task reports the mode applied again, and main.cpp's bridge reads the flag as it pushes the report. While the flag
// holds, nothing more is reported.
void test_the_handheld_driving_is_reported_with_the_mode_on_each_change() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    robot.tick();
    struct Report {
        int tick;
        MOTION_STATE mode;
        bool handheld;
    };
    std::vector<Report> reports;
    int t = 0;
    auto listener = EventBus<ModeMsg>::subscribe([&](const ModeMsg &m) {
        if (m.kind == ModeMsgKind::APPLIED) reports.push_back({t, m.mode, robot.handheldDriving()});
    });
    const int release = ticksFor(1.0f);
    for (; t < ticksFor(2.5f); ++t) {
        if (t < release && t % HANDHELD_PERIOD_TICKS == 0) robot.handheld(handheldFrame(0, 0.5f));
        if (t == release) robot.handheld(handheldFrame(0, 0));
        robot.tick();
    }
    TEST_ASSERT_EQUAL_size_t(2, reports.size());
    TEST_ASSERT_EQUAL_INT(0, reports[0].tick);
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, reports[0].mode);
    TEST_ASSERT_TRUE(reports[0].handheld);
    TEST_ASSERT_EQUAL_INT(release - HANDHELD_PERIOD_TICKS + ticksFor(0.6f), reports[1].tick);
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, reports[1].mode);
    TEST_ASSERT_FALSE(reports[1].handheld);
}

// The handheld's height trim in STAND: the right stick pushed up halfway with its button held.
controller_packet_t handheldTrim() {
    controller_packet_t trim = handheldFrame(0, 0, 0, 0.5f);
    trim.buttons = BTN_RIGHT;
    return trim;
}

// The app stands the robot with its right stick held up for a tilt and then walks it at its defaults, re-sending each
// command every 200 ms. Before it, the handheld trimmed the stand's height and was put down switched on, sending its
// centred frames `idlePeriodTicks` apart between the app's; with 0 there is neither the trim nor the frames. How far
// the app walked, and how often the stand's target height and tilt moved from the app's first command until the walk.
struct AppDrive {
    float walkedM;
    int heightMoves, tiltMoves;
};

AppDrive appDriveAfterAHandheldTrim(int idlePeriodTicks) {
    VirtualRobot robot;
    standUp(robot);
    const CommandMsg stand {0, 0, 0, 0.8f, 0, 0, 0, 0, 0};
    const CommandMsg walk {0, 0.8f, 0, 0, 0, 0.5f, 0, 0, 0.3f};
    const int released = ticksFor(0.5f), appFrom = ticksFor(1.5f), walkFrom = ticksFor(3.5f), end = ticksFor(7.5f);
    AppDrive drive {0, 0, 0};
    BodyStateMsg last = robot.bodyCommand();
    float walkedFrom = 0;
    for (int t = 0; t < end; ++t) {
        if (idlePeriodTicks && t < released && t % HANDHELD_PERIOD_TICKS == 0) robot.handheld(handheldTrim());
        if (idlePeriodTicks && t >= released && (t - released) % idlePeriodTicks == idlePeriodTicks / 2)
            robot.handheld(handheldFrame(0, 0));
        if (t == walkFrom) robot.setMode(MOTION_STATE::WALK);
        if (t >= appFrom && (t - appFrom) % APP_HEARTBEAT_TICKS == 0) robot.command(t < walkFrom ? stand : walk);
        robot.tick();
        if (t == walkFrom) walkedFrom = robot.odometry().distance;
        const BodyStateMsg &body = robot.bodyCommand();
        if (t >= appFrom && t < walkFrom) {
            drive.heightMoves += body.zm != last.zm;
            drive.tiltMoves += body.omega != last.omega;
        }
        last = body;
    }
    drive.walkedM = robot.odometry().distance - walkedFrom;
    return drive;
}

// Roadmap 11.1b: an idle handheld's frame is dropped whole while a client's command is in force. So after a trim with
// the handheld left on, the app's posture is the stand's from its first command and its walk goes as far as with no
// handheld, where the frames' posture used to flip the stand's height and tilt at their beat and freeze the walk.
void test_an_idle_handheld_leaves_the_apps_posture_and_walk_alone_after_a_trim() {
    const AppDrive alone = appDriveAfterAHandheldTrim(0);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.5f, alone.walkedM);
    TEST_ASSERT_EQUAL_INT(0, alone.heightMoves);
    TEST_ASSERT_EQUAL_INT(1, alone.tiltMoves);
    for (const int period : {APP_HEARTBEAT_TICKS, HANDHELD_PERIOD_TICKS}) {
        const AppDrive beside = appDriveAfterAHandheldTrim(period);
        char msg[48];
        std::snprintf(msg, sizeof(msg), "handheld every %d ticks", period);
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, beside.heightMoves, msg);
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, beside.tiltMoves, msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02f * alone.walkedM, alone.walkedM, beside.walkedM, msg);
    }
}

// A handheld on its own keeps its trim: with no client in force its centred frames are its commands, so they hold the
// stand's height and keep the dead-man from firing for as long as the handheld is switched on.
void test_a_lone_idle_handheld_keeps_its_trim_with_no_dead_man_stop() {
    VirtualRobot robot;
    standUp(robot);
    const float neutral = robot.bodyCommand().zm;
    const int released = ticksFor(0.5f), settled = released + ticksFor(0.5f);
    float trimmed = 0;
    for (int t = 0; t < released + ticksFor(5.0f); ++t) {
        if (t < released && t % HANDHELD_PERIOD_TICKS == 0) robot.handheld(handheldTrim());
        if (t >= released && (t - released) % APP_HEARTBEAT_TICKS == 0) robot.handheld(handheldFrame(0, 0));
        robot.tick();
        TEST_ASSERT_FALSE_MESSAGE(robot.linkLost(), "the dead-man fired under a live handheld");
        if (t == settled) trimmed = robot.bodyCommand().zm;
        if (t > settled) TEST_ASSERT_EQUAL_FLOAT_MESSAGE(trimmed, robot.bodyCommand().zm, "the trim did not hold");
    }
    TEST_ASSERT_GREATER_THAN_FLOAT(neutral + 10, trimmed);
}

// An app whose link drops mid-walk beside an idle handheld: while the app's last command is in force the handheld's
// frames are dropped, so the dead-man stops the walk 600 ms after that command and reports the link lost; once the app
// has been silent for 2 s the frames are the handheld's commands again, which keep the robot stopped.
void test_the_dead_man_stops_a_lost_apps_walk_beside_an_idle_handheld() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    const int lastCommand = ticksFor(1.0f);
    int fired = -1;
    for (int t = 0; t <= lastCommand + ticksFor(3.0f); ++t) {
        if (t <= lastCommand && t % APP_HEARTBEAT_TICKS == 0) robot.command(FORWARD);
        if (t % APP_HEARTBEAT_TICKS == APP_HEARTBEAT_TICKS / 2) robot.handheld(handheldFrame(0, 0));
        robot.tick();
        if (fired < 0 && robot.linkLost()) fired = t;
        if (t > COMMAND_PERIOD_TICKS && fired < 0)
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0, robot.gaitCommand().step_y,
                                                   "the walk stopped before the dead-man");
        if (fired >= 0 && t < lastCommand + ticksFor(2.0f))
            TEST_ASSERT_TRUE_MESSAGE(robot.linkLost(), "an idle frame hid the lost link");
        if (fired >= 0) TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0, robot.gaitCommand().step_y, "the walk resumed");
    }
    TEST_ASSERT_EQUAL_INT(lastCommand + ticksFor(0.6f), fired);
    TEST_ASSERT_FALSE(robot.linkLost());
}

// The app walks the robot sideways, re-sending its stick every 200 ms, when the handheld takes over forward: the
// handheld's frames take priority though the app's command is in force, and its centred frame on release, within its
// window, stops the walk at once.
void test_the_handheld_takes_over_an_apps_walk_and_stops_it_on_release() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::WALK);
    const CommandMsg sideways {0.8f, 0, 0, 0, 0, 0, 0, 0, 0};
    const int takeover = ticksFor(2.0f), release = ticksFor(3.0f);
    for (int t = 0; t <= release; ++t) {
        if (t % APP_HEARTBEAT_TICKS == 0) robot.command(sideways);
        if (t >= takeover && t < release && t % HANDHELD_PERIOD_TICKS == 0) robot.handheld(handheldFrame(0, 0.8f));
        if (t == release) robot.handheld(handheldFrame(0, 0));
        robot.tick();
        const gait_state_t &target = robot.gaitCommand();
        if (t == takeover - 1)
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(GAIT_STRIDE_DEADBAND_MM, std::fabs(target.step_x),
                                                   "the app did not walk the robot");
        if (t >= takeover && t < release) {
            TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0, target.step_x, "the app's stick was walked");
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0, target.step_y, "the handheld's stick was not walked");
        }
    }
    TEST_ASSERT_EQUAL_FLOAT(0, robot.gaitCommand().step_x);
    TEST_ASSERT_EQUAL_FLOAT(0, robot.gaitCommand().step_y);
}

// The telemetry reports the write of its own tick: none in a mode that sends no pose, a success in a stand, and an idle
// tick after a stand tick does not inherit the stand's write (Hexapod::planMotion starts each tick's record).
void test_the_telemetry_reports_the_servo_write_of_its_own_tick() {
    VirtualRobot robot;
    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_NONE, robot.servoWrite());
    robot.setMode(MOTION_STATE::STAND);
    robot.tick();
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_OK, robot.servoWrite());
    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::IDLE, robot.mode());
    TEST_ASSERT_EQUAL(socket_message_ServoWrite_SERVO_WRITE_NONE, robot.servoWrite());
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

// Only the control task changes the motion (roadmap 9.5): what a transport posts reaches its bus's worker at once
// (host_bus::drainAll), and the motion takes it at the start of the next tick, never in the middle of one.
void test_nothing_posted_changes_the_motion_before_the_next_tick() {
    {
        VirtualRobot robot;
        standUp(robot);
        robot.command({1, 0, 0, 0, 0, 0, 0, 0});
        host_bus::drainAll();
        TEST_ASSERT_EQUAL_FLOAT(0.f, robot.bodyCommand().xm);
        robot.tick();
        TEST_ASSERT_EQUAL_FLOAT(-50.f, robot.bodyCommand().xm);
    }
    {
        VirtualRobot robot;
        standUp(robot);
        robot.setMode(MOTION_STATE::WALK);
        robot.tick();
        robot.command(FORWARD);
        host_bus::drainAll();
        TEST_ASSERT_EQUAL_FLOAT(0.f, robot.gaitCommand().step_y);
        robot.tick();
        TEST_ASSERT_GREATER_THAN_FLOAT(GAIT_STRIDE_DEADBAND_MM, robot.gaitCommand().step_y);
    }
    {
        VirtualRobot robot;
        standUp(robot);
        robot.setGait(GaitType::BI_GATE);
        host_bus::drainAll();
        TEST_ASSERT_EQUAL(GaitType::TRI_GATE, robot.gaitCommand().gait_type);
        robot.tick();
        TEST_ASSERT_EQUAL(GaitType::BI_GATE, robot.gaitCommand().gait_type);
    }
    {
        VirtualRobot robot;
        standUp(robot);
        robot.setMode(MOTION_STATE::WALK);
        largestStep(robot, ticksFor(0.5f), &FORWARD);
        const float stroke = robot.gaitCommand().step_y;
        TEST_ASSERT_GREATER_THAN_FLOAT(GAIT_STRIDE_DEADBAND_MM, stroke);
        robot.setMode(MOTION_STATE::STAND);
        host_bus::drainAll();
        TEST_ASSERT_EQUAL_FLOAT(stroke, robot.gaitCommand().step_y);
        robot.tick();
        TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
        TEST_ASSERT_EQUAL_FLOAT(0.f, robot.gaitCommand().step_y);
        robot.setMode(MOTION_STATE::DEACTIVATED);
        host_bus::drainAll();
        TEST_ASSERT_TRUE(robot.servosDriven());
        robot.tick();
        TEST_ASSERT_FALSE(robot.servosDriven());
    }
}

// Modes are taken in the order they arrived, none merged into another: a borrow arriving in the tick a DEACTIVATED
// did is decided from DEACTIVATED and refused, so the boards stay asleep.
void test_a_deactivation_and_a_borrow_in_one_tick_leave_the_robot_deactivated() {
    VirtualRobot robot;
    standUp(robot);
    EventBus<ModeMsg>::publish({MOTION_STATE::DEACTIVATED});
    EventBus<ModeMsg>::publish({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW});
    robot.tick();
    robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, robot.mode());
    TEST_ASSERT_FALSE(robot.servosDriven());
    TEST_ASSERT_FALSE(ServoController::output().awake);
}

// A borrow with nothing to play finds the runner idle at stance, so the control task hands the mode back on its own,
// and the observers learn both outcomes as APPLIED.
void test_a_borrow_with_nothing_to_play_hands_the_mode_back() {
    VirtualRobot robot;
    standUp(robot);
    std::vector<ModeMsg> applied;
    auto listener = EventBus<ModeMsg>::subscribe([&applied](const ModeMsg &m) {
        if (m.kind == ModeMsgKind::APPLIED) applied.push_back(m);
    });
    EventBus<ModeMsg>::publish({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW});
    for (int t = 0; t < 3; ++t) robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
    host_bus::drainAll();
    TEST_ASSERT_EQUAL(2, applied.size());
    TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, applied[0].mode);
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, applied[1].mode);
}

namespace {

AnimationCommandMsg playOf(const char *name) {
    AnimationCommandMsg play {};
    std::snprintf(play.name, sizeof(play.name), "%s", name);
    return play;
}

} // namespace

// A clip that holds until stopped, played from a stand, is the animation these tests judge a stop and a pose by.
void test_a_play_from_a_stand_holds_in_animate_until_stopped() {
    VirtualRobot robot;
    standUp(robot);
    robot.animationPlay(playOf("fx_single"));
    largestStep(robot, ticksFor(1.5f));
    TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, robot.mode());
    robot.animationStop();
    largestStep(robot, ticksFor(1.5f));
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
}

// A stop that arrives while the play's borrow is queued in the inbox, within the tick the play came in, is the play's
// stop: the clip never plays and the mode is handed back, where the stop dropped would leave the clip holding.
void test_a_stop_in_the_tick_of_the_play_ends_it() {
    VirtualRobot robot;
    standUp(robot);
    robot.animationPlay(playOf("fx_single"));
    host_bus::drainAll(); // the borrow reaches the inbox, which the control task takes at its next tick
    robot.animationStop();
    largestStep(robot, ticksFor(1.5f));
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
}

// A pose that arrives while a borrow is queued is the puppet's first pose: it holds ANIMATE while the body is away from
// the stance, where the pose dropped leaves a borrow with nothing to do, handed back within three ticks.
void test_a_pose_in_the_tick_of_the_borrow_is_taken() {
    VirtualRobot robot;
    standUp(robot);
    EventBus<ModeMsg>::publish({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW});
    host_bus::drainAll();
    PoseMsg pose {};
    pose.body[5] = 20.0f;
    robot.animationPose(pose);
    for (int t = 0; t < 10; ++t) robot.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, robot.mode());
}

// The joint angles are the control task's alone: the angles the robot publishes for the app are never written back.
void test_angles_on_the_bus_leave_the_motions_angles_alone() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::DEACTIVATED);
    robot.tick();
    float held[18];
    std::copy_n(robot.angles(), 18, held);
    ServoAnglesMsg stale;
    std::fill_n(stale.angles, 18, 99.0f);
    EventBus<ServoAnglesMsg>::publish(stale);
    robot.tick();
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(held, robot.angles(), 18);
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

// lx, ly, rx at s = +1, the fastest the stick can ask for, at the speed-optimal stride unless given another.
CommandMsg fullStick(float lx, float ly, float rx, float stride = 0) { return {lx, ly, rx, 0, 0, 1, 0, 0, stride}; }

const CommandMsg FULL_STICKS[] = {fullStick(0, 1, 0), fullStick(0, -1, 0), fullStick(1, 0, 0), fullStick(-1, 0, 0),
                                  fullStick(0, 0, 1), fullStick(0, 0, -1), fullStick(1, 1, 1), fullStick(-1, -1, -1)};

// A pose beyond a joint's travel, which the searched gait asks for, is held within it by the controller, so IDLE
// entered from it publishes the angles within travel, as the servos hold them, not the motion's command.
void test_idle_entered_from_a_pose_beyond_travel_publishes_the_held_angles() {
    VirtualRobot robot;
    walkIn(robot, GaitType::TUNED);
    const CommandMsg stick = fullStick(1, 1, 1);
    bool beyond = false;
    for (int t = 0; t < ticksFor(5.0f) && !beyond; ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(stick);
        robot.tick();
        for (int j = 0; j < 18; ++j) beyond |= withinTravel(robot.angles()[j], j % 3) != robot.angles()[j];
    }
    TEST_ASSERT_TRUE_MESSAGE(beyond, "the walk never asked for a joint beyond its travel");
    const std::array<float, 18> held = servoAnglesOf(robot);
    Published heard;
    auto listener = recordAngles(heard);
    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    TEST_ASSERT_EQUAL_size_t(1, heard.size());
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(held.data(), heard[0].data(), 18);
}

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
        const AutoDemand demand = autoDemand(0, 1, 0, k.s, 0, 0, 0, 0);
        const WalkTarget left = autoWalkTarget(GaitType::RIPPLE, demand, 0, 1, 0, 0, 0, 0, 0);
        const WalkTarget expected = autoWalkTarget(k.selected, demand, 0, 1, 0, 0, 0, 0, 0);
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
// switch to TUNED here lands in the ramp window right after a stick at the shortest stride, whose first ramp tick stays
// inside the stride deadband, and the stick is released before TRI.
void test_a_classic_gait_loaded_after_tuned_walks_no_stale_stick() {
    VirtualRobot robot;
    walkIn(robot, GaitType::TRI_GATE);
    settleWalk(robot);
    robot.command({0, 0.3f, 0, 0, 0, 0, 0, 0, -1});
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
                const auto owner = std::make_unique<VirtualRobot>(); // heap: the stack would exceed -Wstack-usage
                VirtualRobot &robot = *owner;
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
    TEST_ASSERT_EQUAL_FLOAT(classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, 0, 0, 0, 0).stepY,
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
    const WalkTarget expected = classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, s1, h, fd, 0);
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
// a zero stroke, the live one as well as the commanded one, and takes its posture again before it steps, even within
// the dead-man's 600 ms. The classic stroke left live used to be ramped out from wherever the policy left the feet,
// which took a coxa past its travel after a walk of 1950 or 2300 ms but not 2000 ms, so the return is checked at
// walks ending in every part of the cycle.
void test_a_walk_after_walk_nn_takes_its_posture_again() {
    for (int walkMs = 1500; walkMs <= 2600; walkMs += 50) {
        VirtualRobot robot;
        walkIn(robot, GaitType::TRI_GATE);
        const CommandMsg stick = fullStick(0, 1, 0);
        hold(robot, stick, ticksFor(walkMs / 1000.0f));
        TEST_ASSERT_TRUE(walkResumed(robot));

        robot.setMode(MOTION_STATE::WALK_NN);
        largestStep(robot, ticksFor(0.4f));
        robot.setMode(MOTION_STATE::WALK);
        robot.tick();
        char msg[48];
        std::snprintf(msg, sizeof(msg), "after a %d ms walk", walkMs);
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, robot.gaitCommand().step_y, msg);
        TEST_ASSERT_FALSE_MESSAGE(gaitIsCommanded(robot.gaitState()), msg);
        robot.command(stick);
        robot.tick();
        TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, robot.gaitCommand().step_y, msg);
        const HeldRun run = hold(robot, stick, ticksFor(3.0f));
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
        TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
    }
}

// What WALK_NN reads of a command, which the app's controls in that mode follow (app/src/lib/mode-controls.ts): the
// policy walks the sticks' lx, ly and rx alone. It walks the stance it was trained on and rides at its own height, lift,
// stride and cadence, so the feet distance, height, speed, step height, stride and the right stick's ry change nothing.
void test_walk_nn_reads_the_sticks_alone() {
    const CommandMsg walking {0.3f, 0.6f, 0.2f, 0, 0, 0, 0, 0, 0};
    const auto base = traceHeld(MOTION_STATE::WALK_NN, walking, ticksFor(2.0f));
    struct Field {
        const char *name;
        float CommandMsg::*field;
        bool read;
    };
    const Field fields[] = {
        {"lx", &CommandMsg::lx, true},  {"ly", &CommandMsg::ly, true}, {"rx", &CommandMsg::rx, true},
        {"ry", &CommandMsg::ry, false}, {"h", &CommandMsg::h, false},  {"s", &CommandMsg::s, false},
        {"s1", &CommandMsg::s1, false}, {"fd", &CommandMsg::fd, false}, {"stride", &CommandMsg::stride, false}};
    for (const Field &f : fields) {
        CommandMsg changed = walking;
        changed.*f.field = -0.8f;
        const bool differs = traceHeld(MOTION_STATE::WALK_NN, changed, ticksFor(2.0f)) != base;
        TEST_ASSERT_EQUAL_MESSAGE(f.read, differs, f.name);
    }
}

// The middle right foot's mean distance out from the body over a WALK_NN walk of `ticks`, entered from a stand at
// the feet distance `fd`, the walk's commands sending it too.
float walkNnStanceMm(VirtualRobot &robot, float fd, int ticks) {
    CommandMsg walking {0.3f, 0.6f, 0.2f, 0, 0, 0, 0, fd, 0};
    standUp(robot);
    hold(robot, {0, 0, 0, 0, 0, 0, 0, fd, 0}, ticksFor(3.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, DEFAULT_STANCE[1][0] * feetScale(fd), robot.bodyState().feet[1][0]);
    robot.setMode(MOTION_STATE::WALK_NN);
    float sum = 0.0f;
    for (int t = 0; t < ticks; ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(walking);
        robot.tick();
        sum += robot.bodyState().feet[1][0];
    }
    return sum / ticks;
}

// The policy was trained on the stance of fd 0 alone (firmware/host/policy_api.cpp), so WALK_NN entered from a stand
// at a wider stance resets on that stance and walks it as from a stand at fd 0, a quarter narrower. Leaving it, the
// stand steps the feet back to the stance the last feet distance set, with no command to ask for it.
void test_walk_nn_walks_the_stance_it_was_trained_on() {
    const float wideFd = 1.0f;
    VirtualRobot trainedRobot;
    const float trained = walkNnStanceMm(trainedRobot, 0.0f, ticksFor(2.0f));
    VirtualRobot robot;
    const float wide = walkNnStanceMm(robot, wideFd, ticksFor(2.0f));
    char msg[80];
    std::snprintf(msg, sizeof(msg), "middle foot out %.1f mm from fd 0, %.1f mm from fd 1", trained, wide);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(2.0f, trained, wide, msg);

    // The sticks centred, so the dead-man's 2 s, not its 600 ms while steering, runs from the last command.
    robot.command({0, 0, 0, 0, 0, 0, 0, wideFd, 0});
    robot.tick();
    robot.setMode(MOTION_STATE::STAND);
    for (int t = 0; t < ticksFor(1.8f); ++t) robot.tick();
    TEST_ASSERT_FALSE(robot.linkLost());
    TEST_ASSERT_FLOAT_WITHIN(0.5f, DEFAULT_STANCE[1][0] * feetScale(wideFd), robot.bodyState().feet[1][0]);
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
    const WalkTarget expected = classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, s1, h, fd, 0);
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
            classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, posture.s1, posture.h, posture.fd, 0);
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

// The steady walks below peak at 3.95 deg in a tick (measured), just past the servo's 3.75 deg (13.1 rad/s over 5 ms),
// and their stops at 3.96 deg.
constexpr float STEP_BOUND_DEG = 4.0f;

struct StopRun {
    float walkPeak = 0.0f; // the largest joint step in a tick of the steady walk
    HeldRun stop;          // the stop, from its first tick until the feet are home
    bool cadenceFollowsStroke = true; // the cadence scaled with the share of the stroke left
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
            run.cadenceFollowsStroke &=
                std::fabs(live.phase_rate * stroke(walked) - walked.phase_rate * stroke(live)) <=
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
// 4.5 deg. The stop ramp eases the stroke out in two stages, from rest, and scales the cadence with the stroke left.
// Every way a classic walk stops is checked, at a slow and a fast stick, in postures from neutral to the envelope's
// edges, a release at eight points of the cycle, and no stop may move a joint faster than STEP_BOUND_DEG, leave travel,
// let its cadence part from the stroke left, or leave a cadence set once the gait has stopped.
void test_every_stop_eases_the_stroke_out_with_the_cadence() {
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
                        TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(STEP_BOUND_DEG, run.stop.largestStep, msg);
                        TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.stop.clamped, msg);
                        TEST_ASSERT_TRUE_MESSAGE(run.cadenceFollowsStroke, msg);
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

// What the servos receive while a mode asked for from IDLE takes over: the largest change of a servo's angle in a tick,
// from the angles it held in IDLE, over `ticks`, and the mode reported each tick.
struct Takeover {
    float largestStep = 0.0f;
    std::vector<MOTION_STATE> modes;
};

// What the servos receive over `ticks` from the angles they hold now.
Takeover follow(VirtualRobot &robot, int ticks) {
    Takeover run;
    std::array<float, 18> previous = servoAnglesOf(robot);
    for (int t = 0; t < ticks; ++t) {
        robot.tick();
        run.modes.push_back(robot.mode());
        for (int j = 0; j < 18; ++j)
            run.largestStep = std::max(run.largestStep, std::fabs(robot.servoAngles()[j] - previous[j]));
        previous = servoAnglesOf(robot);
    }
    return run;
}

Takeover takeOver(VirtualRobot &robot, MOTION_STATE mode, int ticks) {
    robot.setMode(mode);
    return follow(robot, ticks);
}

// The ease out of IDLE turns a joint at most half the servo's top speed in a tick, 13.1 rad/s over the 5 ms tick
// halved, so a leg folded far from the stand swings home at a pace the servo follows with torque to spare; the puppet
// path otherwise turns one at the servo's top speed.
constexpr float IDLE_EASE_STEP_DEG = 1.875f;
constexpr float PUPPET_STEP_DEG = 3.75f;
constexpr float CAP_TOLERANCE_DEG = 1e-3f;

// The joints dragged far from the stand: a coxa at its travel, a femur raised square and a tibia folded back.
void dragAway(VirtualRobot &robot) {
    robot.setJoint(0, JOINT_LIMIT_DEG[0]);
    robot.setJoint(1, JOINT_LIMIT_DEG[1]);
    robot.setJoint(5, JOINT_LIMIT_DEG[2]);
    robot.tick();
}

} // namespace

// Leaving an IDLE whose joints were dragged for STAND or WALK eases every joint from the angle it holds to the stand,
// through ANIMATE's puppet path as leaving an animation does, at half the servo's top speed, and then hands back to the
// mode asked for, standing as a robot stood up from rest stands.
void test_leaving_idle_after_drags_eases_the_joints_to_the_stand() {
    VirtualRobot stood;
    standUp(stood);
    const std::array<float, 18> stand = servoAnglesOf(stood);
    for (const MOTION_STATE next : {MOTION_STATE::STAND, MOTION_STATE::WALK}) {
        VirtualRobot robot;
        idleFromStand(robot);
        dragAway(robot);
        const Takeover run = takeOver(robot, next, ticksFor(2.0f));
        char msg[64];
        std::snprintf(msg, sizeof(msg), "to mode %d: largest step %.2f deg", (int)next, run.largestStep);
        TEST_MESSAGE(msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(CAP_TOLERANCE_DEG, IDLE_EASE_STEP_DEG, run.largestStep, msg);
        TEST_ASSERT_EQUAL_MESSAGE(MOTION_STATE::ANIMATE, run.modes.front(), msg);
        TEST_ASSERT_EQUAL_MESSAGE(next, run.modes.back(), msg);
        for (int j = 0; j < 18; ++j) TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, stand[j], robot.servoAngles()[j], msg);
    }
}

// From limp servos, a mode stands the robot up as it does from DEACTIVATED: at once, with no ease, since a limp servo
// holds no angle to ease from. Dragged ones among them still ease; the limp ones are written at the stand at once.
void test_leaving_limp_idle_stands_up_as_from_deactivated() {
    VirtualRobot robot;
    robot.setMode(MOTION_STATE::IDLE);
    robot.tick();
    Takeover run = takeOver(robot, MOTION_STATE::STAND, 1);
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, run.modes.front());
    TEST_ASSERT_TRUE(robot.servosDriven());

    VirtualRobot dragged;
    dragged.setMode(MOTION_STATE::IDLE);
    dragged.tick();
    dragged.setJoint(1, JOINT_LIMIT_DEG[1]);
    dragged.tick();
    run = takeOver(dragged, MOTION_STATE::STAND, 1);
    TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, run.modes.front());
    TEST_ASSERT_TRUE(dragged.servosDriven());
    TEST_ASSERT_FLOAT_WITHIN(STEP_BOUND_DEG, JOINT_LIMIT_DEG[1], dragged.servoAngles()[1]);
    const std::array<float, 18> first = servoAnglesOf(dragged);
    for (int t = 0; t < ticksFor(2.0f); ++t) dragged.tick();
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, dragged.mode());
    for (int j = 0; j < 18; ++j)
        if (j / 3 != 0) TEST_ASSERT_FLOAT_WITHIN(0.5f, dragged.servoAngles()[j], first[j]);
}

// IDLE taken during the ease out of IDLE, as the handheld's next press or the app's IDLE does, holds the joints where
// the ease left them, a pose no motion set, so a STAND after it eases them again from there instead of snapping them to
// the stand: the tibia it left folded would otherwise swing up to 240 deg in a tick.
void test_idle_taken_during_the_ease_out_of_idle_holds_the_pose_the_ease_left() {
    for (const int eased : {3, 10, 30}) {
        VirtualRobot robot;
        idleFromStand(robot);
        dragAway(robot);
        TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, takeOver(robot, MOTION_STATE::STAND, eased).modes.back());
        Takeover run = takeOver(robot, MOTION_STATE::IDLE, 2);
        TEST_ASSERT_EQUAL(MOTION_STATE::IDLE, run.modes.back());
        TEST_ASSERT_EQUAL_FLOAT(0.0f, run.largestStep);
        run = takeOver(robot, MOTION_STATE::STAND, ticksFor(2.0f));
        char msg[80];
        std::snprintf(msg, sizeof(msg), "IDLE after %d ticks of the ease: largest step %.2f deg", eased,
                      run.largestStep);
        TEST_MESSAGE(msg);
        TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(IDLE_EASE_STEP_DEG + CAP_TOLERANCE_DEG, run.largestStep, msg);
        TEST_ASSERT_EQUAL_MESSAGE(MOTION_STATE::ANIMATE, run.modes.front(), msg);
        TEST_ASSERT_EQUAL_MESSAGE(MOTION_STATE::STAND, run.modes.back(), msg);
    }
}

// Two modes out of an IDLE whose joints were dragged, taken in one tick, as a double press or two clients send them:
// the first borrows ANIMATE for the ease, and the second, finding the ease begun, becomes the mode it hands back to
// rather than applying at once.
void test_a_second_mode_in_the_tick_of_the_ease_out_of_idle_waits_for_it() {
    for (const MOTION_STATE second : {MOTION_STATE::STAND, MOTION_STATE::WALK}) {
        VirtualRobot robot;
        idleFromStand(robot);
        dragAway(robot);
        robot.setMode(MOTION_STATE::STAND);
        const Takeover run = takeOver(robot, second, ticksFor(2.0f));
        char msg[80];
        std::snprintf(msg, sizeof(msg), "STAND then mode %d: largest step %.2f deg", (int)second, run.largestStep);
        TEST_MESSAGE(msg);
        TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(IDLE_EASE_STEP_DEG + CAP_TOLERANCE_DEG, run.largestStep, msg);
        TEST_ASSERT_EQUAL_MESSAGE(MOTION_STATE::ANIMATE, run.modes.front(), msg);
        TEST_ASSERT_EQUAL_MESSAGE(second, run.modes.back(), msg);
    }
}

// Puppeteering keeps the servo's top speed whatever ANIMATE was entered for: a pose sent during the ease out of IDLE,
// and after it in an ANIMATE asked for, is approached at 3.75 deg a tick, and so is the stand a graceful leave of that
// pose eases home to.
void test_puppeteering_and_its_leave_keep_the_servos_top_speed_beside_the_ease_out_of_idle() {
    PoseMsg folded {};
    folded.joints[0] = true;
    std::copy_n(JOINT_LIMIT_DEG, 3, folded.legs[0]);

    VirtualRobot during;
    idleFromStand(during);
    dragAway(during);
    takeOver(during, MOTION_STATE::STAND, 1);
    during.animationPose(folded);
    Takeover run = follow(during, ticksFor(1.0f));
    TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, run.modes.back());
    TEST_ASSERT_FLOAT_WITHIN(CAP_TOLERANCE_DEG, PUPPET_STEP_DEG, run.largestStep);

    VirtualRobot after;
    idleFromStand(after);
    dragAway(after);
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, takeOver(after, MOTION_STATE::STAND, ticksFor(2.0f)).modes.back());
    takeOver(after, MOTION_STATE::ANIMATE, 1);
    after.animationPose(folded);
    run = follow(after, ticksFor(1.0f));
    TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, run.modes.back());
    TEST_ASSERT_FLOAT_WITHIN(CAP_TOLERANCE_DEG, PUPPET_STEP_DEG, run.largestStep);
    run = takeOver(after, MOTION_STATE::STAND, ticksFor(2.0f));
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, run.modes.back());
    TEST_ASSERT_FLOAT_WITHIN(CAP_TOLERANCE_DEG, PUPPET_STEP_DEG, run.largestStep);
}

namespace {

// The fixtures whose own motion, played from a stand, stays within the ease's pace, with their entry times;
// fx_overlay's body outruns the ease from a stand too.
struct PacedClip {
    const char *name;
    float entrySeconds;
};
constexpr PacedClip EASE_PACED_CLIPS[] = {{"fx_single", 0.31f}, {"fx_mixed_legs", 0.37f}, {"fx_params", 0.5f}};

// What the servos receive while a play runs, and the ticks, counted from the play, at which the player began the clip's
// Entry and ended it.
struct PlayRun {
    float largestStep = 0.0f;
    int entryTick = -1;
    int playingTick = -1;
};

PlayRun followPlay(VirtualRobot &robot, const char *clip, int ticks) {
    PlayRun run;
    int tick = 0;
    auto listener = EventBus<socket_message_AnimationStatus>::subscribe([&](const socket_message_AnimationStatus &s) {
        if (s.state == socket_message_AnimationState_ANIM_ENTRY && run.entryTick < 0) run.entryTick = tick;
        if (s.state == socket_message_AnimationState_ANIM_PLAYING && run.playingTick < 0) run.playingTick = tick;
    });
    robot.animationPlay(playOf(clip));
    std::array<float, 18> previous = servoAnglesOf(robot);
    for (; tick < ticks; ++tick) {
        robot.tick();
        for (int j = 0; j < 18; ++j)
            run.largestStep = std::max(run.largestStep, std::fabs(robot.servoAngles()[j] - previous[j]));
        previous = servoAnglesOf(robot);
    }
    return run;
}

void assertPacedPlay(const PlayRun &run, const PacedClip &clip, const char *from) {
    char msg[120];
    std::snprintf(msg, sizeof(msg), "%s played %s: largest step %.2f deg, Entry from tick %d to %d", clip.name, from,
                  run.largestStep, run.entryTick, run.playingTick);
    TEST_MESSAGE(msg);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(IDLE_EASE_STEP_DEG + CAP_TOLERANCE_DEG, run.largestStep, msg);
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(0, run.entryTick, msg);
    TEST_ASSERT_INT_WITHIN_MESSAGE(1, ticksFor(clip.entrySeconds), run.playingTick - run.entryTick, msg);
}

// A failed assertion jumps out of the test past every destructor, and a robot left alive keeps its bus subscriptions,
// which the next test's messages reach on a robot gone. So each run below ends its robot before anything is asserted.

PlayRun playFromStand(const char *clip) {
    VirtualRobot robot;
    standUp(robot);
    return followPlay(robot, clip, ticksFor(3.0f));
}

// The play `eased` ticks into the ease a STAND borrowed out of dragged joints, those ticks' largest step included.
PlayRun playDuringTheEase(const char *clip, int eased) {
    VirtualRobot robot;
    idleFromStand(robot);
    dragAway(robot);
    const float easeStep = takeOver(robot, MOTION_STATE::STAND, eased).largestStep;
    PlayRun run = followPlay(robot, clip, ticksFor(4.0f));
    run.largestStep = std::max(run.largestStep, easeStep);
    return run;
}

// The play in an ANIMATE asked for out of dragged joints, the tick after the ANIMATE.
PlayRun playInAnimateFromHeldAngles(const char *clip) {
    VirtualRobot robot;
    idleFromStand(robot);
    dragAway(robot);
    const Takeover entered = takeOver(robot, MOTION_STATE::ANIMATE, 1);
    PlayRun run = followPlay(robot, clip, ticksFor(4.0f));
    run.largestStep = std::max(run.largestStep, entered.largestStep);
    return run;
}

} // namespace

// A play asked for while the robot comes out of IDLE-held angles, during the ease a STAND borrowed or in an ANIMATE
// asked for from them, waits for the ease to reach the stand and then enters at the clip's own entry time, as from a
// stand: an Entry from the folded joints covers their whole distance in that time, 8.5 deg a tick on a dragged tibia.
void test_a_play_out_of_idle_held_angles_waits_for_the_ease() {
    for (const PacedClip &clip : EASE_PACED_CLIPS) {
        const PlayRun stood = playFromStand(clip.name);
        assertPacedPlay(stood, clip, "from a stand");
        TEST_ASSERT_EQUAL_INT(0, stood.entryTick);
        for (const int eased : {1, 10, 60}) {
            char from[48];
            std::snprintf(from, sizeof(from), "%d ticks into the ease", eased);
            assertPacedPlay(playDuringTheEase(clip.name, eased), clip, from);
        }
        assertPacedPlay(playInAnimateFromHeldAngles(clip.name), clip, "in an ANIMATE from the held angles");
    }
}

namespace {

enum class AfterPair { STOP, STAND, WALK };

// What IDLE and STAND taken in one tick from ANIMATE, a play's or the ease's, and `next` sent five ticks later leave.
struct PairRun {
    MOTION_STATE before = MOTION_STATE::DEACTIVATED; // the mode the pair was sent in
    float largestStep = 0.0f;
    MOTION_STATE last = MOTION_STATE::DEACTIVATED; // the mode three seconds after `next`
};

PairRun idleAndStandFromAnimate(bool playing, AfterPair next) {
    VirtualRobot robot;
    if (playing) {
        standUp(robot);
        robot.animationPlay(playOf("fx_single"));
        follow(robot, ticksFor(1.0f));
    } else {
        idleFromStand(robot);
        dragAway(robot);
        takeOver(robot, MOTION_STATE::STAND, 10);
    }
    PairRun run;
    run.before = robot.mode();
    robot.setMode(MOTION_STATE::IDLE);
    const Takeover pair = takeOver(robot, MOTION_STATE::STAND, 5);
    if (next == AfterPair::STOP) robot.animationStop();
    if (next == AfterPair::STAND) robot.setMode(MOTION_STATE::STAND);
    if (next == AfterPair::WALK) robot.setMode(MOTION_STATE::WALK);
    const Takeover after = follow(robot, ticksFor(3.0f));
    run.largestStep = std::max(pair.largestStep, after.largestStep);
    run.last = after.modes.back();
    return run;
}

} // namespace

// IDLE and then STAND taken in one tick while ANIMATE runs, a play or the ease out of IDLE, as a script sending them
// back to back does, are the IDLE that ends what ANIMATE ran and holds its pose and the STAND that eases from there, as
// when taken a tick apart: the robot is not left in ANIMATE, so a stop, a STAND and a WALK sent after the pair each act.
void test_idle_and_stand_in_one_tick_from_animate_ease_to_the_stand() {
    for (const bool playing : {true, false}) {
        for (const AfterPair next : {AfterPair::STOP, AfterPair::STAND, AfterPair::WALK}) {
            const PairRun run = idleAndStandFromAnimate(playing, next);
            const char *nextName = next == AfterPair::STOP ? "a stop" : next == AfterPair::STAND ? "STAND" : "WALK";
            char msg[96];
            std::snprintf(msg, sizeof(msg), "%s, then %s: largest step %.2f deg, mode %d",
                          playing ? "mid-play" : "mid-ease", nextName, run.largestStep, (int)run.last);
            TEST_MESSAGE(msg);
            TEST_ASSERT_EQUAL_MESSAGE(MOTION_STATE::ANIMATE, run.before, msg);
            TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(IDLE_EASE_STEP_DEG + CAP_TOLERANCE_DEG, run.largestStep, msg);
            TEST_ASSERT_EQUAL_MESSAGE(next == AfterPair::WALK ? MOTION_STATE::WALK : MOTION_STATE::STAND, run.last, msg);
        }
    }
}

namespace {

// Neutral and the envelope's extreme corners, clamped as the walk clamps them.
const Posture3 STRIDE_POSTURES[] = {{0, 0, 0}, {1, -1, -1}, {-1, -1, 0}, {1, 1, 1}};

constexpr float STRIDE_ANCHORS[3] = {-1.0f, 0.0f, 1.0f};

// A walk entered from a stand in the posture, as an operator would enter it.
void walkFromStandIn(VirtualRobot &robot, GaitType gait, const Posture3 &posture) {
    standUp(robot);
    robot.setGait(gait);
    robot.tick();
    hold(robot, withPosture(CommandMsg {}, {0, posture.h, posture.fd}), ticksFor(2.0f));
    robot.setMode(MOTION_STATE::WALK);
}

void describeStride(char *msg, size_t size, GaitType gait, const CommandMsg &c, const HeldRun &run) {
    std::snprintf(msg, size,
                  "gait %d stick (%.1f %.1f %.1f) stride %.0f posture (%.2f %.2f %.2f): %d clamped, largest "
                  "step %.2f deg",
                  (int)gait, c.lx, c.ly, c.rx, c.stride, c.s1, c.h, c.fd, run.clamped, run.largestStep);
}

} // namespace

// Every stride, from three tenths of the speed-optimal stroke to the longest the joints can travel, walks a full stick
// inside travel in every classic gait, at the envelope's extreme postures as at neutral.
void test_full_stick_walk_stays_in_travel_at_every_stride() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (Posture3 posture : STRIDE_POSTURES) {
            clampPosture(posture.s1, posture.h, posture.fd);
            for (const float stride : STRIDE_ANCHORS) {
                for (CommandMsg stick : FULL_STICKS) {
                    stick.stride = stride;
                    stick = withPosture(stick, posture);
                    VirtualRobot robot;
                    walkFromStandIn(robot, gait, posture);
                    // Long enough for the slowest posture settle before the walk, WAVE's of about 4 s.
                    const HeldRun entering = hold(robot, stick, ticksFor(5.0f));
                    const HeldRun walking = hold(robot, stick, ticksFor(1.0f));
                    char msg[200];
                    describeStride(msg, sizeof(msg), gait, stick, walking);
                    TEST_ASSERT_EQUAL_INT_MESSAGE(0, entering.clamped + walking.clamped, msg);
                    TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(STEP_BOUND_DEG,
                                                            std::max(entering.largestStep, walking.largestStep), msg);
                    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(STEPPING_DEG_PER_TICK, walking.largestStep, msg);
                    TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
                }
            }
        }
    }
}

// A gait at rest takes a new stride on the tick it arrives, since no foot moves with it; a walking one slews to it.
void test_a_gait_at_rest_takes_a_new_stride_at_once() {
    for (const GaitType gait : CLASSIC_GAITS) {
        VirtualRobot robot;
        walkIn(robot, gait);
        settleWalk(robot);
        TEST_ASSERT_FALSE(gaitIsCommanded(robot.gaitState()));
        const CommandMsg longest = fullStick(0, 1, 0, 1);
        robot.command(longest);
        robot.tick();
        const WalkTarget atOnce = classicWalkTarget(gait, 0, 1, 0, 1, 0, 0, 0, 1);
        TEST_ASSERT_EQUAL_FLOAT(atOnce.stepY, robot.gaitCommand().step_y);
        TEST_ASSERT_EQUAL_FLOAT(atOnce.phaseRate, robot.gaitCommand().phase_rate);

        hold(robot, longest, ticksFor(2.0f));
        const CommandMsg shortest = fullStick(0, 1, 0, -1);
        robot.command(shortest);
        robot.tick();
        TEST_ASSERT_TRUE(robot.gaitCommand().step_y >
                         classicWalkTarget(gait, 0, 1, 0, 1, 0, 0, 0, -1).stepY + GAIT_STRIDE_DEADBAND_MM);
    }
}

// The stride holds the stroke and the stick sets the cadence, so a small stick at the longest stride walks the longest
// stroke slowly rather than a short stroke at the walk's pace.
void test_a_small_stick_at_the_longest_stride_steps_slowly_with_the_full_stroke() {
    for (const GaitType gait : CLASSIC_GAITS) {
        VirtualRobot robot;
        walkIn(robot, gait);
        const CommandMsg small {0, 0.1f, 0, 0, 0, 0, 0, 0, 1};
        const HeldRun run = hold(robot, small, ticksFor(3.0f));
        const AxisEnvelope longest = envelopeAt(gait, 0, 0, 0, 0, 1);
        const gait_state_t &live = robot.gaitState();
        char msg[160];
        describeStride(msg, sizeof(msg), gait, small, run);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3f * longest.stroke, longest.stroke, live.step_y, msg);
        TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.0f, live.phase_rate, msg);
        TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.4f * longest.cadence, live.phase_rate, msg);
        TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(STEPPING_DEG_PER_TICK, run.largestStep, msg);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
    }
}

// A release at either end of the stride, from a small stick and from full ones, stops as every stop does: through the
// stop ramp, inside travel, no faster in a tick than the stop bound, and with no cadence left once the gait is still.
void test_a_release_at_any_stride_stops_with_no_cadence_left() {
    // The steady walks peak at 3.95 deg in a tick, and a stop from the longest stroke at up to 4.02 (measured). The
    // bound is there to catch gross overruns, such as the 10.4 deg a stop reached before the stop ramp.
    constexpr float RELEASE_STEP_BOUND_DEG = 4.05f;
    constexpr int PHASES = 4, PHASE_STEP_TICKS = 14;
    for (const GaitType gait : CLASSIC_GAITS) {
        for (Posture3 posture : STRIDE_POSTURES) {
            clampPosture(posture.s1, posture.h, posture.fd);
            for (const float stride : {-1.0f, 1.0f}) {
                for (CommandMsg stick : {CommandMsg {0, 0.1f, 0, 0, 0, 0, 0, 0}, fullStick(0, 1, 0), fullStick(1, 0, 0),
                                         fullStick(0, 0, 1), fullStick(1, 1, 1)}) {
                    stick.stride = stride;
                    stick = withPosture(stick, posture);
                    for (int k = 0; k <= PHASES; ++k) {
                        const Stop stop = k < PHASES ? Stop::RELEASE : Stop::DEAD_MAN;
                        const StopRun run = walkThenStop(gait, stick, stop, k < PHASES ? k * PHASE_STEP_TICKS : 0);
                        char msg[200];
                        describeStride(msg, sizeof(msg), gait, stick, run.stop);
                        TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(RELEASE_STEP_BOUND_DEG, run.stop.largestStep, msg);
                        TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.stop.clamped, msg);
                        TEST_ASSERT_TRUE_MESSAGE(run.cadenceFollowsStroke, msg);
                        TEST_ASSERT_FALSE_MESSAGE(run.cadenceLeft, msg);
                    }
                }
            }
        }
    }
}

// A stride change while walking is a gait-command change, not a posture change: the walk slews from one stroke and
// cadence to the other (motion.h slewStride, a full -1 to +1 in 2 s) without pausing, inside travel, and no joint moves
// faster in a tick than a stop may move it.
void test_a_stride_change_while_walking_eases_through_without_a_pause() {
    const float changes[][2] = {{-1, 1}, {1, -1}, {0, 1}, {0, -1}};
    for (const GaitType gait : CLASSIC_GAITS) {
        for (Posture3 posture : STRIDE_POSTURES) {
            clampPosture(posture.s1, posture.h, posture.fd);
            for (const CommandMsg &full : {fullStick(0, 1, 0), fullStick(1, 0, 0), fullStick(0, 0, 1),
                                           fullStick(1, 1, 1), CommandMsg {0.1f, 0.3f, 0, 0, 0, 0, 0, 0}}) {
                for (const auto &change : changes) {
                    CommandMsg stick = withPosture(full, posture);
                    stick.stride = change[0];
                    const auto owner = std::make_unique<VirtualRobot>(); // heap: the stack would exceed -Wstack-usage
                    VirtualRobot &robot = *owner;
                    walkFromStandIn(robot, gait, posture);
                    hold(robot, stick, ticksFor(5.0f));
                    TEST_ASSERT_TRUE(walkResumed(robot));
                    stick.stride = change[1];
                    HeldRun run;
                    bool paused = false;
                    float previous[18];
                    std::copy_n(robot.angles(), 18, previous);
                    for (int t = 0; t < ticksFor(3.0f); ++t) {
                        if (t % COMMAND_PERIOD_TICKS == 0) robot.command(stick);
                        robot.tick();
                        paused |= !walkResumed(robot) || !gaitIsCommanded(robot.gaitState());
                        for (int j = 0; j < 18; ++j) {
                            const float angle = robot.angles()[j];
                            if (withinTravel(angle, j % 3) != angle) run.clamped++;
                            run.largestStep = std::max(run.largestStep, std::fabs(angle - previous[j]));
                            previous[j] = angle;
                        }
                    }
                    char msg[220];
                    describeStride(msg, sizeof(msg), gait, stick, run);
                    TEST_ASSERT_FALSE_MESSAGE(paused, msg);
                    TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
                    TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(STEP_BOUND_DEG, run.largestStep, msg);
                    const WalkTarget expected = classicWalkTarget(gait, stick.lx, stick.ly, stick.rx, stick.s,
                                                                  posture.s1, posture.h, posture.fd, change[1]);
                    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(expected.stepX, robot.gaitCommand().step_x, msg);
                    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(expected.stepY, robot.gaitCommand().step_y, msg);
                    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(expected.stepAngle, robot.gaitCommand().step_angle, msg);
                    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(expected.phaseRate, robot.gaitCommand().phase_rate, msg);
                }
            }
        }
    }
}

namespace {

// A ControllerInputData as the firmware gets it: encoded by the client, decoded by the transport.
socket_message_ControllerInputData overTheWire(const socket_message_ControllerInputData &sent) {
    uint8_t wire[socket_message_ControllerInputData_size];
    pb_ostream_t out = pb_ostream_from_buffer(wire, sizeof wire);
    TEST_ASSERT_TRUE(pb_encode(&out, socket_message_ControllerInputData_fields, &sent));
    socket_message_ControllerInputData received = socket_message_ControllerInputData_init_zero;
    pb_istream_t in = pb_istream_from_buffer(wire, out.bytes_written);
    TEST_ASSERT_TRUE(pb_decode(&in, socket_message_ControllerInputData_fields, &received));
    return received;
}

} // namespace

// Every field of the message reaches the command in its place, and the telemetry reports the command in that order.
void test_a_controller_input_reaches_the_command_field_by_field() {
    socket_message_ControllerInputData sent = socket_message_ControllerInputData_init_zero;
    sent.has_left = sent.has_right = true;
    sent.left = {0.1f, 0.2f};
    sent.right = {0.3f, 0.4f};
    sent.height = 0.5f;
    sent.speed = 0.6f;
    sent.s1 = 0.7f;
    sent.feet_distance = 0.8f;
    sent.stride = 0.9f;
    VirtualRobot robot;
    robot.input(overTheWire(sent));
    robot.tick();
    const float expected[9] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f};
    TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, robot.telemetryCommand(), 9);
}

// A client that never sends the stride, such as an older app, walks the speed-optimal stride: proto3 leaves a zero
// float off the wire, so the message with stride 0 is the one such a client sends. Strides -1 and +1 show the field is
// read at all.
void test_a_client_that_never_sends_the_stride_walks_the_speed_optimal_one() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (const float stride : STRIDE_ANCHORS) {
            VirtualRobot robot;
            walkIn(robot, gait);
            settleWalk(robot);
            socket_message_ControllerInputData sent = socket_message_ControllerInputData_init_zero;
            sent.has_left = true;
            sent.left.y = 1.0f;
            sent.speed = 1.0f;
            sent.stride = stride;
            robot.input(overTheWire(sent));
            robot.tick();
            const WalkTarget expected = classicWalkTarget(gait, 0, 1, 0, 1, 0, 0, 0, stride);
            TEST_ASSERT_TRUE(expected.phaseRate > 0.0f);
            TEST_ASSERT_EQUAL_FLOAT(expected.stepY, robot.gaitCommand().step_y);
            TEST_ASSERT_EQUAL_FLOAT(expected.phaseRate, robot.gaitCommand().phase_rate);
        }
    }
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

void test_stand_stick_reaches_eighteen_degrees_without_exceeding_joint_travel() {
    constexpr float LIMIT = 18.f * static_cast<float>(M_PI) / 180.f;
    Kinematics kin;
    for (int direction = 0; direction < 8; ++direction) {
        const float angle = direction * static_cast<float>(M_PI) / 4.f;
        const CommandMsg command {0, 0, std::cos(angle), std::sin(angle), 0, 0, 0, 0, 0};
        VirtualRobot robot;
        standUp(robot);
        largestStep(robot, ticksFor(1.f), &command);
        const auto &body = robot.bodyState();
        TEST_ASSERT_FLOAT_WITHIN(0.001f, -command.rx * LIMIT, body.phi);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, command.ry * LIMIT, body.omega);
        for (int leg = 0; leg < 6; ++leg) TEST_ASSERT_TRUE(kin.footReachable(body, leg));
        for (int joint = 0; joint < 18; ++joint) {
            TEST_ASSERT_TRUE(std::isfinite(robot.angles()[joint]));
            TEST_ASSERT_FLOAT_WITHIN(0.001f, withinTravel(robot.angles()[joint], joint % 3), robot.angles()[joint]);
        }
    }
}

void test_stand_translation_and_walking_tilt_keep_their_ranges() {
    VirtualRobot robot;
    standUp(robot);
    const CommandMsg shift {1, -1, 0, 0, 0, 0, 0, 0, 0};
    largestStep(robot, ticksFor(1.f), &shift);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -50.f, robot.bodyState().xm);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.f, robot.bodyState().ym);

    robot.setMode(MOTION_STATE::WALK);
    const CommandMsg tilt {0, 0, 0, 1, 0, 0, 0, 0, 0};
    largestStep(robot, ticksFor(2.f), &tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.f, robot.bodyState().omega);
    robot.setImu({{1.f, 0, 0}, {0, 0, 1}, {0, 0, 0}});
    largestStep(robot, ticksFor(1.f), &tilt);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.254f, std::fabs(robot.bodyState().omega));
}

void test_stand_self_levelling_keeps_the_larger_tilt_bounded() {
    VirtualRobot robot;
    standUp(robot);
    robot.setImu({{1.f, -1.f, 0}, {0, 0, 1}, {0, 0, 0}});
    const CommandMsg tilt {0, 0, -1, 1, 0, 0, 0, 0, 0};
    largestStep(robot, ticksFor(1.f), &tilt);
    constexpr float LIMIT = 18.f * static_cast<float>(M_PI) / 180.f;
    TEST_ASSERT_TRUE(std::fabs(robot.bodyState().phi) <= LIMIT + 0.001f);
    TEST_ASSERT_TRUE(std::fabs(robot.bodyState().omega) <= LIMIT + 0.001f);
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
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_new_robot_is_deactivated);
    RUN_TEST(test_a_new_robot_drives_no_servo);
    RUN_TEST(test_idle_from_boot_wakes_the_boards_but_sends_no_pose);
    RUN_TEST(test_idle_after_a_deactivated_stand_leaves_every_servo_limp);
    RUN_TEST(test_a_standing_robot_drives_its_servos_to_the_motions_pose);
    RUN_TEST(test_a_mode_request_reaches_the_motion_through_the_bus);
    RUN_TEST(test_a_mode_is_reported_applied_in_the_tick_that_applies_it);
    RUN_TEST(test_entering_idle_holds_the_servos_angles);
    RUN_TEST(test_a_joint_in_idle_moves_only_its_servo);
    RUN_TEST(test_a_joint_in_idle_is_held_within_its_travel);
    RUN_TEST(test_a_joint_outside_idle_is_ignored);
    RUN_TEST(test_a_joint_sent_behind_a_mode_out_of_idle_is_ignored);
    RUN_TEST(test_the_servos_are_centred_only_in_idle);
    RUN_TEST(test_a_centring_is_carried_out_in_arrival_order);
    RUN_TEST(test_a_centring_behind_a_queued_mode_is_refused);
    RUN_TEST(test_entering_idle_publishes_the_held_angles_once);
    RUN_TEST(test_idle_entered_from_a_pose_beyond_travel_publishes_the_held_angles);
    RUN_TEST(test_a_joint_write_in_idle_publishes_the_angles);
    RUN_TEST(test_centring_in_idle_publishes_the_centre_pose);
    RUN_TEST(test_idle_reports_its_limp_servos_as_limp);
    RUN_TEST(test_deactivating_publishes_every_servo_limp);
    RUN_TEST(test_the_servo_pages_switch_is_applied_by_the_control_task);
    RUN_TEST(test_a_standing_robot_holds_still);
    RUN_TEST(test_a_stick_held_in_walk_steps_the_legs);
    RUN_TEST(test_a_walk_obeys_its_last_command_for_600_ms_and_no_longer);
    RUN_TEST(test_the_telemetry_flags_the_dead_man_stop_from_the_tick_it_fires);
    RUN_TEST(test_the_dead_man_window_runs_from_the_commands_arrival_not_its_take);
    RUN_TEST(test_the_apps_heartbeat_leaves_a_handheld_walk_alone);
    RUN_TEST(test_the_app_is_heard_again_600_ms_after_the_handheld_lets_go);
    RUN_TEST(test_the_dead_man_stops_a_handheld_walk_whose_link_drops_under_the_apps_heartbeat);
    RUN_TEST(test_a_stale_stick_stream_does_not_resume_the_walk_after_the_handheld);
    RUN_TEST(test_the_handheld_driving_is_reported_with_the_mode_on_each_change);
    RUN_TEST(test_an_idle_handheld_leaves_the_apps_posture_and_walk_alone_after_a_trim);
    RUN_TEST(test_a_lone_idle_handheld_keeps_its_trim_with_no_dead_man_stop);
    RUN_TEST(test_the_dead_man_stops_a_lost_apps_walk_beside_an_idle_handheld);
    RUN_TEST(test_the_handheld_takes_over_an_apps_walk_and_stops_it_on_release);
    RUN_TEST(test_the_telemetry_reports_the_servo_write_of_its_own_tick);
    RUN_TEST(test_a_height_only_command_is_obeyed_for_2_s_after_silence);
    RUN_TEST(test_no_servo_is_driven_past_its_joint_travel);
    RUN_TEST(test_a_walk_without_commands_comes_to_rest);
    RUN_TEST(test_deactivating_mid_walk_freezes_the_joints);
    RUN_TEST(test_nothing_posted_changes_the_motion_before_the_next_tick);
    RUN_TEST(test_a_deactivation_and_a_borrow_in_one_tick_leave_the_robot_deactivated);
    RUN_TEST(test_a_borrow_with_nothing_to_play_hands_the_mode_back);
    RUN_TEST(test_a_play_from_a_stand_holds_in_animate_until_stopped);
    RUN_TEST(test_a_stop_in_the_tick_of_the_play_ends_it);
    RUN_TEST(test_a_pose_in_the_tick_of_the_borrow_is_taken);
    RUN_TEST(test_angles_on_the_bus_leave_the_motions_angles_alone);
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
    RUN_TEST(test_walk_nn_reads_the_sticks_alone);
    RUN_TEST(test_walk_nn_walks_the_stance_it_was_trained_on);
    RUN_TEST(test_an_interrupted_settle_starts_over);
    RUN_TEST(test_a_walk_from_a_stand_at_rest_steps_at_once);
    RUN_TEST(test_a_walk_from_a_stand_at_rest_settles_a_new_posture_first);
    RUN_TEST(test_a_tuned_walk_after_the_stand_spends_its_rest_posture);
    RUN_TEST(test_every_stop_eases_the_stroke_out_with_the_cadence);
    RUN_TEST(test_leaving_idle_after_drags_eases_the_joints_to_the_stand);
    RUN_TEST(test_leaving_limp_idle_stands_up_as_from_deactivated);
    RUN_TEST(test_idle_taken_during_the_ease_out_of_idle_holds_the_pose_the_ease_left);
    RUN_TEST(test_a_second_mode_in_the_tick_of_the_ease_out_of_idle_waits_for_it);
    RUN_TEST(test_puppeteering_and_its_leave_keep_the_servos_top_speed_beside_the_ease_out_of_idle);
    RUN_TEST(test_a_play_out_of_idle_held_angles_waits_for_the_ease);
    RUN_TEST(test_idle_and_stand_in_one_tick_from_animate_ease_to_the_stand);
    RUN_TEST(test_full_stick_walk_stays_in_travel_at_every_stride);
    RUN_TEST(test_a_gait_at_rest_takes_a_new_stride_at_once);
    RUN_TEST(test_a_small_stick_at_the_longest_stride_steps_slowly_with_the_full_stroke);
    RUN_TEST(test_a_release_at_any_stride_stops_with_no_cadence_left);
    RUN_TEST(test_a_stride_change_while_walking_eases_through_without_a_pause);
    RUN_TEST(test_a_controller_input_reaches_the_command_field_by_field);
    RUN_TEST(test_a_client_that_never_sends_the_stride_walks_the_speed_optimal_one);
    RUN_TEST(test_a_dead_man_stop_takes_the_neutral_posture_once_stopped);
    RUN_TEST(test_a_dead_man_stop_in_a_stand_waits_for_the_feet);
    RUN_TEST(test_a_classic_walk_ignores_the_tilt_stick);
    RUN_TEST(test_a_classic_walk_after_a_shifted_stand_stays_in_travel);
    RUN_TEST(test_stand_stick_reaches_eighteen_degrees_without_exceeding_joint_travel);
    RUN_TEST(test_stand_translation_and_walking_tilt_keep_their_ranges);
    RUN_TEST(test_stand_self_levelling_keeps_the_larger_tilt_bounded);
    RUN_TEST(test_self_levelling_reaches_down_with_the_low_side);
    return UNITY_END();
}
