// The heading hold of a walk driven by the sticks (heading_hold.h): the controller alone, then MotionService walking
// the virtual robot (firmware/host/virtual_robot.h) on a floor that turns it where its feet do not.

#define USE_POLICY 1
#define USE_MPU6050 1

#include <unity.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <heading_hold.h>
#include <imu_frame.h>
#include <virtual_robot.h>

namespace {

constexpr int TICKS_PER_SECOND = 200;
constexpr float DT = 1.0f / TICKS_PER_SECOND;
constexpr int COMMAND_PERIOD_TICKS = 4; // the app's 20 ms stick period
constexpr float DEG = PI_F / 180;

int ticksFor(float seconds) { return static_cast<int>(std::lround(seconds * TICKS_PER_SECOND)); }

// A hold walking forward at `yaw`, run until it has taken its heading there.
HeadingHold heldAt(float yaw) {
    HeadingHold hold;
    for (int t = 0; t < ticksFor(HeadingHold::SETTLE_S) + 1; ++t) hold.update(0, 0.5f, 0, true, yaw, DT);
    TEST_ASSERT_TRUE(hold.held());
    return hold;
}

} // namespace

// A turn stick let go leaves the robot turning on, so the heading is taken only once it has been centred for
// SETTLE_S, and nothing is steered before: a heading taken at once would be the turn's, not the walk's.
void test_the_heading_is_taken_once_the_turn_stick_has_settled() {
    HeadingHold hold;
    const int settle = ticksFor(HeadingHold::SETTLE_S);
    float yaw = 0;
    for (int t = 0; t < settle - 1; ++t) {
        yaw += 0.5f * DT; // still turning on
        TEST_ASSERT_EQUAL_FLOAT(0, hold.update(0, 0.5f, 0, true, yaw, DT));
    }
    TEST_ASSERT_FALSE(hold.held());
    hold.update(0, 0.5f, 0, true, yaw, DT);
    hold.update(0, 0.5f, 0, true, yaw, DT);
    TEST_ASSERT_TRUE(hold.held());
    // Held at the yaw it settled on: 10 deg further counter-clockwise turns back clockwise.
    TEST_ASSERT_GREATER_THAN_FLOAT(0, hold.update(0, 0.5f, 0, true, yaw + 10 * DEG, DT));
}

// The turn stick is MotionService's, clockwise positive, 2 per rad of error and at most 0.3, as a goal's: a yaw
// counter-clockwise of the heading turns clockwise, across the wrap at pi too, and an error within the deadband steers
// nothing, so the fused yaw's noise does not step the gait.
void test_the_turn_stick_steers_against_the_error_by_the_goals_law() {
    HeadingHold hold = heldAt(0);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, hold.update(0, 0.5f, 0, true, 0.05f, DT));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.1f, hold.update(0, 0.5f, 0, true, -0.05f, DT));
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, GoalController::HEADING_STICK_MAX, hold.update(0, 0.5f, 0, true, 1.0f, DT));
    TEST_ASSERT_EQUAL_FLOAT(0, hold.update(0, 0.5f, 0, true, 0.9f * HeadingHold::DEADBAND_RAD, DT));
    TEST_ASSERT_EQUAL_FLOAT(0, hold.update(0, 0.5f, 0, true, -0.9f * HeadingHold::DEADBAND_RAD, DT));

    HeadingHold wrapped = heldAt(PI_F - 0.02f);
    // 0.04 rad counter-clockwise, past the wrap.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.08f, wrapped.update(0, 0.5f, 0, true, -PI_F + 0.02f, DT));
}

// A turn stick, a centred walk stick or a client that stops asking lets the heading go, and the next walk takes its
// own once settled: the robot is not turned back to where it once faced.
void test_a_turn_a_stop_or_no_hold_lets_the_heading_go() {
    struct Case {
        float lx, ly, rx;
        bool asked;
    };
    for (const Case let : {Case {0, 0.5f, 0.4f, true}, Case {0, 0, 0, true}, Case {0, 0.5f, 0, false}}) {
        HeadingHold hold = heldAt(0);
        TEST_ASSERT_EQUAL_FLOAT(0, hold.update(let.lx, let.ly, let.rx, let.asked, 0.5f, DT));
        TEST_ASSERT_FALSE(hold.held());
        for (int t = 0; t < ticksFor(HeadingHold::SETTLE_S) + 1; ++t)
            TEST_ASSERT_EQUAL_FLOAT(0, hold.update(0, 0.5f, 0, true, 0.5f, DT));
        TEST_ASSERT_TRUE(hold.held());
        TEST_ASSERT_EQUAL_FLOAT(0, hold.update(0, 0.5f, 0, true, 0.5f, DT));
    }
    // A sideways walk is held as a forward one.
    HeadingHold sideways;
    for (int t = 0; t < ticksFor(HeadingHold::SETTLE_S) + 1; ++t) sideways.update(-0.5f, 0, 0, true, 0, DT);
    TEST_ASSERT_TRUE(sideways.held());
}

namespace {

// The virtual robot on a floor that turns it `drift` rad/s counter-clockwise beyond what its feet turn it while it
// steps, as slip does: its DMP's yaw is the feet's turn plus the drift. A robot standing still does not slip, so its
// odometry learns no drift as the gyro's bias. The client sends its sticks every 20 ms.
struct Rig {
    VirtualRobot robot;
    float drift = 0;
    float slipped = 0;
    socket_message_ControllerInputData sticks = socket_message_ControllerInputData_init_zero;
    bool sending = true;
    int ticks = 0;

    void tick() {
        if (sending && ticks % COMMAND_PERIOD_TICKS == 0) robot.input(sticks);
        ++ticks;
        if (gaitIsCommanded(robot.gaitState())) slipped += drift * DT;
        Peripherals::Imu imu;
        imu.rpyRad[2] = imu_frame::YAW_SIGN * (robot.odometry().kinematicYaw + slipped);
        robot.setImu(imu);
        robot.tick();
    }

    void run(float s) {
        for (int t = 0; t < ticksFor(s); ++t) tick();
    }

    void walk(float ly, float rx, bool hold) {
        sticks.has_left = sticks.has_right = true;
        sticks.left = {0, ly};
        sticks.right = {rx, 0};
        sticks.hold_heading = hold;
    }
};

std::unique_ptr<Rig> currentRig;

Rig &newRig(float drift) {
    currentRig.reset();
    currentRig = std::make_unique<Rig>();
    currentRig->drift = drift;
    currentRig->robot.setMode(MOTION_STATE::STAND);
    currentRig->run(1.0f);
    currentRig->robot.setMode(MOTION_STATE::WALK);
    return *currentRig;
}

// The fused yaw turned over `seconds` of a forward walk on the drifting floor, the hold on or off.
float turnedWalking(float seconds, bool hold, float drift) {
    Rig &rig = newRig(drift);
    rig.walk(0.6f, 0, hold);
    rig.run(1.0f);
    const float start = rig.robot.odometry().yaw;
    rig.run(seconds);
    return std::remainder(rig.robot.odometry().yaw - start, 2 * PI_F);
}

} // namespace

void setUp() {}
void tearDown() { currentRig.reset(); }

// A floor that turns the walk 0.15 rad/s carries it well over a radian off its heading in 8 s with the hold off; held,
// the walk ends within the hold's steady error of it, the drift over the gain, about 0.1 rad with the turn a stick of
// 0.3 gives on TRI.
void test_a_held_walk_keeps_its_heading_on_a_drifting_floor() {
    constexpr float DRIFT = 0.15f;
    const float unheld = turnedWalking(8.0f, false, DRIFT);
    TEST_ASSERT_GREATER_THAN_FLOAT(1.0f, unheld);
    const float held = turnedWalking(8.0f, true, DRIFT);
    TEST_ASSERT_FLOAT_WITHIN(0.12f, 0, held);
    // The other way round as well.
    TEST_ASSERT_FLOAT_WITHIN(0.12f, 0, turnedWalking(8.0f, true, -DRIFT));
}

// On a floor that does not turn it, the hold steers nothing: the walk's own yaw rocking stays within the deadband, so
// the gait is commanded no turn at all.
void test_a_held_walk_on_a_still_floor_is_commanded_no_turn() {
    Rig &rig = newRig(0);
    rig.walk(0.6f, 0, true);
    for (int t = 0; t < ticksFor(4.0f); ++t) {
        rig.tick();
        TEST_ASSERT_EQUAL_FLOAT(0, rig.robot.gaitCommand().step_angle);
    }
}

// The operator's turn stick turns the robot as without the hold, and the heading it ends on is the one held next.
void test_the_turn_stick_turns_and_its_end_heading_is_held() {
    Rig &rig = newRig(0.15f);
    rig.walk(0.6f, 0, true);
    rig.run(2.0f);
    const float before = rig.robot.odometry().yaw;
    rig.walk(0.6f, -0.6f, true); // counter-clockwise
    rig.run(1.5f);
    rig.walk(0.6f, 0, true);
    rig.run(HeadingHold::SETTLE_S + 0.05f);
    const float settled = rig.robot.odometry().yaw;
    TEST_ASSERT_GREATER_THAN_FLOAT(0.5f, std::remainder(settled - before, 2 * PI_F));
    rig.run(6.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.12f, 0, std::remainder(rig.robot.odometry().yaw - settled, 2 * PI_F));
}

// The dead-man stops a held walk with no turn left over: the stop clears the command and with it the hold.
void test_a_dead_man_stop_leaves_no_held_turn() {
    Rig &rig = newRig(0.15f);
    rig.walk(0.6f, 0, true);
    rig.run(4.0f);
    TEST_ASSERT_NOT_EQUAL_FLOAT(0, rig.robot.gaitCommand().step_angle);
    rig.sending = false;
    rig.run(1.0f);
    const gait_state_t &target = rig.robot.gaitCommand();
    TEST_ASSERT_EQUAL_FLOAT(0, target.step_x);
    TEST_ASSERT_EQUAL_FLOAT(0, target.step_y);
    TEST_ASSERT_EQUAL_FLOAT(0, target.step_angle);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_the_heading_is_taken_once_the_turn_stick_has_settled);
    RUN_TEST(test_the_turn_stick_steers_against_the_error_by_the_goals_law);
    RUN_TEST(test_a_turn_a_stop_or_no_hold_lets_the_heading_go);
    RUN_TEST(test_a_held_walk_keeps_its_heading_on_a_drifting_floor);
    RUN_TEST(test_a_held_walk_on_a_still_floor_is_commanded_no_turn);
    RUN_TEST(test_the_turn_stick_turns_and_its_end_heading_is_held);
    RUN_TEST(test_a_dead_man_stop_leaves_no_held_turn);
    return UNITY_END();
}
