// MotionService's goal moves through the virtual robot (firmware/host/virtual_robot.h): a goal walks the classic gait
// on its own sticks and odometry, and what an operator, a script or a transport does meanwhile ends it or leaves it
// alone (docs/superpowers/specs/2026-10-17-goal-moves-design.md).

#define USE_POLICY 1
#define USE_MPU6050 1

#include <unity.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <vector>
#include <communication/comm_base.hpp>
#include <gait_speed.h>
#include <virtual_robot.h>

namespace {

constexpr int TICKS_PER_SECOND = 200;
constexpr float DT = 1.0f / TICKS_PER_SECOND;
constexpr int COMMAND_PERIOD_TICKS = 4; // the app's 20 ms stick period
constexpr int GYRO_PERIOD_TICKS = 5;    // hexapod.h reads the IMU every 25 ms
constexpr float TWO_PI = 2 * PI_F;
constexpr float DEG = PI_F / 180;
constexpr float GOAL_LIMIT_S = 40;

int ticksFor(float seconds) { return static_cast<int>(std::lround(seconds * TICKS_PER_SECOND)); }

Goal forward(float metres, float speed = 0.5f) { return {GoalKind::FORWARD, metres, 0, 0, speed}; }
Goal turn(float angle, float speed = 0.5f) { return {GoalKind::TURN, 0, angle, 0, speed}; }
Goal arc(float radius, float angle, float speed = 0.5f) { return {GoalKind::ARC, 0, angle, radius, speed}; }

// The style's posture and stride; its sticks are the goal's own.
CommandMsg style(float s1 = 0, float h = 0, float fd = 0, float stride = 0) {
    return {0, 0, 0, 0, h, 0, s1, fd, stride};
}

// The pose `end` reached from `start`, in the frame the robot faced at `start`.
OdometryEstimate relative(const OdometryEstimate &start, const OdometryEstimate &end) {
    const float dx = end.x - start.x, dy = end.y - start.y, c = std::cos(start.yaw), s = std::sin(start.yaw);
    return {.x = c * dx + s * dy,
            .y = -s * dx + c * dy,
            .yaw = std::remainder(end.yaw - start.yaw, TWO_PI),
            .kinematicYaw = std::remainder(end.kinematicYaw - start.kinematicYaw, TWO_PI),
            .distance = end.distance - start.distance};
}

bool running(GoalState s) { return s == GoalState::SETTLING || s == GoalState::RUNNING; }

// The robot as a script finds it on the floor: its feet do not slip, so its gyro reads the turn its feet make over the
// gyro's period. Every goal status pushed is recorded.
struct Rig {
    VirtualRobot robot;
    std::deque<float> kinematicYaw {0};
    std::vector<socket_message_GoalStatus> pushes;
    EventBus<socket_message_GoalStatus>::Handle listener = EventBus<socket_message_GoalStatus>::subscribe(
        [this](const socket_message_GoalStatus &s) { pushes.push_back(s); });

    int ticks = 0;

    void tick() {
        ++ticks;
        Peripherals::Imu imu;
        if (kinematicYaw.size() > 1)
            imu.gyroRad[2] = std::remainder(kinematicYaw.back() - kinematicYaw.front(), TWO_PI) /
                             (DT * static_cast<float>(kinematicYaw.size() - 1));
        robot.setImu(imu);
        robot.tick();
        kinematicYaw.push_back(robot.odometry().kinematicYaw);
        if (kinematicYaw.size() > GYRO_PERIOD_TICKS + 1) kinematicYaw.pop_front();
    }

    void run(float seconds) {
        for (int t = 0; t < ticksFor(seconds); ++t) tick();
    }

    GoalState state() const { return static_cast<GoalState>(robot.goalStatus().state); }

    // Ticks until the goal ends, `before` called ahead of each tick with its number; returns the ticks run.
    int runGoal(const std::function<void(int)> &before = {}) {
        int t = 0;
        for (; t < ticksFor(GOAL_LIMIT_S) && running(state()); ++t) {
            if (before) before(t);
            tick();
        }
        return t;
    }

    // Runs while the goal is in `phase`, at most `seconds`.
    void runWhile(GoalState phase, float seconds = GOAL_LIMIT_S) {
        for (int t = 0; t < ticksFor(seconds) && state() == phase; ++t) tick();
    }

    float largestStep(float seconds) {
        float previous[18], largest = 0;
        std::copy_n(robot.angles(), 18, previous);
        for (int t = 0; t < ticksFor(seconds); ++t) {
            tick();
            for (int j = 0; j < 18; ++j) largest = std::max(largest, std::fabs(robot.angles()[j] - previous[j]));
            std::copy_n(robot.angles(), 18, previous);
        }
        return largest;
    }
};

// One rig at a time, as one VirtualRobot exists at a time. A failed assertion leaves its test without unwinding, so the
// rig is held here and deleted by the next rig or by tearDown, never left subscribed to the bus from a dead stack.
std::unique_ptr<Rig> currentRig;

Rig &newRig() {
    currentRig.reset();
    currentRig = std::make_unique<Rig>();
    return *currentRig;
}

void standUp(Rig &rig) {
    rig.robot.setMode(MOTION_STATE::STAND);
    rig.run(1.0f);
}

// Stood up, a 1 m forward goal accepted and walking.
void walkingForward(Rig &rig) {
    standUp(rig);
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(forward(1.0f), style(), GaitType::TRI_GATE));
    rig.run(1.5f);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());
    TEST_ASSERT_GREATER_THAN_FLOAT(GAIT_STRIDE_DEADBAND_MM, rig.robot.gaitCommand().step_y);
}

// Standing at rest with no cadence left: the gait is still and so is every joint, and nothing would step it again. A
// goal is judged once every foot is within FEET_AT_REST_MM of its place, so the last one is let down first.
void assertAtRest(Rig &rig) {
    rig.run(0.25f);
    const gait_state_t &live = rig.robot.gaitState(), &target = rig.robot.gaitCommand();
    TEST_ASSERT_FALSE(gaitIsCommanded(live));
    TEST_ASSERT_EQUAL_FLOAT(0, live.phase_rate);
    TEST_ASSERT_EQUAL_FLOAT(0, target.step_x);
    TEST_ASSERT_EQUAL_FLOAT(0, target.step_y);
    TEST_ASSERT_EQUAL_FLOAT(0, target.step_angle);
    TEST_ASSERT_EQUAL_FLOAT(0, target.phase_rate);
    TEST_ASSERT_LESS_THAN_FLOAT(1e-3f, rig.largestStep(0.5f));
}

// A transport's client registry as main.cpp wires it: a client that leaves is reported with its adapter and id.
class FakeAdapter : public CommAdapterBase {
  public:
    bool hasClient() const override { return true; }
    void leave(int id) { removeClient(id); }

  protected:
    void send(const uint8_t *, size_t, int) override {}
};

// A goal walked from a stand to its end: every tick's joint angles, and the body height the walk held at the end.
struct GoalRun {
    std::vector<std::array<float, 18>> angles;
    float bodyHeight;
};

GoalRun walkGoal(const Goal &goal, const CommandMsg &style, const std::function<void(Rig &, int)> &before = {}) {
    Rig &rig = newRig();
    standUp(rig);
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(goal, style, GaitType::TRI_GATE));
    GoalRun run;
    rig.runGoal([&](int t) {
        if (before) before(rig, t);
        run.angles.emplace_back();
        std::copy_n(rig.robot.angles(), 18, run.angles.back().begin());
    });
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    run.bodyHeight = rig.robot.bodyCommand().zm;
    // The IMU read and the angles' publish run on process-wide timers (EXECUTE_EVERY_N_MS, 25 and 20 ms): a run that
    // ends on a whole period of both leaves the next one starting in the same phase.
    while (rig.ticks % 20 != 0) rig.tick();
    return run;
}

void assertSameWalk(const GoalRun &expected, const GoalRun &actual) {
    TEST_ASSERT_GREATER_THAN(ticksFor(2), expected.angles.size());
    TEST_ASSERT_EQUAL(expected.angles.size(), actual.angles.size());
    for (size_t t = 0; t < expected.angles.size(); ++t)
        TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected.angles[t].data(), actual.angles[t].data(), 18);
}

} // namespace

void setUp() {}
// The rig and the IMU stand-in are process-wide, so a test must not hand them on, even when it fails.
void tearDown() {
    currentRig.reset();
    Peripherals::setImu({});
}

// From a stand, the goal enters WALK through a mode request, walks 1 m on the odometry and ends standing at rest.
void test_a_forward_goal_walks_one_metre_and_ends_at_rest() {
    Rig &rig = newRig();
    standUp(rig);
    std::vector<ModeMsg> modes;
    auto modeListener = EventBus<ModeMsg>::subscribe([&modes](const ModeMsg &m) { modes.push_back(m); });
    const OdometryEstimate start = rig.robot.odometry();
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(forward(1.0f), style(), GaitType::TRI_GATE));
    const int ticks = rig.runGoal();
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, rig.robot.mode());
    TEST_ASSERT_TRUE(std::any_of(modes.begin(), modes.end(), [](const ModeMsg &m) {
        return m.mode == MOTION_STATE::WALK && m.kind == ModeMsgKind::REQUEST;
    }));
    const OdometryEstimate end = rig.robot.odometry();
    const OdometryEstimate walked = relative(start, end);
    TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_M, 1.0f, walked.y);
    TEST_ASSERT_FLOAT_WITHIN(GoalController::CROSS_REACHED_M, 0, walked.x);
    TEST_ASSERT_LESS_THAN_INT(ticksFor(20), ticks);
    const socket_message_GoalStatus status = rig.robot.goalStatus();
    TEST_ASSERT_EQUAL(socket_message_GoalKind_GOAL_FORWARD, status.kind);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, status.target);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, status.progress);
    TEST_ASSERT_EQUAL_FLOAT(end.x, status.x);
    TEST_ASSERT_EQUAL_FLOAT(end.y, status.y);
    TEST_ASSERT_EQUAL_FLOAT(end.yaw, status.yaw);
    assertAtRest(rig);
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, rig.robot.mode());
}

// A sideways goal is positive to the right, in the firmware's body frame; its stick is the lateral one.
void test_a_sideways_goal_walks_to_the_right() {
    Rig &rig = newRig();
    standUp(rig);
    const OdometryEstimate start = rig.robot.odometry();
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove({GoalKind::SIDEWAYS, 0.3f, 0, 0, 0.5f}, style(), GaitType::TRI_GATE));
    rig.runGoal();
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    const OdometryEstimate walked = relative(start, rig.robot.odometry());
    TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_M, 0.3f, walked.x);
    TEST_ASSERT_FLOAT_WITHIN(GoalController::CROSS_REACHED_M, 0, walked.y);
    assertAtRest(rig);
}

void test_a_turn_goal_turns_90_degrees_in_place() {
    Rig &rig = newRig();
    standUp(rig);
    const OdometryEstimate start = rig.robot.odometry();
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(turn(90 * DEG), style(), GaitType::TRI_GATE));
    rig.runGoal();
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    const OdometryEstimate turned = relative(start, rig.robot.odometry());
    TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_RAD, 90 * DEG, turned.yaw);
    TEST_ASSERT_LESS_THAN_FLOAT(0.03f, std::hypot(turned.x, turned.y));
    assertAtRest(rig);
}

// A left arc of 0.5 m through 90 deg ends half a metre ahead and half a metre left, facing left.
void test_an_arc_goal_ends_on_its_arc() {
    Rig &rig = newRig();
    standUp(rig);
    const OdometryEstimate start = rig.robot.odometry();
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(arc(0.5f, 90 * DEG), style(), GaitType::TRI_GATE));
    rig.runGoal();
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    const OdometryEstimate end = relative(start, rig.robot.odometry());
    TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_RAD, 90 * DEG, end.yaw);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(GoalController::ARC_REACHED_M, std::hypot(end.x + 0.5f, end.y - 0.5f));
    assertAtRest(rig);
}

// Review focus 1: the app's heartbeat, the facade's keep-alive and the ESP-NOW controller's centred packets carry a
// posture, a speed, a stride and a tilt of their own; during a goal none of it changes a single joint angle.
void test_centred_commands_during_a_goal_change_nothing() {
    const CommandMsg centred {0, 0, 0, 0.7f, -0.5f, -1, 1, 0.5f, -1};
    socket_message_ControllerInputData input = socket_message_ControllerInputData_init_zero;
    input.has_left = input.has_right = true;
    input.right.y = 0.7f;
    input.height = -0.5f;
    input.speed = -1;
    input.s1 = 1;
    input.feet_distance = 0.5f;
    input.stride = -1;
    const GoalRun quiet = walkGoal(forward(0.3f), style());
    const GoalRun heard = walkGoal(forward(0.3f), style(), [&](Rig &rig, int t) {
        if (t % COMMAND_PERIOD_TICKS != 0) return;
        if (t % (2 * COMMAND_PERIOD_TICKS) == 0)
            rig.robot.command(centred);
        else
            rig.robot.input(input);
    });
    assertSameWalk(quiet, heard);
}

// Review focus 1: any stick off centre, from any source, cancels the goal and walks as the operator asks at once.
void test_an_off_centre_stick_cancels_the_goal_and_drives_the_walk() {
    for (int axis = 0; axis < 3; ++axis) {
        Rig &rig = newRig();
        walkingForward(rig);
        CommandMsg stick {};
        (axis == 0 ? stick.lx : axis == 1 ? stick.ly : stick.rx) = axis == 1 ? -0.5f : 0.5f;
        rig.robot.command(stick);
        rig.tick();
        TEST_ASSERT_EQUAL(GoalState::CANCELLED, rig.state());
        rig.run(0.3f);
        TEST_ASSERT_EQUAL(GoalState::CANCELLED, rig.state());
        const gait_state_t &target = rig.robot.gaitCommand();
        if (axis == 0)
            TEST_ASSERT_LESS_THAN_FLOAT(-GAIT_STRIDE_DEADBAND_MM, target.step_x); // lx > 0 walks left
        else
            TEST_ASSERT_EQUAL_FLOAT(0, target.step_x);
        if (axis == 1)
            TEST_ASSERT_LESS_THAN_FLOAT(-GAIT_STRIDE_DEADBAND_MM, target.step_y);
        else
            TEST_ASSERT_EQUAL_FLOAT(0, target.step_y);
        if (axis == 2)
            TEST_ASSERT_GREATER_THAN_FLOAT(GAIT_YAW_DEADBAND_RAD, target.step_angle);
        else
            TEST_ASSERT_EQUAL_FLOAT(0, target.step_angle);
    }
}

// A mode change cancels and the new mode stands the robot at rest; a request for the walk it is in changes no mode.
void test_a_mode_change_cancels_the_goal() {
    {
        Rig &rig = newRig();
        walkingForward(rig);
        rig.robot.setMode(MOTION_STATE::WALK);
        rig.run(0.5f);
        TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());
    }
    Rig &rig = newRig();
    walkingForward(rig);
    rig.robot.setMode(MOTION_STATE::STAND);
    rig.tick();
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, rig.state());
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, rig.robot.mode());
    rig.run(2.0f);
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, rig.state());
    assertAtRest(rig);
}

// A gait asked for over the wire cancels, and the walk stops before it takes the gait; a selection of the gait already
// asked for is no change.
void test_a_gait_change_over_the_wire_cancels_the_goal() {
    {
        Rig &rig = newRig();
        walkingForward(rig);
        rig.robot.setGait(GaitType::TRI_GATE);
        rig.run(0.5f);
        TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());
    }
    Rig &rig = newRig();
    walkingForward(rig);
    rig.robot.setGait(GaitType::BI_GATE);
    rig.tick();
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, rig.state());
    rig.run(2.0f);
    TEST_ASSERT_EQUAL(GaitType::BI_GATE, rig.robot.gait());
    assertAtRest(rig);
}

// AUTO picks the gait from the goal's own sticks and swaps it while the gait is at rest; that is no gait change.
void test_an_auto_swap_at_rest_does_not_cancel_the_goal() {
    Rig &rig = newRig();
    standUp(rig);
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, rig.robot.gait());
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(forward(0.5f, 0.25f), style(), GaitType::AUTO));
    bool swappedWhileRunning = false;
    GaitType last = rig.robot.gait();
    GoalState lastState = rig.state();
    rig.runGoal([&](int) {
        swappedWhileRunning |=
            lastState == GoalState::RUNNING && rig.state() == GoalState::RUNNING && rig.robot.gait() != last;
        last = rig.robot.gait();
        lastState = rig.state();
    });
    TEST_ASSERT_TRUE(swappedWhileRunning);
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
}

// Review focus 3: the dead-man stop guards the operator's stick; a goal walks on with no command at all, in the posture
// of its style, after both the steering and the link timeouts would have stopped a walk.
void test_the_dead_man_timeout_does_not_stop_a_goal() {
    Rig &rig = newRig();
    standUp(rig);
    rig.robot.setMode(MOTION_STATE::WALK);
    rig.tick();
    rig.robot.command({0, 1, 0, 0, 0, 0, 0, 0});
    rig.run(0.2f);
    TEST_ASSERT_TRUE(gaitIsCommanded(rig.robot.gaitState()));
    // A raised body, which the dead-man's neutral posture would lower.
    float s1 = 0, h = 0.4f, fd = 0;
    clampPosture(s1, h, fd);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.1f, h);
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(forward(0.6f, 0.3f), style(s1, h, fd), GaitType::TRI_GATE));
    rig.runWhile(GoalState::SETTLING);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());
    int runningTicks = 0;
    rig.runGoal([&](int) {
        runningTicks += rig.state() == GoalState::RUNNING;
        TEST_ASSERT_EQUAL_FLOAT(bodyHeightMm(h), rig.robot.bodyCommand().zm);
    });
    TEST_ASSERT_GREATER_THAN(ticksFor(2.5f), runningTicks);
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    assertAtRest(rig);
}

// Review focus 3: a cancel mid-stride stops the walk through the stop ramp and leaves no cadence.
void test_a_cancelled_goal_stops_with_no_cadence_left() {
    Rig &rig = newRig();
    walkingForward(rig);
    rig.robot.goalCancel();
    rig.tick();
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, rig.state());
    rig.run(2.0f);
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, rig.state());
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, rig.robot.mode());
    assertAtRest(rig);
}

// Review focus 2: a goal belongs to the client that asked for it, by transport and id. Another client leaving, on the
// same transport or with the same id on another, leaves it walking; its own leaving cancels it.
void test_the_owner_leaving_cancels_the_goal_and_another_client_leaving_does_not() {
    Rig &rig = newRig();
    // Each holds a Message and an encode buffer, some 4 KB: on the heap.
    const auto websocket = std::make_unique<FakeAdapter>(), serial = std::make_unique<FakeAdapter>();
    for (FakeAdapter *adapter : {websocket.get(), serial.get()})
        adapter->onClientGone([&rig](const CommAdapterBase *from, int id) { rig.robot.clientGone({from, id}); });
    standUp(rig);
    const GoalOwner owner {static_cast<const CommAdapterBase *>(websocket.get()), 7};
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(forward(1.0f), style(), GaitType::TRI_GATE, owner));
    rig.run(1.5f);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());
    websocket->leave(8);
    serial->leave(7);
    rig.run(0.5f);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());
    websocket->leave(7);
    rig.tick();
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, rig.state());
    rig.run(2.0f);
    assertAtRest(rig);
}

// Review focus 4: the stance change and the body height a style settles into before the walk are no travel.
void test_settling_into_the_style_adds_no_travel() {
    Rig &rig = newRig();
    standUp(rig);
    const OdometryEstimate start = rig.robot.odometry();
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(forward(0.2f), style(0, 0.3f, 0.6f), GaitType::TRI_GATE));
    float before[18];
    std::copy_n(rig.robot.angles(), 18, before);
    rig.runWhile(GoalState::SETTLING);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, rig.state());
    float moved = 0;
    for (int j = 0; j < 18; ++j) moved = std::max(moved, std::fabs(rig.robot.angles()[j] - before[j]));
    TEST_ASSERT_GREATER_THAN_FLOAT(5.0f, moved);
    const OdometryEstimate settled = relative(start, rig.robot.odometry());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, settled.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, settled.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, settled.yaw);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, settled.distance);
    rig.runGoal();
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
}

// Review focus 5: a style outside the walkable region walks exactly as the walkable posture the sliders' clamp gives
// it.
void test_an_unwalkable_style_is_clamped_like_the_sliders() {
    float s1 = 0, h = 1, fd = -1;
    TEST_ASSERT_FALSE(postureWalkable(s1, h, fd));
    clampPosture(s1, h, fd);
    TEST_ASSERT_TRUE(h > 0.1f && h < 1 && fd > -1 && fd < -0.1f);
    const GoalRun asked = walkGoal(forward(0.3f), style(0, 1, -1));
    const GoalRun clamped = walkGoal(forward(0.3f), style(s1, h, fd));
    assertSameWalk(clamped, asked);
    TEST_ASSERT_EQUAL_FLOAT(bodyHeightMm(h), asked.bodyHeight);
}

// Review focus 5: TUNED has no speed envelope to walk a goal on; it is refused, and nothing changes.
void test_a_tuned_style_is_refused() {
    Rig &rig = newRig();
    standUp(rig);
    TEST_ASSERT_EQUAL_INT(422, rig.robot.goalMove(forward(0.3f), style(), GaitType::TUNED));
    rig.run(0.5f);
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, rig.robot.mode());
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, rig.robot.gait());
    TEST_ASSERT_EQUAL(GoalState::IDLE, rig.state());
    TEST_ASSERT_EQUAL_UINT32(0, rig.robot.goalStatus().id);
    TEST_ASSERT_EQUAL(0, rig.pushes.size());
}

// Review focus 5: only a stand or a classic walk takes a goal: not deactivated, idle, posed, animating or walking the
// policy.
void test_a_goal_is_refused_unless_standing_or_walking() {
    for (const MOTION_STATE mode : {MOTION_STATE::DEACTIVATED, MOTION_STATE::IDLE, MOTION_STATE::POSE,
                                    MOTION_STATE::ANIMATE, MOTION_STATE::WALK_NN}) {
        Rig &rig = newRig();
        standUp(rig);
        rig.robot.setMode(mode);
        rig.run(0.2f);
        TEST_ASSERT_EQUAL(mode, rig.robot.mode());
        TEST_ASSERT_EQUAL_INT(409, rig.robot.goalMove(forward(0.3f), style(), GaitType::TRI_GATE));
        rig.run(0.5f);
        TEST_ASSERT_EQUAL(mode, rig.robot.mode());
        TEST_ASSERT_EQUAL(GoalState::IDLE, rig.state());
        TEST_ASSERT_EQUAL(0, rig.pushes.size());
    }
}

void test_a_malformed_goal_is_refused() {
    Rig &rig = newRig();
    standUp(rig);
    const float nan = std::nanf("");
    CommandMsg nanStyle = style();
    nanStyle.fd = nan;
    TEST_ASSERT_EQUAL_INT(400, rig.robot.goalMove(forward(0.3f, 0), style(), GaitType::TRI_GATE));
    TEST_ASSERT_EQUAL_INT(400, rig.robot.goalMove(forward(0.3f, 1.5f), style(), GaitType::TRI_GATE));
    TEST_ASSERT_EQUAL_INT(400, rig.robot.goalMove(forward(nan), style(), GaitType::TRI_GATE));
    TEST_ASSERT_EQUAL_INT(400, rig.robot.goalMove(forward(0), style(), GaitType::TRI_GATE));
    TEST_ASSERT_EQUAL_INT(400, rig.robot.goalMove(arc(0, 90 * DEG), style(), GaitType::TRI_GATE));
    TEST_ASSERT_EQUAL_INT(
        400, rig.robot.goalMove({static_cast<GoalKind>(9), 0.3f, 0.3f, 0.3f, 0.5f}, style(), GaitType::TRI_GATE));
    TEST_ASSERT_EQUAL_INT(400, rig.robot.goalMove(forward(0.3f), nanStyle, GaitType::TRI_GATE));
    TEST_ASSERT_EQUAL_INT(400, rig.robot.goalMove(forward(0.3f), style(), static_cast<GaitType>(9)));
    rig.run(0.5f);
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, rig.robot.mode());
    TEST_ASSERT_EQUAL(GoalState::IDLE, rig.state());
    TEST_ASSERT_EQUAL(0, rig.pushes.size());
}

// A goal asked for while another runs takes over: the one in progress ends cancelled and the new one walks.
void test_a_new_goal_replaces_the_one_in_progress() {
    Rig &rig = newRig();
    walkingForward(rig);
    const uint32_t first = rig.robot.goalStatus().id;
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(turn(-45 * DEG), style(), GaitType::TRI_GATE));
    const uint32_t second = rig.robot.goalStatus().id;
    TEST_ASSERT_GREATER_THAN_UINT32(first, second);
    rig.runGoal();
    TEST_ASSERT_TRUE(std::any_of(rig.pushes.begin(), rig.pushes.end(), [first](const socket_message_GoalStatus &s) {
        return s.id == first && s.state == socket_message_GoalState_GOAL_CANCELLED;
    }));
    TEST_ASSERT_EQUAL_UINT32(second, rig.robot.goalStatus().id);
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
}

// The status is pushed on every change of state and on every 5 % of progress, and on nothing else.
void test_the_status_is_pushed_on_state_changes_and_every_5_percent() {
    Rig &rig = newRig();
    standUp(rig);
    TEST_ASSERT_EQUAL_INT(202, rig.robot.goalMove(forward(1.0f), style(), GaitType::TRI_GATE));
    const uint32_t id = rig.robot.goalStatus().id;
    const int ticks = rig.runGoal();
    TEST_ASSERT_EQUAL(GoalState::REACHED, rig.state());
    const auto step = [](float p) { return static_cast<int>(std::floor(p * 20)); };
    const std::vector<socket_message_GoalStatus> &pushes = rig.pushes;
    TEST_ASSERT_LESS_THAN(ticks / 20, pushes.size());
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_SETTLING, pushes.front().state);
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_REACHED, pushes.back().state);
    for (size_t i = 0; i < pushes.size(); ++i) {
        TEST_ASSERT_EQUAL_UINT32(id, pushes[i].id);
        if (i == 0) continue;
        TEST_ASSERT_TRUE(pushes[i].state != pushes[i - 1].state ||
                         step(pushes[i].progress) != step(pushes[i - 1].progress));
    }
    // The walk covers every 5 % step, and each was pushed.
    for (int k = 0; k <= 20; ++k)
        TEST_ASSERT_TRUE(std::any_of(pushes.begin(), pushes.end(),
                                     [&](const socket_message_GoalStatus &s) { return step(s.progress) == k; }));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_forward_goal_walks_one_metre_and_ends_at_rest);
    RUN_TEST(test_a_sideways_goal_walks_to_the_right);
    RUN_TEST(test_a_turn_goal_turns_90_degrees_in_place);
    RUN_TEST(test_an_arc_goal_ends_on_its_arc);
    RUN_TEST(test_centred_commands_during_a_goal_change_nothing);
    RUN_TEST(test_an_off_centre_stick_cancels_the_goal_and_drives_the_walk);
    RUN_TEST(test_a_mode_change_cancels_the_goal);
    RUN_TEST(test_a_gait_change_over_the_wire_cancels_the_goal);
    RUN_TEST(test_an_auto_swap_at_rest_does_not_cancel_the_goal);
    RUN_TEST(test_the_dead_man_timeout_does_not_stop_a_goal);
    RUN_TEST(test_a_cancelled_goal_stops_with_no_cadence_left);
    RUN_TEST(test_the_owner_leaving_cancels_the_goal_and_another_client_leaving_does_not);
    RUN_TEST(test_settling_into_the_style_adds_no_travel);
    RUN_TEST(test_an_unwalkable_style_is_clamped_like_the_sliders);
    RUN_TEST(test_a_tuned_style_is_refused);
    RUN_TEST(test_a_goal_is_refused_unless_standing_or_walking);
    RUN_TEST(test_a_malformed_goal_is_refused);
    RUN_TEST(test_a_new_goal_replaces_the_one_in_progress);
    RUN_TEST(test_the_status_is_pushed_on_state_changes_and_every_5_percent);
    return UNITY_END();
}
