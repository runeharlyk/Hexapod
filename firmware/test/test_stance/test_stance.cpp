// The stance as MotionService keeps it, through the virtual robot (firmware/host/virtual_robot.h): a state a client
// selects (StanceData), which no command, handheld frame or goal changes, pushed on each change, walked by the classic
// timings and refused where a gait or mode walks the default stance alone
// (docs/superpowers/specs/2026-10-30-crab-stance-design.md, 5.2).

// The animation store reads the parity fixtures, whose "fx_single" holds its last keyframe until stopped.
#define ANIMATION_DIRECTORY "animations/fixtures"
#define USE_POLICY 1
#define USE_MPU6050 1

#include <unity.h>
#include <algorithm>
#include <type_traits>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>
#include <gait_speed.h>
#include <leg_clearance.h>
#include <virtual_robot.h>
#include <event_bus.h>

namespace {

constexpr int TICKS_PER_SECOND = 200;
constexpr float DT = 1.0f / TICKS_PER_SECOND;
constexpr int COMMAND_PERIOD_TICKS = 4; // the app's 20 ms stick period
constexpr GaitType CLASSIC_GAITS[4] = {GaitType::TRI_GATE, GaitType::BI_GATE, GaitType::WAVE, GaitType::RIPPLE};
// The servo's top speed in a control tick (gait_envelope.h SERVO_RAD_S), 3.75 deg.
const float SERVO_STEP_DEG = gait_envelope::SERVO_RAD_S * DT * 180.0f / PI_F;
// A foot within this of its resting height stands on the floor; the stance's press is 0.002 mm deep and a swing's
// first tick lifts its foot 0.08 mm (gait.h's Bezier lift, 15 mm, over TRI's 97-tick swing).
constexpr float ON_FLOOR_MM = 0.01f;
// A planted foot that moves less than this in a tick holds its place.
constexpr float PLANTED_SLIDE_MM = 0.01f;

int ticksFor(float seconds) { return static_cast<int>(std::lround(seconds * TICKS_PER_SECOND)); }

AnimationCommandMsg playOf(const char *name) {
    AnimationCommandMsg play {};
    std::snprintf(play.name, sizeof(play.name), "%s", name);
    return play;
}

// A command with the posture and these sticks.
CommandMsg posture(float h, float fd, float lx = 0, float ly = 0, float rx = 0) { return {lx, ly, rx, 0, h, 0, 0, fd}; }

const char *stanceName(Stance s) { return s == Stance::CRAB ? "crab" : "default"; }

// What a run of ticks did to the legs: the largest joint step, the least leg clearance, and the largest horizontal
// slide of a foot that stood on the floor at both ends of a tick.
struct Watch {
    float largestStep = 0.0f;
    float leastClearance = INFINITY;
    float largestSlide = 0.0f;
    int ticks = 0;
};

// Ticks `ticks` times, sending `command` at the app's stick period when given, `each` called after every tick; an
// `each` that returns true ends the watch at that tick, which the watch then leaves out.
template <typename Each>
Watch watch(VirtualRobot &robot, int ticks, const CommandMsg *command, Each each) {
    Watch w;
    float angles[18];
    std::copy_n(robot.angles(), 18, angles);
    BodyStateMsg feet = robot.bodyState();
    for (int t = 0; t < ticks; ++t) {
        if (command && t % COMMAND_PERIOD_TICKS == 0) robot.command(*command);
        robot.tick();
        if constexpr (std::is_same_v<decltype(each(t)), bool>) {
            if (each(t)) break;
        } else {
            each(t);
        }
        const BodyStateMsg &now = robot.bodyState();
        for (int j = 0; j < 18; ++j) w.largestStep = std::max(w.largestStep, std::fabs(robot.angles()[j] - angles[j]));
        w.leastClearance = std::min(w.leastClearance, legClearance(robot.angles()));
        for (int leg = 0; leg < 6; ++leg) {
            const float rest = DEFAULT_STANCE[leg][2];
            const bool planted = feet.feet[leg][2] <= rest + ON_FLOOR_MM && now.feet[leg][2] <= rest + ON_FLOOR_MM;
            if (planted)
                w.largestSlide = std::max(w.largestSlide, std::hypot(now.feet[leg][0] - feet.feet[leg][0],
                                                                     now.feet[leg][1] - feet.feet[leg][1]));
        }
        std::copy_n(robot.angles(), 18, angles);
        feet = now;
        ++w.ticks;
    }
    return w;
}

Watch watch(VirtualRobot &robot, int ticks, const CommandMsg *command = nullptr) {
    return watch(robot, ticks, command, [](int) {});
}

void standUp(VirtualRobot &robot) {
    robot.setMode(MOTION_STATE::STAND);
    watch(robot, ticksFor(1.0f));
}

// Every foot stands where the stance puts it at the feet distance, in the body frame of a level, unshifted body.
void assertFeetOn(VirtualRobot &robot, Stance stance, float fd, const char *msg) {
    float expected[6][4];
    stanceFeet(stance, feetScale(fd), expected);
    for (int leg = 0; leg < 6; ++leg)
        for (int k = 0; k < 3; ++k)
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, expected[leg][k], robot.bodyState().feet[leg][k], msg);
}

void assertStepsCleanly(const Watch &w, const char *msg) {
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(SERVO_STEP_DEG, w.largestStep, msg);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE(LEG_CLEARANCE_MM, w.leastClearance, msg);
    TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(PLANTED_SLIDE_MM, w.largestSlide, msg);
}

// Records the stances the motion publishes.
struct StancePushes {
    std::vector<Stance> heard;
    StanceBus::Handle listener = StanceBus::subscribe([this](const StanceMsg &s) { heard.push_back(s.stance); });
};

// Records the gaits on the bus, a client's requests and the motion's own selections alike, as the gait topic pushes
// them.
struct GaitPushes {
    std::vector<GaitType> heard;
    EventBus<GaitMsg>::Handle listener =
        EventBus<GaitMsg>::subscribe([this](const GaitMsg &g) { heard.push_back(g.gait); });
};

} // namespace

void setUp() { host_bus::resetAll(); }
void tearDown() { Peripherals::setImu({}); }

// The crab points every leg straight out to its side: each foot on the line through its coxa mount across the body, at
// every posture a client can command, clamped as the stand clamps it, with every leg pair apart.
void test_the_crab_stands_every_leg_straight_out_at_every_posture() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    for (const float h : {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f}) {
        for (const float fd : {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f}) {
            const CommandMsg command = posture(h, fd);
            const Watch w = watch(robot, ticksFor(3.0f), &command);
            char msg[64];
            std::snprintf(msg, sizeof(msg), "h %.1f fd %.1f", h, fd);
            TEST_ASSERT_EQUAL_MESSAGE(Stance::CRAB, robot.feetStance(), msg);
            for (int leg = 0; leg < 6; ++leg) {
                const float *foot = robot.bodyState().feet[leg];
                const float mountX = hexapodConfig.mountX[leg], mountY = hexapodConfig.mountY[leg];
                TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, mountY, foot[1], msg);
                TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(std::fabs(mountX) + 50.0f, std::copysign(1.0f, mountX) * foot[0],
                                                       msg);
            }
            TEST_ASSERT_TRUE_MESSAGE(legsClear(robot.angles()), msg);
            TEST_ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE(LEG_CLEARANCE_MM, w.leastClearance, msg);
        }
    }
}

// A stance change in a stand steps the feet through the timing's own schedule, foot by foot, both ways: no planted foot
// slides, no joint steps faster than the servo, and every leg pair stays apart. The servo bound holds at the neutral
// posture, where this runs. Off it, at near-full reach (h -1, fd 1), a swing's lift-off steps up to 13.95 deg a tick in
// WAVE and RIPPLE, as a plain feet-distance change on the default stance already does (13.56): the repositioning's
// lift-off, which predates the stance and is left as a known limit.
void test_a_stance_change_in_a_stand_steps_the_feet_without_dragging_them() {
    for (const GaitType gait : CLASSIC_GAITS) {
        VirtualRobot robot;
        standUp(robot);
        robot.setGait(gait);
        robot.tick();
        for (const Stance stance : {Stance::CRAB, Stance::DEFAULT}) {
            robot.setStance(stance);
            const Watch w = watch(robot, ticksFor(3.0f));
            char msg[64];
            std::snprintf(msg, sizeof(msg), "gait %d to the %s", (int)gait, stanceName(stance));
            assertStepsCleanly(w, msg);
            assertFeetOn(robot, stance, 0.0f, msg);
        }
    }
}

// A stance asked for in IDLE is taken by the stand that follows, which steps the feet to it as a stand does.
void test_a_stance_asked_for_in_idle_is_stepped_into_by_the_stand() {
    VirtualRobot robot;
    standUp(robot);
    robot.setMode(MOTION_STATE::IDLE);
    watch(robot, 10);
    robot.setStance(Stance::CRAB);
    watch(robot, 10);
    TEST_ASSERT_EQUAL(Stance::CRAB, robot.stance());
    TEST_ASSERT_EQUAL(Stance::DEFAULT, robot.feetStance());
    robot.setMode(MOTION_STATE::STAND);
    const Watch w = watch(robot, ticksFor(3.0f));
    assertStepsCleanly(w, "idle to stand");
    assertFeetOn(robot, Stance::CRAB, 0.0f, "idle to stand");
}

// A stance asked for mid-walk stops the walk first, as a posture does: the feet start for the stance only once the
// gait has stopped stepping, then step to it, and the walk resumes on its own in the stance.
void test_a_stance_change_mid_walk_stops_the_walk_first() {
    for (const GaitType gait : CLASSIC_GAITS) {
        VirtualRobot robot;
        standUp(robot);
        robot.setGait(gait);
        robot.tick();
        robot.setMode(MOTION_STATE::WALK);
        const CommandMsg sideways = posture(0, 0, 0.5f);
        watch(robot, ticksFor(1.5f), &sideways);
        TEST_ASSERT_TRUE(gaitIsCommanded(robot.gaitState()));
        robot.setStance(Stance::CRAB);
        // The walk stops at the stance's own tick, not at the next command's.
        robot.tick();
        TEST_ASSERT_FALSE(gaitIsCommanded(robot.gaitCommand()));
        int taken = -1;
        bool commandedWhenTaken = true, stroked = false;
        const Watch stopping = watch(robot, ticksFor(5.0f), &sideways, [&](int t) {
            const gait_state_t &c = robot.gaitCommand();
            stroked |= c.step_x != 0 || c.step_y != 0 || c.step_angle != 0;
            if (robot.feetStance() != Stance::CRAB) return false;
            taken = t;
            commandedWhenTaken = gaitIsCommanded(robot.gaitState());
            return true;
        });
        // From the stance's take until the walk is commanded again, the gait is stopped and only steps the feet.
        const Watch stepping =
            watch(robot, ticksFor(5.0f), &sideways, [&](int) { return gaitIsCommanded(robot.gaitCommand()); });
        const Watch walking = watch(robot, ticksFor(1.0f), &sideways);
        char msg[32];
        std::snprintf(msg, sizeof(msg), "gait %d", (int)gait);
        TEST_ASSERT_FALSE_MESSAGE(stroked, msg);
        TEST_ASSERT_GREATER_THAN_INT_MESSAGE(0, taken, msg);
        TEST_ASSERT_FALSE_MESSAGE(commandedWhenTaken, msg);
        assertStepsCleanly(stepping, msg);
        TEST_ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE(LEG_CLEARANCE_MM,
                                                   std::min(stopping.leastClearance, walking.leastClearance), msg);
        TEST_ASSERT_TRUE_MESSAGE(gaitIsCommanded(robot.gaitState()), msg);
        TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(GAIT_STRIDE_DEADBAND_MM, robot.gaitState().step_x, msg);
    }
}

// A walk asked for from a stand at rest, in the tick the stance arrives in, starts from the stand's feet only if they
// stand on the stance: here they do not, so it settles into the crab first, and strokes only once the feet stand on it.
void test_a_walk_from_a_stand_at_rest_in_another_stance_settles_into_it_first() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    robot.setMode(MOTION_STATE::WALK);
    const CommandMsg sideways = posture(0, 0, 0.5f);
    bool strokedOffCrab = false;
    const Watch w = watch(robot, ticksFor(5.0f), &sideways, [&](int) {
        strokedOffCrab |= gaitIsCommanded(robot.gaitState()) && robot.feetStance() != Stance::CRAB;
    });
    TEST_ASSERT_FALSE(strokedOffCrab);
    TEST_ASSERT_EQUAL(Stance::CRAB, robot.feetStance());
    TEST_ASSERT_TRUE(gaitIsCommanded(robot.gaitState()));
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(LEG_CLEARANCE_MM, w.leastClearance);
}

// The stance is a client's alone: neither a client's command, which never names it, nor the handheld's frames, nor a
// goal change it. A goal walks the stance in force.
void test_commands_the_handheld_and_a_goal_leave_the_stance_as_it_is() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    watch(robot, ticksFor(3.0f));
    StancePushes pushes;
    const CommandMsg command = posture(0.2f, 0.3f);
    watch(robot, ticksFor(1.0f), &command);
    for (int t = 0; t < ticksFor(1.0f); ++t) {
        if (t % COMMAND_PERIOD_TICKS == 0)
            robot.handheld({CONTROLLER_PACKET_VERSION, 0,
                            static_cast<int16_t>(t < ticksFor(0.5f) ? AXIS_FULL_SCALE / 2 : 0), 0, 0, 0, 0});
        robot.tick();
    }
    watch(robot, ticksFor(1.0f));
    TEST_ASSERT_EQUAL_INT(
        202, robot.goalMove({GoalKind::SIDEWAYS, 0.2f, 0, 0, 0.5f}, posture(0.2f, 0.3f), GaitType::TRI_GATE));
    float leastClearance = INFINITY;
    for (int t = 0; t < ticksFor(30.0f) && robot.goalStatus().state != socket_message_GoalState_GOAL_REACHED; ++t) {
        robot.tick();
        leastClearance = std::min(leastClearance, legClearance(robot.angles()));
        TEST_ASSERT_EQUAL(Stance::CRAB, robot.feetStance());
    }
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_REACHED, robot.goalStatus().state);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(LEG_CLEARANCE_MM, leastClearance);
    TEST_ASSERT_EQUAL(Stance::CRAB, robot.stance());
    TEST_ASSERT_EQUAL(0, pushes.heard.size());
}

// A stance other than the one selected ends a goal on arrival, as a gait does, since the goal's pace was reckoned in
// the stance it started in; the stance in force asked for again does not.
void test_a_stance_change_ends_a_goal_and_a_repeat_does_not() {
    VirtualRobot robot;
    standUp(robot);
    TEST_ASSERT_EQUAL_INT(202,
                          robot.goalMove({GoalKind::FORWARD, 1.0f, 0, 0, 0.5f}, posture(0, 0), GaitType::TRI_GATE));
    watch(robot, ticksFor(1.5f));
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_RUNNING, robot.goalStatus().state);
    robot.setStance(Stance::DEFAULT);
    watch(robot, 2);
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_RUNNING, robot.goalStatus().state);
    robot.setStance(Stance::CRAB);
    watch(robot, 2);
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_CANCELLED, robot.goalStatus().state);
}

// The stance topic: its snapshot is the stance in force from boot, a change is pushed once, and a repeat never (a
// refusal is answered with the stance in force, test_walk_nn_and_the_crab_exclude_each_other).
void test_the_stance_is_published_once_per_change() {
    VirtualRobot robot;
    StanceMsg latest {Stance::CRAB};
    TEST_ASSERT_TRUE(StanceBus::peek(latest));
    TEST_ASSERT_EQUAL(Stance::DEFAULT, latest.stance);
    StancePushes pushes;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    watch(robot, 2);
    robot.setStance(Stance::CRAB);
    watch(robot, 2);
    TEST_ASSERT_EQUAL(1, pushes.heard.size());
    TEST_ASSERT_EQUAL(Stance::CRAB, pushes.heard[0]);
    TEST_ASSERT_TRUE(StanceBus::peek(latest));
    TEST_ASSERT_EQUAL(Stance::CRAB, latest.stance);
    robot.setStance(Stance::DEFAULT);
    watch(robot, 2);
    TEST_ASSERT_EQUAL(2, pushes.heard.size());
    TEST_ASSERT_EQUAL(Stance::DEFAULT, pushes.heard[1]);
    TEST_ASSERT_TRUE(StanceBus::peek(latest));
    TEST_ASSERT_EQUAL(Stance::DEFAULT, latest.stance);
}

// The crab asked for while AUTO or TUNED is selected selects TRI_GATE, published, and then takes the crab: TUNED and
// AUTO walk the default stance alone. Standing, and walking, where TUNED's walk stops for it and never steps the crab.
void test_the_crab_in_auto_or_tuned_selects_tri_gate_first() {
    for (const GaitType gait : {GaitType::AUTO, GaitType::TUNED}) {
        for (const MOTION_STATE mode : {MOTION_STATE::STAND, MOTION_STATE::WALK}) {
            VirtualRobot robot;
            standUp(robot);
            TEST_ASSERT_TRUE(robot.setGait(gait));
            robot.setMode(mode);
            const CommandMsg stick {1, 0, 0, 0, 0, 1, 0, 0};
            watch(robot, ticksFor(1.5f), mode == MOTION_STATE::WALK ? &stick : nullptr);
            // AUTO walks this stick, the full one sideways, in BI_GATE, so the switch shows.
            if (mode == MOTION_STATE::WALK) TEST_ASSERT_NOT_EQUAL(GaitType::TRI_GATE, robot.gait());
            GaitPushes gaits;
            robot.setStance(Stance::CRAB);
            bool tunedOnCrab = false;
            const Watch w = watch(robot, ticksFor(5.0f), mode == MOTION_STATE::WALK ? &stick : nullptr, [&](int) {
                tunedOnCrab |= robot.gait() == GaitType::TUNED && robot.feetStance() == Stance::CRAB;
            });
            char msg[48];
            std::snprintf(msg, sizeof(msg), "gait %d in mode %d", (int)gait, (int)mode);
            TEST_ASSERT_FALSE_MESSAGE(tunedOnCrab, msg);
            TEST_ASSERT_EQUAL_MESSAGE(1, gaits.heard.size(), msg);
            TEST_ASSERT_EQUAL_MESSAGE(GaitType::TRI_GATE, gaits.heard[0], msg);
            TEST_ASSERT_EQUAL_MESSAGE(GaitType::TRI_GATE, robot.gait(), msg);
            TEST_ASSERT_EQUAL_MESSAGE(Stance::CRAB, robot.stance(), msg);
            TEST_ASSERT_EQUAL_MESSAGE(Stance::CRAB, robot.feetStance(), msg);
            TEST_ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE(LEG_CLEARANCE_MM, w.leastClearance, msg);
            if (mode == MOTION_STATE::WALK) TEST_ASSERT_TRUE_MESSAGE(gaitIsCommanded(robot.gaitState()), msg);
        }
    }
}

// In the crab, and while the feet still stand on it or step from it, AUTO and TUNED are refused, published nowhere and
// answered with the gait in force, and a goal in AUTO is refused as a style the robot cannot walk; the classic gaits
// are taken.
void test_auto_and_tuned_are_refused_while_the_crab_stands() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    // Refused already while the crab waits for the control task's next tick.
    TEST_ASSERT_FALSE(robot.setGait(GaitType::AUTO));
    watch(robot, ticksFor(3.0f));
    GaitPushes gaits;
    for (const GaitType gait : {GaitType::AUTO, GaitType::TUNED}) {
        TEST_ASSERT_FALSE(robot.setGait(gait));
        watch(robot, 2);
        TEST_ASSERT_EQUAL(GaitType::TRI_GATE, robot.gait());
    }
    // Each refusal is answered with the gait in force, the refused one published nowhere.
    TEST_ASSERT_EQUAL(2, gaits.heard.size());
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, gaits.heard[0]);
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, gaits.heard[1]);
    TEST_ASSERT_EQUAL_INT(422, robot.goalMove({GoalKind::SIDEWAYS, 0.2f, 0, 0, 0.5f}, posture(0, 0), GaitType::AUTO));
    TEST_ASSERT_TRUE(robot.setGait(GaitType::WAVE));
    watch(robot, 2);
    TEST_ASSERT_EQUAL(GaitType::WAVE, robot.gait());

    // Back to the default stance asked for while idle: the feet still stand on the crab until a stand steps them off.
    robot.setMode(MOTION_STATE::IDLE);
    watch(robot, 2);
    robot.setStance(Stance::DEFAULT);
    watch(robot, 2);
    TEST_ASSERT_FALSE(robot.setGait(GaitType::TUNED));
    robot.setMode(MOTION_STATE::STAND);
    watch(robot, ticksFor(3.0f));
    TEST_ASSERT_EQUAL(Stance::DEFAULT, robot.feetStance());
    TEST_ASSERT_TRUE(robot.setGait(GaitType::TUNED));
    watch(robot, 2);
    TEST_ASSERT_EQUAL(GaitType::TUNED, robot.gait());
}

// A gait the crab refuses that reaches the motion anyway, sent before a crab was taken, is refused there too, and the
// gait in force published again so the gait topic reports it.
void test_a_refused_gait_on_the_bus_republishes_the_gait_in_force() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    watch(robot, ticksFor(3.0f));
    GaitPushes gaits;
    EventBus<GaitMsg>::publish({GaitType::AUTO});
    watch(robot, 2);
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, robot.gait());
    TEST_ASSERT_EQUAL(2, gaits.heard.size());
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, gaits.heard[1]);
    robot.setMode(MOTION_STATE::WALK);
    const CommandMsg stick = posture(0, 0, 0.5f);
    watch(robot, ticksFor(1.0f), &stick);
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, robot.gait());
}

// WALK_NN walks the stance its policy was trained on and takes its feet in one tick, so it is refused in the crab, and
// the crab in WALK_NN; each refusal is answered with the value in force on its topic, so a client that showed the
// request comes back to the robot's.
void test_walk_nn_and_the_crab_exclude_each_other() {
    {
        VirtualRobot robot;
        standUp(robot);
        robot.setStance(Stance::CRAB);
        watch(robot, ticksFor(3.0f));
        std::vector<MOTION_STATE> applied;
        auto listener = EventBus<ModeMsg>::subscribe([&applied](const ModeMsg &m) {
            if (m.kind == ModeMsgKind::APPLIED) applied.push_back(m.mode);
        });
        robot.setMode(MOTION_STATE::WALK_NN);
        watch(robot, 2);
        TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
        TEST_ASSERT_EQUAL(1, applied.size());
        TEST_ASSERT_EQUAL(MOTION_STATE::STAND, applied[0]);
    }
    {
        VirtualRobot robot;
        standUp(robot);
        robot.setMode(MOTION_STATE::WALK_NN);
        watch(robot, 2);
        StancePushes pushes;
        robot.setStance(Stance::CRAB);
        watch(robot, ticksFor(1.0f));
        TEST_ASSERT_EQUAL(MOTION_STATE::WALK_NN, robot.mode());
        TEST_ASSERT_EQUAL(Stance::DEFAULT, robot.stance());
        // The refusal is answered with the stance in force.
        TEST_ASSERT_EQUAL(1, pushes.heard.size());
        TEST_ASSERT_EQUAL(Stance::DEFAULT, pushes.heard[0]);

        // An animation borrowed from WALK_NN hands back to it, so the crab is refused there too.
        robot.animationPlay(playOf("fx_single"));
        watch(robot, ticksFor(0.5f));
        TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, robot.mode());
        robot.setStance(Stance::CRAB);
        watch(robot, 2);
        TEST_ASSERT_EQUAL(Stance::DEFAULT, robot.stance());
        TEST_ASSERT_EQUAL(2, pushes.heard.size());
        TEST_ASSERT_EQUAL(Stance::DEFAULT, pushes.heard[1]);
    }
}

// The factory calibration with one coxa centred elsewhere, as the robot's own coxas read 291 to 338 counts, and turning
// the way `direction` says.
ServoSettings coxaCentredAt(int servo, float centerPwm, float direction = 1.0f) {
    ServoSettings settings = ServoSettings_defaults();
    settings.servos[servo].center_pwm = centerPwm;
    settings.servos[servo].direction = direction;
    return settings;
}

// A corner coxa whose calibration leaves it less than the crab's largest angle on the side the crab turns it refuses
// the crab, answered with the stance in force. Legs 2 and 3 rest at +45 deg in the crab, legs 0 and 5 at -45. At 2
// counts a degree within the safe 103 to 512 counts, a coxa centred at 400 counts reaches +56 deg and -90, one centred
// at 212 +90 and -54.5, both short of the crab's 61 on one side, and a coxa turning the other way swaps the two
// sides; the robot's own centres, 291 to 338 counts, reach at least 85.5 deg either way.
void test_the_crab_is_refused_where_a_corner_coxa_cannot_turn_out_to_it() {
    struct Case {
        int servo;
        float centre;
        float direction;
        Stance taken;
    };
    for (const Case c : {Case {6, 400.0f, 1.0f, Stance::DEFAULT}, Case {6, 306.0f, 1.0f, Stance::CRAB},
                         Case {0, 400.0f, 1.0f, Stance::CRAB}, Case {0, 212.0f, 1.0f, Stance::DEFAULT},
                         Case {9, 400.0f, 1.0f, Stance::DEFAULT}, Case {15, 212.0f, 1.0f, Stance::DEFAULT},
                         Case {6, 400.0f, -1.0f, Stance::CRAB}, Case {6, 212.0f, -1.0f, Stance::DEFAULT},
                         Case {6, 338.0f, 1.0f, Stance::CRAB}, Case {0, 291.0f, 1.0f, Stance::CRAB}}) {
        VirtualRobot robot;
        robot.calibrate(coxaCentredAt(c.servo, c.centre, c.direction));
        standUp(robot);
        StancePushes pushes;
        robot.setStance(Stance::CRAB);
        watch(robot, ticksFor(3.0f));
        char msg[64];
        std::snprintf(msg, sizeof(msg), "servo %d centred at %.0f, direction %+.0f", c.servo, c.centre, c.direction);
        TEST_ASSERT_EQUAL_MESSAGE(c.taken, robot.stance(), msg);
        TEST_ASSERT_EQUAL_MESSAGE(1, pushes.heard.size(), msg);
        TEST_ASSERT_EQUAL_MESSAGE(c.taken, pushes.heard[0], msg);
        assertFeetOn(robot, c.taken, 0.0f, msg);
    }
}

// A calibration that arrives while the crab is in force and leaves a corner coxa short of it steps the feet back to the
// default stance, as a client's DEFAULT does: without dragging them, published once.
void test_a_calibration_the_crab_cannot_walk_steps_the_feet_back_to_the_default_stance() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    watch(robot, ticksFor(3.0f));
    TEST_ASSERT_EQUAL(Stance::CRAB, robot.stance());
    StancePushes pushes;
    robot.calibrate(coxaCentredAt(6, 400.0f));
    const Watch w = watch(robot, ticksFor(3.0f));
    TEST_ASSERT_EQUAL(Stance::DEFAULT, robot.stance());
    TEST_ASSERT_EQUAL(1, pushes.heard.size());
    TEST_ASSERT_EQUAL(Stance::DEFAULT, pushes.heard[0]);
    assertFeetOn(robot, Stance::DEFAULT, 0.0f, "back on the default stance");
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(LEG_CLEARANCE_MM, w.leastClearance);
    TEST_ASSERT_LESS_THAN_FLOAT(PLANTED_SLIDE_MM, w.largestSlide);
}

// A goal judged in a crab posted but not yet taken, which the control task then refuses for a coxa's travel, does not
// walk: its style was judged in the crab, and the feet stand on the default stance.
void test_a_goal_judged_in_a_crab_refused_for_travel_does_not_walk() {
    VirtualRobot robot;
    robot.calibrate(coxaCentredAt(6, 400.0f));
    standUp(robot);
    robot.setStance(Stance::CRAB);
    const CommandMsg style {0, 0, 0, 0, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL(202, robot.goalMove({GoalKind::SIDEWAYS, 0.2f, 0, 0, 0.5f}, style, GaitType::TRI_GATE));
    watch(robot, ticksFor(1.0f));
    TEST_ASSERT_EQUAL(Stance::DEFAULT, robot.stance());
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_REFUSED, robot.goalStatus().state);
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
    assertFeetOn(robot, Stance::DEFAULT, 0.0f, "the feet never walked");
}

// CRAB_COXA_MAX_DEG is the largest angle the crab walks a corner coxa to: its forward and turning strokes, alone and
// combined with each other and with its sideways one, at full stick and at the speed-optimal and the longest stride, in
// every classic timing at the neutral posture and the corners of the posture cube, reach it and go no further.
void test_the_crab_walks_its_corner_coxas_out_to_its_largest_angle_and_no_further() {
    const float sticks[][3] = {{0, 1, 0},  {0, -1, 0}, {0, 0, 1},  {0, 0, -1}, {0, 1, 1},
                               {0, 1, -1}, {1, 0, 1},  {1, 0, -1}, {1, 1, 0},  {1, -1, 0}};
    float largest = 0.0f;
    for (const GaitType gait : CLASSIC_GAITS)
        for (const auto &stick : sticks)
            for (const float stride : {0.0f, 1.0f})
                for (int corner = 0; corner < 9; ++corner) {
                    const float h = corner == 8 ? 0.0f : (corner & 1 ? 1.0f : -1.0f);
                    const float fd = corner == 8 ? 0.0f : (corner & 2 ? 1.0f : -1.0f);
                    const float s1 = corner == 8 ? 0.0f : (corner & 4 ? 1.0f : -1.0f);
                    VirtualRobot robot;
                    robot.setGait(gait);
                    standUp(robot);
                    robot.setStance(Stance::CRAB);
                    CommandMsg held {0, 0, 0, 0, h, 0, s1, fd};
                    watch(robot, ticksFor(3.0f), &held, [](int) {});
                    robot.setMode(MOTION_STATE::WALK);
                    CommandMsg walk = held;
                    walk.s = 1.0f;
                    walk.stride = stride;
                    walk.lx = stick[0];
                    walk.ly = stick[1];
                    walk.rx = stick[2];
                    watch(robot, ticksFor(5.0f), &walk, [&](int) {
                        for (const int leg : {0, 2, 3, 5})
                            largest = std::max(largest, std::fabs(robot.angles()[leg * 3]));
                    });
                }
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(CRAB_COXA_MAX_DEG + 0.01f, largest);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(CRAB_COXA_MAX_DEG - 0.5f, largest);
}

// An animation plays on the stance in force: from the crab its legs move from the crab's feet, so entering it slides
// no foot the 83.5 mm from a crab's corner foot to the default stance's, and the stance stays the crab, to whose feet
// the clip's exit brings the legs back.
void test_an_animation_from_the_crab_plays_on_the_crabs_feet() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    watch(robot, ticksFor(3.0f));
    StancePushes pushes;
    robot.animationPlay(playOf("fx_single"));
    const Watch played = watch(robot, ticksFor(1.5f));
    TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, robot.mode());
    float crab[6][4];
    stanceFeet(Stance::CRAB, feetScale(0.0f), crab);
    for (const int leg : {0, 1, 2, 4, 5}) {
        TEST_ASSERT_FLOAT_WITHIN(0.5f, crab[leg][0], robot.bodyState().feet[leg][0]);
        TEST_ASSERT_FLOAT_WITHIN(0.5f, crab[leg][1], robot.bodyState().feet[leg][1]);
    }
    robot.animationStop();
    const Watch left = watch(robot, ticksFor(1.5f));
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, robot.mode());
    assertFeetOn(robot, Stance::CRAB, 0.0f, "after the clip");
    TEST_ASSERT_EQUAL(Stance::CRAB, robot.stance());
    TEST_ASSERT_EQUAL(0, pushes.heard.size());
    TEST_ASSERT_LESS_THAN_FLOAT(PLANTED_SLIDE_MM, played.largestSlide);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(LEG_CLEARANCE_MM, std::min(played.leastClearance, left.leastClearance));
}

// The crab walks sideways in every classic timing at the walkable extremes of its posture, at every stride, with every
// leg pair apart on every tick.
void test_the_crab_walks_sideways_with_the_legs_apart() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (const std::array<float, 3> &p :
             {std::array<float, 3> {0, 0, 0}, {-1, -1, -1}, {1, 1, 1}, {1, -1, 1}, {-1, 1, -1}}) {
            for (const float stride : {-1.0f, 0.0f, 1.0f}) {
                for (const float lx : {-1.0f, 1.0f}) {
                    VirtualRobot robot;
                    standUp(robot);
                    robot.setGait(gait);
                    robot.setStance(Stance::CRAB);
                    watch(robot, ticksFor(3.0f));
                    robot.setMode(MOTION_STATE::WALK);
                    const CommandMsg stick {lx, 0, 0, 0, p[1], 1, p[0], p[2], stride};
                    const Watch w = watch(robot, ticksFor(4.0f), &stick);
                    char msg[80];
                    std::snprintf(msg, sizeof(msg), "gait %d s1 %.0f h %.0f fd %.0f stride %.0f lx %.0f", (int)gait,
                                  p[0], p[1], p[2], stride, lx);
                    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT_MESSAGE(LEG_CLEARANCE_MM, w.leastClearance, msg);
                    TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(GAIT_STRIDE_DEADBAND_MM, std::fabs(robot.gaitState().step_x),
                                                           msg);
                }
            }
        }
    }
}

// A goal in AUTO and a crab in the same mail: the crab, posted after the goal, ends it, and the robot never walks AUTO
// on the crab's feet.
void test_a_goal_in_auto_in_the_crabs_mail_never_walks_auto_on_the_crab() {
    VirtualRobot robot;
    standUp(robot);
    TEST_ASSERT_EQUAL_INT(202, robot.goalMove({GoalKind::FORWARD, 1.0f, 0, 0, 1.0f}, posture(0, 0), GaitType::AUTO));
    robot.setStance(Stance::CRAB);
    float leastClearance = INFINITY;
    bool strokedOnCrab = false;
    for (int t = 0; t < ticksFor(5.0f); ++t) {
        robot.tick();
        leastClearance = std::min(leastClearance, legClearance(robot.angles()));
        strokedOnCrab |= robot.feetStance() == Stance::CRAB && gaitIsCommanded(robot.gaitState());
    }
    TEST_ASSERT_FALSE(strokedOnCrab);
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(LEG_CLEARANCE_MM, leastClearance);
    TEST_ASSERT_TRUE(robot.goalStatus().state == socket_message_GoalState_GOAL_CANCELLED ||
                     robot.goalStatus().state == socket_message_GoalState_GOAL_REFUSED);
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, robot.gait());
}

// A goal walks the stance in force, judged there when it is accepted and walked there: a style the default stance walks
// and the crab does not is clamped into the crab's walkable region and reaches its target.
void test_a_goal_in_the_crab_clamps_its_style_into_the_crab() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    watch(robot, ticksFor(3.0f));
    // s1 -1, h 0.5, fd 0: walkable in the default stance, not in the crab (simulation's envelope, both tables).
    TEST_ASSERT_TRUE(postureWalkable(-1.0f, 0.5f, 0.0f, Stance::DEFAULT));
    TEST_ASSERT_FALSE(postureWalkable(-1.0f, 0.5f, 0.0f, Stance::CRAB));
    const CommandMsg style {0, 0, 0, 0, 0.5f, 0, -1.0f, 0.0f, 0};
    TEST_ASSERT_EQUAL_INT(202, robot.goalMove({GoalKind::SIDEWAYS, 0.2f, 0, 0, 0.5f}, style, GaitType::TRI_GATE));
    for (int t = 0; t < ticksFor(30.0f) && robot.goalStatus().state != socket_message_GoalState_GOAL_REACHED &&
                    robot.goalStatus().state != socket_message_GoalState_GOAL_TIMED_OUT;
         ++t)
        robot.tick();
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_REACHED, robot.goalStatus().state);
}

// A crab goal's top speed is the crab's sideways envelope at its posture, here 394 mm/s where the default stance's is
// 237 (TRI, stride +1, s1 -1, h -0.5, fd -1), and its approach, its stroke capped to what remains, walks the crab's
// capped stroke and cadence (withStrokeCap on the crab's tables), which the default stance's tables would not give.
void test_a_crab_goal_walks_the_crabs_envelope() {
    VirtualRobot robot;
    standUp(robot);
    robot.setStance(Stance::CRAB);
    watch(robot, ticksFor(3.0f));
    const float s1 = -1.0f, h = -0.5f, fd = -1.0f, stride = 1.0f;
    const float crab = envelopeAt(GaitType::TRI_GATE, 1, s1, h, fd, stride, Stance::CRAB).vMax;
    const float other = envelopeAt(GaitType::TRI_GATE, 1, s1, h, fd, stride, Stance::DEFAULT).vMax;
    TEST_ASSERT_GREATER_THAN_FLOAT(100.0f, crab - other);
    const CommandMsg style {0, 0, 0, 0, h, 0, s1, fd, stride};
    TEST_ASSERT_EQUAL_INT(202, robot.goalMove({GoalKind::SIDEWAYS, 0.3f, 0, 0, 1.0f}, style, GaitType::TRI_GATE));
    float topSpeed = 0, worst = 0, apart = 0;
    int capped = 0;
    for (int t = 0; t < ticksFor(30.0f) && robot.goalStatus().state != socket_message_GoalState_GOAL_REACHED; ++t) {
        robot.tick();
        if (robot.goalStatus().state != socket_message_GoalState_GOAL_RUNNING) continue;
        topSpeed = robot.goalTopSpeed();
        const CommandMsg &stick = robot.walkCommand();
        const WalkTarget w = classicWalkTarget(GaitType::TRI_GATE, stick.lx, stick.ly, stick.rx, stick.s, s1, h, fd,
                                               stride, Stance::CRAB);
        const StrokeCap cap = robot.goalStrokeCap();
        if (!(std::fabs(w.stepX) > cap.limit)) continue;
        ++capped;
        const WalkTarget mine = withStrokeCap(GaitType::TRI_GATE, w, cap.axis, cap.limit, s1, h, fd, Stance::CRAB);
        const WalkTarget theirs = withStrokeCap(GaitType::TRI_GATE, w, cap.axis, cap.limit, s1, h, fd, Stance::DEFAULT);
        const gait_state_t &c = robot.gaitCommand();
        worst = std::max({worst, std::fabs(c.step_x - mine.stepX), std::fabs(c.phase_rate - mine.phaseRate)});
        apart = std::max(apart, std::fabs(theirs.phaseRate - mine.phaseRate) / mine.phaseRate);
    }
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_REACHED, robot.goalStatus().state);
    TEST_ASSERT_FLOAT_WITHIN(0.001f * crab, crab, topSpeed * 1000.0f);
    TEST_ASSERT_GREATER_THAN_INT(0, capped);
    TEST_ASSERT_LESS_THAN_FLOAT(1e-4f, worst);
    // The two stances' tables cap this walk apart, so the walk shows which it read.
    TEST_ASSERT_GREATER_THAN_FLOAT(0.05f, apart);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_the_crab_stands_every_leg_straight_out_at_every_posture);
    RUN_TEST(test_a_stance_change_in_a_stand_steps_the_feet_without_dragging_them);
    RUN_TEST(test_a_stance_asked_for_in_idle_is_stepped_into_by_the_stand);
    RUN_TEST(test_a_stance_change_mid_walk_stops_the_walk_first);
    RUN_TEST(test_a_walk_from_a_stand_at_rest_in_another_stance_settles_into_it_first);
    RUN_TEST(test_commands_the_handheld_and_a_goal_leave_the_stance_as_it_is);
    RUN_TEST(test_a_stance_change_ends_a_goal_and_a_repeat_does_not);
    RUN_TEST(test_the_stance_is_published_once_per_change);
    RUN_TEST(test_a_goal_in_auto_in_the_crabs_mail_never_walks_auto_on_the_crab);
    RUN_TEST(test_a_goal_in_the_crab_clamps_its_style_into_the_crab);
    RUN_TEST(test_a_crab_goal_walks_the_crabs_envelope);
    RUN_TEST(test_the_crab_in_auto_or_tuned_selects_tri_gate_first);
    RUN_TEST(test_auto_and_tuned_are_refused_while_the_crab_stands);
    RUN_TEST(test_a_refused_gait_on_the_bus_republishes_the_gait_in_force);
    RUN_TEST(test_walk_nn_and_the_crab_exclude_each_other);
    RUN_TEST(test_an_animation_from_the_crab_plays_on_the_crabs_feet);
    RUN_TEST(test_the_crab_walks_sideways_with_the_legs_apart);
    RUN_TEST(test_the_crab_is_refused_where_a_corner_coxa_cannot_turn_out_to_it);
    RUN_TEST(test_a_goal_judged_in_a_crab_refused_for_travel_does_not_walk);
    RUN_TEST(test_a_calibration_the_crab_cannot_walk_steps_the_feet_back_to_the_default_stance);
    RUN_TEST(test_the_crab_walks_its_corner_coxas_out_to_its_largest_angle_and_no_further);
    return UNITY_END();
}
