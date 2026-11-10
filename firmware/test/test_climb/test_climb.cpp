// The climb walk (gait_climb.h, MotionService's climb posture) on the virtual robot: a classic walk asked to climb
// settles into the climb's lift on its raised body between steps, walks within joint travel and the servos' speed in
// every classic gait, and leaves the climb the same way.

#define USE_POLICY 1
#define USE_MPU6050 1

#include <unity.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <gait_climb.h>
#include <gait_speed.h>
#include <virtual_robot.h>

namespace {

constexpr int TICKS_PER_SECOND = 200;
constexpr int COMMAND_PERIOD_TICKS = 4; // the app's 20 ms stick period
constexpr float STEPPING_DEG_PER_TICK = 0.05f;
// The servo's 13.1 rad/s over a 5 ms tick is 3.75 deg; the classic walks peak at about 3.9 deg (test_virtual_robot),
// since the envelope reads the joint rate at sampled points of the curves.
constexpr float SERVO_STEP_BOUND_DEG = 4.0f;
// The walk's first steps in the climb run above it once: BI turning at full stick moves the tibias of legs 2 and 3
// 4.37 deg in a tick 0.21-0.25 s after its cycle starts, inside the first swing of those legs, which BI's offsets put
// mid-swing at the start, so gait.h replays that swing over the time left to it; the steady walk peaks at 3.38 deg on
// the same stick. Measured 2026-11-08; the bound guards against more.
constexpr float START_STEP_BOUND_DEG = 4.5f;
// The classic lift of a stick whose s1 is 0, (s1 + 1) * 20 mm.
constexpr float CLASSIC_LIFT_MM = 20.0f;
constexpr GaitType CLASSIC_GAITS[4] = {GaitType::TRI_GATE, GaitType::BI_GATE, GaitType::WAVE, GaitType::RIPPLE};

int ticksFor(float seconds) { return static_cast<int>(std::lround(seconds * TICKS_PER_SECOND)); }

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
            if (ServoController::withinTravel(j, angle) != angle) run.clamped++;
            run.largestStep = std::max(run.largestStep, std::fabs(angle - previous[j]));
            previous[j] = angle;
        }
    }
    return run;
}

HeldRun &operator+=(HeldRun &run, const HeldRun &more) {
    run.clamped += more.clamped;
    run.largestStep = std::max(run.largestStep, more.largestStep);
    return run;
}

void walkIn(VirtualRobot &robot, GaitType gait) {
    robot.setMode(MOTION_STATE::STAND);
    hold(robot, CommandMsg {}, ticksFor(1.0f));
    robot.setGait(gait);
    robot.tick();
    robot.setMode(MOTION_STATE::WALK);
}

CommandMsg climbing(CommandMsg c) {
    c.climb = true;
    return c;
}

// lx, ly, rx at s = +1, the fastest the stick can ask for.
CommandMsg fullStick(float lx, float ly, float rx) { return {lx, ly, rx, 0, 0, 1, 0, 0, 0}; }

const CommandMsg FULL_STICKS[] = {fullStick(0, 1, 0), fullStick(0, -1, 0), fullStick(1, 0, 0), fullStick(-1, 0, 0),
                                  fullStick(0, 0, 1), fullStick(0, 0, -1), fullStick(1, 1, 1), fullStick(-1, -1, -1)};

// The walk is driving a stroke at a cadence, not only stepping a stance change in.
bool walkResumed(VirtualRobot &robot) {
    const gait_state_t &target = robot.gaitCommand();
    return target.phase_rate > 0.0f && std::hypot(target.step_x, target.step_y) + std::fabs(target.step_angle) > 0.0f;
}

// The body stands at the climb's height and the gait steps at its lift.
bool inClimbPosture(VirtualRobot &robot) {
    return std::fabs(robot.bodyState().zm - bodyHeightMm(gait_climb::BODY_HEIGHT)) < 0.5f &&
           std::fabs(robot.gaitState().step_height - gait_climb::LIFT_MM) < 0.5f;
}

void describe(char *msg, size_t size, GaitType gait, const CommandMsg &c, const HeldRun &run) {
    std::snprintf(msg, size, "gait %d stick (%.0f %.0f %.0f): %d clamped, largest step %.2f deg", (int)gait, c.lx, c.ly,
                  c.rx, run.clamped, run.largestStep);
}

} // namespace

void setUp() { host_bus::resetAll(); }

void tearDown() {}

// The climb's envelope was measured at its posture as the classic one at a grid point, so no full stick, alone or
// combined, asks a joint for more than its travel or, walking steadily, the servo for more than its speed, in any
// classic gait; the settle into the posture and the walk's first steps keep the travel too.
void test_every_full_stick_climbs_within_travel_and_the_servos_speed() {
    for (const GaitType gait : CLASSIC_GAITS) {
        for (const CommandMsg &stick : FULL_STICKS) {
            VirtualRobot robot;
            walkIn(robot, gait);
            const HeldRun start = hold(robot, climbing(stick), ticksFor(5.0f));
            const HeldRun steady = hold(robot, climbing(stick), ticksFor(3.0f));
            char msg[160];
            describe(msg, sizeof(msg), gait, stick, start);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, start.clamped, msg);
            TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(START_STEP_BOUND_DEG, start.largestStep, msg);
            describe(msg, sizeof(msg), gait, stick, steady);
            TEST_ASSERT_TRUE_MESSAGE(inClimbPosture(robot), msg);
            TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
            TEST_ASSERT_EQUAL_INT_MESSAGE(0, steady.clamped, msg);
            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(STEPPING_DEG_PER_TICK, steady.largestStep, msg);
            TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(SERVO_STEP_BOUND_DEG, steady.largestStep, msg);
        }
    }
}

// A walk asked to climb at full stick stops, settles into the climb's posture and walks on in it; asked to stop
// climbing, it settles back into its sliders' posture the same way. No joint leaves its travel on the way.
void test_a_walk_settles_into_the_climb_and_out_of_it_between_steps() {
    for (const GaitType gait : CLASSIC_GAITS) {
        VirtualRobot robot;
        walkIn(robot, gait);
        const CommandMsg stick = fullStick(0, 1, 0);
        HeldRun run = hold(robot, stick, ticksFor(2.0f));
        TEST_ASSERT_TRUE(walkResumed(robot));
        TEST_ASSERT_EQUAL_FLOAT(CLASSIC_LIFT_MM, robot.gaitCommand().step_height);

        run += hold(robot, climbing(stick), ticksFor(8.0f));
        char msg[160];
        describe(msg, sizeof(msg), gait, stick, run);
        TEST_ASSERT_TRUE_MESSAGE(inClimbPosture(robot), msg);
        TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);

        run += hold(robot, stick, ticksFor(8.0f));
        describe(msg, sizeof(msg), gait, stick, run);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, bodyHeightMm(0.0f), robot.bodyState().zm, msg);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.5f, CLASSIC_LIFT_MM, robot.gaitState().step_height, msg);
        TEST_ASSERT_TRUE_MESSAGE(walkResumed(robot), msg);
        TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
    }
}

// The climb walks the climb's envelope, not the classic one at the sliders: the posture sliders a climbing command
// carries move nothing.
void test_a_climbing_walk_follows_the_climb_envelope_whatever_the_sliders() {
    VirtualRobot robot;
    walkIn(robot, GaitType::WAVE);
    CommandMsg stick = climbing(fullStick(0, 1, 0));
    hold(robot, stick, ticksFor(8.0f));
    const WalkTarget expected = climbWalkTarget(GaitType::WAVE, 0, 1, 0, 1, 0);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, expected.phaseRate);
    TEST_ASSERT_EQUAL_FLOAT(expected.stepY, robot.gaitCommand().step_y);
    TEST_ASSERT_EQUAL_FLOAT(expected.phaseRate, robot.gaitCommand().phase_rate);

    stick.h = 0.5f;
    stick.s1 = -1.0f;
    stick.fd = 1.0f;
    hold(robot, stick, ticksFor(2.0f));
    TEST_ASSERT_TRUE(inClimbPosture(robot));
    TEST_ASSERT_EQUAL_FLOAT(expected.stepY, robot.gaitCommand().step_y);
}

// The climb's posture was measured in the default stance; the crab and TUNED walk their own posture when asked to
// climb.
void test_the_crab_and_tuned_walk_their_own_posture_when_asked_to_climb() {
    VirtualRobot crab;
    walkIn(crab, GaitType::TRI_GATE);
    crab.setStance(Stance::CRAB);
    hold(crab, CommandMsg {}, ticksFor(3.0f));
    TEST_ASSERT_EQUAL(Stance::CRAB, crab.stance());
    const CommandMsg sideways = climbing(fullStick(1, 0, 0));
    const HeldRun run = hold(crab, sideways, ticksFor(4.0f));
    TEST_ASSERT_EQUAL_INT(0, run.clamped);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, bodyHeightMm(0.0f), crab.bodyState().zm);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, CLASSIC_LIFT_MM, crab.gaitState().step_height);
    TEST_ASSERT_TRUE(walkResumed(crab));

    VirtualRobot tuned;
    walkIn(tuned, GaitType::TUNED);
    hold(tuned, climbing(fullStick(0, 1, 0)), ticksFor(4.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -tuned_gait::RIDE_MM, tuned.bodyState().zm);
}

// AUTO climbs on WAVE, the classic gait that stands on the most feet, and walks AUTO's own choice again after the
// climb.
void test_auto_climbs_on_wave() {
    VirtualRobot robot;
    walkIn(robot, GaitType::AUTO);
    const CommandMsg stick = fullStick(0, 1, 0);
    hold(robot, stick, ticksFor(2.0f));
    TEST_ASSERT_EQUAL(GaitType::BI_GATE, robot.gait());
    hold(robot, climbing(stick), ticksFor(8.0f));
    TEST_ASSERT_EQUAL(GaitType::WAVE, robot.gait());
    TEST_ASSERT_TRUE(inClimbPosture(robot));
    TEST_ASSERT_TRUE(walkResumed(robot));
    hold(robot, stick, ticksFor(8.0f));
    TEST_ASSERT_EQUAL(GaitType::BI_GATE, robot.gait());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_every_full_stick_climbs_within_travel_and_the_servos_speed);
    RUN_TEST(test_a_walk_settles_into_the_climb_and_out_of_it_between_steps);
    RUN_TEST(test_a_climbing_walk_follows_the_climb_envelope_whatever_the_sliders);
    RUN_TEST(test_the_crab_and_tuned_walk_their_own_posture_when_asked_to_climb);
    RUN_TEST(test_auto_climbs_on_wave);
    return UNITY_END();
}
