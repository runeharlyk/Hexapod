// AUTO changing gait while it walks (gait_morph.h), on the firmware's Hexapod run off the robot: feet, joint angles and
// servo steps as the robot would command them.

#define ANIMATION_DIRECTORY "animations/fixtures"
#define USE_MPU6050 1

#include <unity.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <gait_morph.h>
#include <imu_frame.h>
#include <virtual_robot.h>
#include <event_bus.h>

namespace {

constexpr int TICKS_PER_SECOND = 200;
constexpr int COMMAND_PERIOD_TICKS = 4; // the app's 20 ms stick period
// The classic walks peak at 3.93 deg in a tick (test_virtual_robot's STEP_BOUND_DEG); the servo turns 3.75.
constexpr float STEP_BOUND_DEG = 4.0f;
constexpr float GROUND_Z = -66.0f; // DEFAULT_STANCE's foot height, where every foot on the floor stands
// The feet the patterns stand on: the wave family's at least TRI's three, BI's two.
constexpr int FAMILY_FEET = 3;
constexpr int BI_FEET = 2;
// The step to BI comes at the stick's share triMax / span (gait_morph.h), 0.70 forward at full speed; 0.65 is clear of
// it, and a forward stick of 0.85 or more walks BI.
constexpr float FAMILY_TOP_STICK = 0.65f;

int ticksFor(float seconds) { return static_cast<int>(std::lround(seconds * TICKS_PER_SECOND)); }

CommandMsg stick(float lx, float ly, float rx) { return {lx, ly, rx, 0, 0, 1, 0, 0}; }
CommandMsg forward(float ly) { return stick(0, ly, 0); }

bool grounded(const float (&foot)[4]) { return foot[2] <= GROUND_Z + 1e-4f; }

// What the robot did over a run of ticks, observed after each.
class Run {
  public:
    int fewestDown = 6;                          // the fewest feet on the floor in a tick
    float largestStep = 0.0f;                    // the largest joint step in a tick, in degrees
    int clamped = 0;                             // joint commands beyond travel
    float slip = 0.0f;                           // the most a floor foot moved unlike the others in a tick, in mm
    int touchdown[6] = {-1, -1, -1, -1, -1, -1}; // each foot's last landing, in ticks

    explicit Run(const VirtualRobot &robot) {
        std::copy_n(robot.angles(), 18, _angles);
        std::copy(&robot.bodyState().feet[0][0], &robot.bodyState().feet[0][0] + 24, &_feet[0][0]);
    }

    // The slip assumes the floor feet translate alike, which a turn's do not.
    void observe(const VirtualRobot &robot, int t) {
        for (int j = 0; j < 18; ++j) {
            const float angle = robot.angles()[j];
            if (ServoController::withinTravel(j, angle) != angle) clamped++;
            largestStep = std::max(largestStep, std::fabs(angle - _angles[j]));
            _angles[j] = angle;
        }
        const auto &now = robot.bodyState().feet;
        int down = 0, both = 0;
        float moved[6][2], mean[2] = {0, 0};
        for (int leg = 0; leg < 6; ++leg) {
            if (!grounded(now[leg])) continue;
            down++;
            if (!grounded(_feet[leg])) {
                touchdown[leg] = t;
                continue;
            }
            moved[both][0] = now[leg][0] - _feet[leg][0];
            moved[both][1] = now[leg][1] - _feet[leg][1];
            mean[0] += moved[both][0];
            mean[1] += moved[both][1];
            both++;
        }
        fewestDown = std::min(fewestDown, down);
        for (int k = 0; k < both; ++k)
            slip = std::max(slip, std::hypot(moved[k][0] - mean[0] / both, moved[k][1] - mean[1] / both));
        std::copy(&now[0][0], &now[0][0] + 24, &_feet[0][0]);
    }

  private:
    float _angles[18];
    float _feet[6][4];
};

// At least `feet` stand at every tick, and the joints step within the servo's speed and their travel.
void assertSafe(const Run &run, int feet, const char *what) {
    char msg[160];
    std::snprintf(msg, sizeof(msg), "%s: down %d, step %.2f deg, clamped %d", what, run.fewestDown, run.largestStep,
                  run.clamped);
    TEST_ASSERT_GREATER_OR_EQUAL_INT_MESSAGE(feet, run.fewestDown, msg);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT_MESSAGE(STEP_BOUND_DEG, run.largestStep, msg);
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, run.clamped, msg);
}

class Walker {
  public:
    Walker() {
        robot.setMode(MOTION_STATE::STAND);
        for (int t = 0; t < ticksFor(1.0f); ++t) robot.tick();
        robot.setGait(GaitType::AUTO);
        robot.setMode(MOTION_STATE::WALK);
        for (int t = 0; t < ticksFor(1.0f); ++t) robot.tick();
    }

    // Walks `ticks` with the stick `stick(t)` sent at the app's period.
    template <typename Stick>
    Run walk(int ticks, Stick stick) {
        Run run(robot);
        for (int t = 0; t < ticks; ++t) {
            if (t % COMMAND_PERIOD_TICKS == 0) robot.command(stick(t));
            robot.tick();
            run.observe(robot, t);
        }
        return run;
    }

    Run hold(float seconds, const CommandMsg &c) {
        return walk(ticksFor(seconds), [&](int) { return c; });
    }

    VirtualRobot robot;
};

// Stood up and sent an AUTO goal at `speed` over `metres` forward, run to its end; the floor holds the feet, so the
// DMP's yaw is the turn they made (test_goal_moves' Rig).
Run autoGoal(VirtualRobot &robot, float speed, float metres) {
    robot.setMode(MOTION_STATE::STAND);
    for (int t = 0; t < ticksFor(1.0f); ++t) robot.tick();
    TEST_ASSERT_EQUAL_INT(
        202, robot.goalMove({GoalKind::FORWARD, metres, 0, 0, speed}, {0, 0, 0, 0, 0, 0, 0, 0, 0}, GaitType::AUTO));
    Run run(robot);
    for (int t = 0; t < ticksFor(30.0f); ++t) {
        const auto state = static_cast<GoalState>(robot.goalStatus().state);
        if (t > 0 && state != GoalState::SETTLING && state != GoalState::RUNNING) break;
        Peripherals::Imu imu;
        imu.rpyRad[2] = imu_frame::YAW_SIGN * robot.odometry().kinematicYaw;
        robot.setImu(imu);
        robot.tick();
        run.observe(robot, t);
    }
    return run;
}

// Each foot's landing as a phase after the right front's, from the last landings and the cadence walked.
std::array<float, 6> landingOffsets(const Run &run, float cadence) {
    std::array<float, 6> offsets;
    for (int leg = 0; leg < 6; ++leg)
        offsets[leg] = auto_morph::wrap01(-(float)(run.touchdown[leg] - run.touchdown[0]) * cadence / TICKS_PER_SECOND);
    return offsets;
}

// The family's two ends are the classic gaits' own patterns (gait.h setGait), up to a shift of every leg alike.
void test_the_wave_family_ends_on_wave_and_tri() {
    for (const GaitType gait : {GaitType::WAVE, GaitType::TRI_GATE}) {
        gait_state_t classic {};
        classic.gait_type = gait;
        GaitController().setGait(classic);
        float family[6];
        auto_morph::familyOffsets(classic.stand_frac, family);
        for (int leg = 0; leg < 6; ++leg)
            TEST_ASSERT_FLOAT_WITHIN(
                1e-5f, 0.0f, auto_morph::cyclic(family[leg] - family[0], classic.offset[leg] - classic.offset[0]));
    }
}

// A steady walk at a crawl, mid-way and near TRI's top lands each foot within a tick of the pattern the planner asks
// for: the stepper converges on the plan from a standstill.
void test_steady_walks_land_on_the_planned_pattern() {
    for (const float ly : {0.05f, 0.4f, 0.68f}) {
        Walker w;
        const Run run = w.hold(12.0f, forward(ly));
        auto_morph::Planner planner;
        const auto_morph::Plan plan = planner.plan(autoDemand(0, ly, 0, 1, 0, 0, 0, 0), 0, ly, 0, 0, 0, 0, 0);
        const float cadence = w.robot.gaitState().phase_rate;
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, plan.walk.phaseRate, cadence);
        const std::array<float, 6> landed = landingOffsets(run, cadence);
        const float tolerance = cadence / TICKS_PER_SECOND + 0.005f;
        for (int leg = 0; leg < 6; ++leg) {
            // A foot lands where its offset's negative phase comes round, so landings run against the offsets.
            const float expected = auto_morph::wrap01(plan.offsets[leg] - plan.offsets[0]);
            char msg[120];
            std::snprintf(msg, sizeof(msg), "stick %.2f duty %.3f leg %d landed at %.3f, planned %.3f", ly, plan.duty,
                          leg, landed[leg], expected);
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(tolerance, 0.0f, auto_morph::cyclic(landed[leg], expected), msg);
        }
    }
}

// The stick pushed from a standstill to the family's top and back walks every duty from WAVE to TRI and back: no foot
// on the floor moves unlike the others, no joint steps faster than the classic walks do or leaves its travel, and at
// least three feet stand at every tick.
void test_a_ramp_through_the_family_walks_without_slip_or_overspeed() {
    Walker w;
    const int ramp = ticksFor(6.0f);
    const Run run = w.walk(2 * ramp, [&](int t) {
        return forward(FAMILY_TOP_STICK * (t < ramp ? (float)t / ramp : 2.0f - (float)t / ramp));
    });
    assertSafe(run, FAMILY_FEET, "family ramp");
    TEST_ASSERT_LESS_THAN_FLOAT(0.01f, run.slip);
}

// A goal's line and heading holds add a small sideways or turning stick to its walk, and an operator walks diagonals
// and arcs. Each stands and steps as the straight walk does: an axis of small weight shortens the joints' reach by
// about its weight, not to its own, and axes of equal weight still keep the joints within their travel.
void test_mixed_sticks_in_the_family_walk_as_safely() {
    for (const CommandMsg &c : {stick(0.03f, 0.3f, 0), stick(0.03f, 0.6f, 0), stick(-0.1f, 0.55f, 0),
                                stick(0, 0.6f, 0.03f), stick(0.3f, 0.3f, 0), stick(0, 0.3f, 0.3f)}) {
        char what[64];
        std::snprintf(what, sizeof(what), "lx %.2f ly %.2f rx %.2f", c.lx, c.ly, c.rx);
        auto_morph::Planner planner;
        const auto_morph::Plan plan =
            planner.plan(autoDemand(c.lx, c.ly, c.rx, c.s, 0, 0, 0, 0), c.lx, c.ly, c.rx, 0, 0, 0, 0);
        TEST_ASSERT_NOT_EQUAL_MESSAGE(GaitType::BI_GATE, plan.label, what);
        Walker w;
        const Run run = w.hold(8.0f, c);
        assertSafe(run, FAMILY_FEET, what);
        if (c.rx == 0) TEST_ASSERT_LESS_THAN_FLOAT_MESSAGE(0.01f, run.slip, what);
    }
}

// A released stick stops the walk on the plan it walked: the feet still stepping keep their pattern while the stroke
// eases away, and come to rest on the floor. The ease home after it slides the stance feet as every classic stop does.
void test_a_released_stick_stops_on_the_plan_it_walked() {
    for (const float ly : {0.1f, 0.5f}) {
        Walker w;
        w.hold(4.0f, forward(ly));
        const Run stop = w.hold(2.0f, forward(0));
        char what[32];
        std::snprintf(what, sizeof(what), "release from %.1f", ly);
        assertSafe(stop, FAMILY_FEET, what);
        TEST_ASSERT_FALSE_MESSAGE(gaitIsCommanded(w.robot.gaitState()), what);
        for (int leg = 0; leg < 6; ++leg) TEST_ASSERT_TRUE_MESSAGE(grounded(w.robot.bodyState().feet[leg]), what);
    }
}

// An AUTO goal walks the morph with its holds, slows on its approach and stops: it reaches its target as safely as a
// stick walks, slow and at the family's top speeds, near and far.
void test_auto_goals_in_the_family_walk_the_morph_safely() {
    for (const float speed : {0.3f, 0.6f})
        for (const float metres : {0.05f, 0.3f}) {
            VirtualRobot robot;
            const Run run = autoGoal(robot, speed, metres);
            char what[48];
            std::snprintf(what, sizeof(what), "goal %.2f m at %.1f", metres, speed);
            TEST_ASSERT_EQUAL_MESSAGE(GoalState::REACHED, static_cast<GoalState>(robot.goalStatus().state), what);
            assertSafe(run, FAMILY_FEET, what);
        }
}

// A goal's capped stroke steps faster to keep its speed, up to the servo's limit at the duty the morph walks, which is
// lower than at TRI's duty, the classic gait the plan is labelled with: a swing of 1 - d takes less of the cycle.
void test_a_capped_plan_steps_within_its_own_dutys_servo_limit() {
    auto_morph::Planner planner;
    const auto_morph::Plan plan = planner.plan(autoDemand(0, 0.45f, 0, 1, 0, 0, 0, 0), 0, 0.45f, 0, 0, 0, 0, 0);
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, plan.label);
    TEST_ASSERT_GREATER_THAN_FLOAT(auto_morph::TRI_DUTY + 0.05f, plan.duty);
    const float cap = 0.3f * plan.walk.stepY;
    const auto_morph::Plan capped = auto_morph::withStrokeCap(plan, 0, cap, 0, 0, 0);
    using namespace gait_speed_detail;
    const float fraction = cap / strokeMaxAt(0, 0, 0, 0, Stance::DEFAULT);
    const Kappas kappas = kappasAt(0, locate(0), locate(0), locate(0), Stance::DEFAULT);
    const float servoLimit =
        topCadenceAtDuty(plan.duty, kappaAtStroke(kappas.swing, fraction), kappaAtStroke(kappas.stance, fraction));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, cap, capped.walk.stepY);
    TEST_ASSERT_LESS_OR_EQUAL_FLOAT(servoLimit * (1 + 1e-5f), capped.walk.phaseRate);
    // The joints' travel stays where it was: the reach grows as the stroke shrinks.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, plan.reach * plan.walk.stepY, capped.reach * capped.walk.stepY);
}

// BI under the morph: open defects, each its own test so the failing list is the list of what BI still lacks.

// A walk started at BI's speeds stands on BI's two feet from its first step.
void test_bi_starts_from_a_standstill_on_two_feet() {
    Walker w;
    assertSafe(w.hold(6.0f, forward(1.0f)), BI_FEET, "BI from a standstill");
}

// A steady BI walk stands on its two feet.
void test_a_steady_bi_walk_stands_on_two_feet() {
    Walker w;
    w.hold(4.0f, forward(0.85f));
    assertSafe(w.hold(6.0f, forward(0.85f)), BI_FEET, "steady BI");
}

// Stepping between the family and BI regroups the legs without dropping below BI's two feet or overspeeding a joint.
void test_the_step_between_the_family_and_bi_is_safe() {
    Walker up;
    up.hold(4.0f, forward(0.5f));
    assertSafe(up.hold(4.0f, forward(1.0f)), BI_FEET, "family to BI");
    Walker down;
    down.hold(4.0f, forward(1.0f));
    assertSafe(down.hold(4.0f, forward(0.5f)), BI_FEET, "BI to family");
}

// A released BI walk stops on its two feet.
void test_a_released_bi_walk_stops_on_two_feet() {
    Walker w;
    w.hold(4.0f, forward(1.0f));
    assertSafe(w.hold(2.0f, forward(0)), BI_FEET, "release from BI");
}

// A fast AUTO goal walks BI and reaches its target on two feet.
void test_a_fast_auto_goal_walks_bi_safely() {
    VirtualRobot robot;
    const Run run = autoGoal(robot, 0.9f, 0.3f);
    TEST_ASSERT_EQUAL(GoalState::REACHED, static_cast<GoalState>(robot.goalStatus().state));
    assertSafe(run, BI_FEET, "fast goal");
}

} // namespace

void setUp() { host_bus::resetAll(); }

void tearDown() { Peripherals::setImu({}); }

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_the_wave_family_ends_on_wave_and_tri);
    RUN_TEST(test_steady_walks_land_on_the_planned_pattern);
    RUN_TEST(test_a_ramp_through_the_family_walks_without_slip_or_overspeed);
    RUN_TEST(test_mixed_sticks_in_the_family_walk_as_safely);
    RUN_TEST(test_a_released_stick_stops_on_the_plan_it_walked);
    RUN_TEST(test_auto_goals_in_the_family_walk_the_morph_safely);
    RUN_TEST(test_a_capped_plan_steps_within_its_own_dutys_servo_limit);
    RUN_TEST(test_bi_starts_from_a_standstill_on_two_feet);
    RUN_TEST(test_a_steady_bi_walk_stands_on_two_feet);
    RUN_TEST(test_the_step_between_the_family_and_bi_is_safe);
    RUN_TEST(test_a_released_bi_walk_stops_on_two_feet);
    RUN_TEST(test_a_fast_auto_goal_walks_bi_safely);
    return UNITY_END();
}
