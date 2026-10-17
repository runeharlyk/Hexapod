// Host tests of the goal controller: scripted poses for the stick laws, and a simple body that walks the sticks for
// the reach, the settle and the corrective passes.

#include <unity.h>
#include <algorithm>
#include <cmath>
#include <goal_controller.h>
#include <initializer_list>
#include <vector>

namespace {

constexpr float DT = 0.005f;
constexpr float TOP_SPEED_MM = 120.0f; // a full forward stick, mm/s
constexpr float TOP_TURN = 1.2f;       // a full turn stick, rad/s
constexpr float PI = static_cast<float>(M_PI);
constexpr float TWO_PI = 2.0f * PI;
constexpr float DEG = PI / 180.0f;
constexpr float LAG_S = 0.1f; // the command filter's time constant

Goal forward(float distance, float speed) { return {GoalKind::FORWARD, distance, 0, 0, speed}; }
Goal sideways(float distance, float speed) { return {GoalKind::SIDEWAYS, distance, 0, 0, speed}; }
Goal turn(float angle, float speed) { return {GoalKind::TURN, 0, angle, 0, speed}; }
Goal arc(float radius, float angle, float speed) { return {GoalKind::ARC, 0, angle, radius, speed}; }

OdometryEstimate at(float x, float y, float yaw, float distance = 0) {
    return {.x = x, .y = y, .yaw = std::remainder(yaw, TWO_PI), .kinematicYaw = 0, .distance = distance};
}

bool centred(const GoalSticks& sticks) { return sticks.lx == 0 && sticks.ly == 0 && sticks.rx == 0; }

bool active(const GoalController& goal) {
    return goal.state() == GoalState::RUNNING || goal.state() == GoalState::SETTLING;
}

// The pose of `p` in the frame faced at `origin`: x right, y forward.
struct Local {
    float right, forward;
};
Local local(const OdometryEstimate& origin, const OdometryEstimate& p) {
    const float dx = p.x - origin.x, dy = p.y - origin.y, c = std::cos(origin.yaw), s = std::sin(origin.yaw);
    return {c * dx + s * dy, -s * dx + c * dy};
}

// Where an arc of `radius` through `angle` (counter-clockwise positive, curving left) ends, in its start frame.
Local arcEnd(float radius, float angle) {
    return {-std::copysign(radius * (1 - std::cos(angle)), angle), radius * std::sin(std::fabs(angle))};
}

/**
 * A body that walks the sticks as MotionService drives the gait: the combined stick is L1-normalized, the walk follows
 * it through a first-order lag, and the gait steps at `cadence`. Centred sticks stop it only at the end of the half
 * cycle in progress, so how far it coasts depends on the phase it stopped in; extraCoast[n] seconds more on the n-th
 * stop, or, when negative, that long rocking back along its path as its feet settle.
 */
struct Body {
    float forwardScale = 1, turnScale = 1; // of what the sticks ask, the gait walks this much
    float drift = 0;                       // rad/s of yaw while walking, as the classical gait drifts
    float cadence = 1.5f;                  // gait cycles/s
    std::vector<float> extraCoast;

    OdometryEstimate pose;
    float heading = 0; // unwrapped
    float right = 0, ahead = 0, yawRate = 0;
    float phase = 0, coastLeft = 0, backLeft = 0, sense = 1;
    bool walking = false, stopping = false;
    int stops = 0;

    Body(float x = 0, float y = 0, float yaw = 0) : pose(at(x, y, yaw)), heading(yaw) {}

    bool atRest() const { return !walking; }

    void step(const GoalSticks& sticks) {
        if (!centred(sticks)) {
            const float norm = std::max(1.0f, std::fabs(sticks.lx) + std::fabs(sticks.ly) + std::fabs(sticks.rx));
            const float k = DT / (LAG_S + DT), top = TOP_SPEED_MM / 1000 * forwardScale;
            right += k * (-sticks.lx / norm * top - right);
            ahead += k * (sticks.ly / norm * top - ahead);
            yawRate += k * (sticks.rx / norm * TOP_TURN * turnScale - yawRate);
            walking = true;
            stopping = false;
            sense = 1;
        } else if (walking) {
            if (!stopping) {
                const float half = 2 * phase;
                const float extra = stops < static_cast<int>(extraCoast.size()) ? extraCoast[stops] : 0;
                coastLeft = (std::floor(half) + 1 - half) / (2 * cadence) + std::max(extra, 0.0f);
                backLeft = std::max(-extra, 0.0f);
                stopping = true;
                stops++;
            }
            if (coastLeft > 0) {
                coastLeft -= DT;
            } else if (backLeft > 0) {
                backLeft -= DT;
                sense = -1;
            } else {
                walking = stopping = false;
                sense = 1;
                right = ahead = yawRate = 0;
            }
        }
        if (!walking) return;
        phase += cadence * DT;
        const float c = std::cos(heading), s = std::sin(heading);
        pose.x += sense * (right * c - ahead * s) * DT;
        pose.y += sense * (right * s + ahead * c) * DT;
        pose.distance += std::hypot(right, ahead) * DT;
        heading += (sense * yawRate + drift) * DT;
        pose.yaw = std::remainder(heading, TWO_PI);
    }
};

struct Run {
    GoalState end = GoalState::RUNNING;
    float seconds = 0;
    float firstLx = 0, firstLy = 0, firstRx = 0;
    float maxLx = 0, maxLy = 0, maxRx = 0;   // the largest stick magnitudes asked for
    float minLy = 0;                         // the most backward forward stick
    bool endedAtRest = false;                // the controller was told the body was at rest on its last tick
    bool sticksCentredWhileSettling = true;  // no SETTLING tick asked for a stick
    std::vector<OdometryEstimate> passPoses; // the settled pose each later pass started from
    float maxRadialError = 0;                // for an arc: the largest distance off its circle
};

// Runs the goal to its end against the body; `centre` is the arc's centre in the world frame when one is followed.
Run drive(GoalController& goal, Body& body, float maxSeconds = 300, const Local* centre = nullptr, float radius = 0) {
    Run run;
    GoalState last = goal.state();
    for (int tick = 0; tick < maxSeconds / DT; tick++) {
        const bool atRest = body.atRest();
        const GoalSticks sticks = goal.update(body.pose, atRest, DT);
        run.seconds = (tick + 1) * DT;
        if (tick == 0) {
            run.firstLx = sticks.lx;
            run.firstLy = sticks.ly;
            run.firstRx = sticks.rx;
        }
        if (last == GoalState::SETTLING && goal.state() == GoalState::RUNNING) {
            run.passPoses.push_back(body.pose);
        }
        if (goal.state() == GoalState::SETTLING && !centred(sticks)) run.sticksCentredWhileSettling = false;
        last = goal.state();
        if (!active(goal)) {
            run.end = goal.state();
            run.endedAtRest = atRest;
            TEST_ASSERT_TRUE(centred(sticks));
            break;
        }
        run.maxLx = std::max(run.maxLx, std::fabs(sticks.lx));
        run.maxLy = std::max(run.maxLy, std::fabs(sticks.ly));
        run.minLy = std::min(run.minLy, sticks.ly);
        run.maxRx = std::max(run.maxRx, std::fabs(sticks.rx));
        body.step(sticks);
        if (centre) {
            const float off = std::hypot(body.pose.x - centre->right, body.pose.y - centre->forward) - radius;
            run.maxRadialError = std::max(run.maxRadialError, std::fabs(off));
        }
    }
    while (!body.atRest()) body.step({});
    return run;
}

// The time a pass of `remaining` (m or rad) may take at `pace` (m/s or rad/s): Leika's expected time, twice, plus 2 s.
float deadline(float remaining, float zone, float pace) {
    const float d = std::fabs(remaining);
    const float expected = (std::max(d - zone, 0.0f) + std::min(d, zone) / GoalController::MIN_FRACTION) / pace;
    return 2 * expected + GoalController::TIMEOUT_MARGIN_S;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_a_forward_goal_lands_within_its_tolerance() {
    for (const float distance : {0.6f, -0.3f}) {
        Body body(0.4f, -0.2f, 2.5f);
        const OdometryEstimate origin = body.pose;
        GoalController goal;
        goal.start(forward(distance, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
        TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_M, distance, local(origin, body.pose).forward);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, std::copysign(0.5f, distance), run.firstLy);
        TEST_ASSERT_EQUAL_FLOAT(0, run.firstLx);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, run.maxLy); // the style's speed caps the driving stick
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, run.maxLx);    // only the line hold, on rounding off the line
        TEST_ASSERT_TRUE(run.endedAtRest);
        TEST_ASSERT_TRUE(run.sticksCentredWhileSettling);
        TEST_ASSERT_TRUE(goal.passes() >= 1 && goal.passes() <= GoalController::MAX_PASSES);
    }
}

// A positive SIDEWAYS goal is to the right, which MotionService walks on a negative lx.
void test_a_sideways_goal_signs_its_stick_by_motion_services_convention() {
    for (const float distance : {0.25f, -0.25f}) {
        Body body(-0.1f, 0.3f, -1.0f);
        const OdometryEstimate origin = body.pose;
        GoalController goal;
        goal.start(sideways(distance, 0.7f), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, -std::copysign(0.7f, distance), run.firstLx);
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, run.maxLy); // only the line hold, on rounding off the line
        TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_M, distance, local(origin, body.pose).right);
        TEST_ASSERT_TRUE(run.endedAtRest);
    }
}

// The odometry's yaw wraps at pi; a turn past it, or past a whole turn, is followed on the unwrapped change.
void test_turns_land_in_both_directions_across_the_wrap() {
    struct Case {
        float startYaw, angle, turnScale;
    };
    for (const Case c : {Case {3.0f, 90 * DEG, 1}, Case {-3.0f, -400 * DEG, 1}, Case {0.2f, 120 * DEG, 0.85f}}) {
        Body body(0.1f, 0.1f, c.startYaw);
        body.turnScale = c.turnScale;
        GoalController goal;
        goal.start(turn(c.angle, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
        TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_RAD, c.angle, body.heading - c.startYaw);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, std::copysign(0.5f, c.angle), run.firstRx);
        TEST_ASSERT_EQUAL_FLOAT(0, run.maxLx);
        TEST_ASSERT_EQUAL_FLOAT(0, run.maxLy);
        TEST_ASSERT_TRUE(run.endedAtRest);
    }
}

// A positive angle curves left, a negative one right; a full circle comes back to where it began.
void test_arcs_land_on_their_end_pose_and_follow_their_circle() {
    struct Case {
        float radius, angle;
    };
    for (const Case c : {Case {0.3f, 90 * DEG}, Case {0.5f, -90 * DEG}, Case {0.25f, 360 * DEG}}) {
        Body body(0.2f, -0.4f, 0.7f);
        const OdometryEstimate origin = body.pose;
        // The centre lies a radius to the left of a left arc, to the right of a right one.
        const float side = std::copysign(c.radius, -c.angle), co = std::cos(origin.yaw), so = std::sin(origin.yaw);
        const Local centre {origin.x + side * co, origin.y + side * so};
        GoalController goal;
        goal.start(arc(c.radius, c.angle, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body, 300, &centre, c.radius);
        TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
        TEST_ASSERT_TRUE(run.firstLy > 0);
        TEST_ASSERT_TRUE(run.firstRx * c.angle > 0);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, run.maxLy);
        const Local end = local(origin, body.pose), ideal = arcEnd(c.radius, c.angle);
        TEST_ASSERT_TRUE(std::hypot(end.right - ideal.right, end.forward - ideal.forward) <=
                         GoalController::ARC_REACHED_M);
        TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_RAD, c.angle, body.heading - origin.yaw);
        TEST_ASSERT_TRUE(run.maxRadialError < 0.03f);
        TEST_ASSERT_TRUE(run.endedAtRest);
    }
}

// An arc tighter than the turn stick can follow at the asked speed slows its forward stick instead.
void test_a_tight_arc_slows_its_forward_stick_to_keep_its_curvature() {
    constexpr float RADIUS = 0.05f;
    Body body;
    GoalController goal;
    goal.start(arc(RADIUS, 180 * DEG, 1), body.pose, TOP_SPEED_MM, TOP_TURN);
    const GoalSticks first = goal.update(body.pose, true, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1, first.rx);
    TEST_ASSERT_TRUE(first.ly < 1);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1 / RADIUS, first.rx * TOP_TURN / (first.ly * TOP_SPEED_MM / 1000));
    // Its floor still turns at a fifth of the top turn rate: short quick steps keep the stop's coast within tolerance.
    body.cadence = 4;
    goal.start(arc(RADIUS, 180 * DEG, 1), body.pose, TOP_SPEED_MM, TOP_TURN);
    const Local centre {-RADIUS, 0};
    const Run run = drive(goal, body, 300, &centre, RADIUS);
    TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
    TEST_ASSERT_TRUE(run.maxRx <= 1);
    TEST_ASSERT_TRUE(run.maxRadialError < 0.01f);
    const Local end = local(at(0, 0, 0), body.pose), ideal = arcEnd(RADIUS, 180 * DEG);
    TEST_ASSERT_TRUE(std::hypot(end.right - ideal.right, end.forward - ideal.forward) <= GoalController::ARC_REACHED_M);
}

// Full speed until the last 50 mm or 30 deg, in proportion over them, and never below a fifth of the asked speed. It
// walks on at the floor until it is past the target, then centres its sticks and settles.
void test_it_slows_over_the_last_stretch_down_to_a_floor() {
    GoalController goal;
    goal.start(forward(0.2f, 0.8f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, goal.update(at(0, 0.1f, 0), false, DT).ly);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.48f, goal.update(at(0, 0.17f, 0), false, DT).ly);   // 30 mm left
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.192f, goal.update(at(0, 0.188f, 0), false, DT).ly); // 12 mm left
    const GoalSticks floor = goal.update(at(0, 0.192f, 0), false, DT);                    // 8 mm left
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.16f, floor.ly);
    TEST_ASSERT_EQUAL_FLOAT(0, floor.lx);
    TEST_ASSERT_EQUAL_FLOAT(0, floor.rx);
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 0.202f, 0), false, DT))); // 2 mm past
    TEST_ASSERT_EQUAL(GoalState::SETTLING, goal.state());

    goal.start(turn(90 * DEG, 0.8f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, goal.update(at(0, 0, 60 * DEG), false, DT).rx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, goal.update(at(0, 0, 75 * DEG), false, DT).rx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.16f, goal.update(at(0, 0, 86 * DEG), false, DT).rx); // 4 deg left
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.16f, goal.update(at(0, 0, 88 * DEG), false, DT).rx);
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 0, 91 * DEG), false, DT)));
    TEST_ASSERT_EQUAL(GoalState::SETTLING, goal.state());

    // Past the target by more than the slow zone it walks back at full speed.
    goal.start(forward(0.1f, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    const GoalSticks back = goal.update(at(0, -0.1f, 0), false, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, back.ly);
}

// A fast goal's slowdown zone is the distance its speed covers in two stops' coasts, when that is longer than
// SLOW_DOWN. It centres its sticks once the remainder is within the coast predicted from the speed its gait walks now,
// which follows the sticks through the command ramp, so a pass from rest drives even on a remainder shorter than its
// floor's coast.
void test_a_fast_goal_slows_early_and_stops_its_coast_short() {
    constexpr float FAST_MM = 600; // about BI's top at the default posture
    const float zone = 2 * FAST_MM / 1000 * GoalController::STOP_S;
    const float coast = FAST_MM / 1000 * GoalController::STOP_S; // after a second at full speed
    TEST_ASSERT_TRUE(zone > 4 * GoalController::SLOW_DOWN_M);
    const auto atSpeed = [&](GoalController& goal) {
        goal.start(forward(1, 1), at(0, 0, 0), FAST_MM, TOP_TURN);
        for (int i = 0; i < 200; i++) TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1, goal.update(at(0, 0, 0), false, DT).ly);
    };
    GoalController goal;
    atSpeed(goal);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.6f, goal.update(at(0, 1 - 0.6f * zone, 0), false, DT).ly);
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 1 - 0.9f * coast, 0), false, DT)));
    TEST_ASSERT_EQUAL(GoalState::SETTLING, goal.state());

    atSpeed(goal);
    const float remaining = 1.1f * coast;
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, remaining / zone, goal.update(at(0, 1 - remaining, 0), false, DT).ly);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, goal.state());

    // From rest, 20 mm is less than the floor's coast at speed, but nothing has started to coast yet.
    goal.start(forward(0.02f, 1), at(0, 0, 0), FAST_MM, TOP_TURN);
    TEST_ASSERT_TRUE(GoalController::MIN_FRACTION * coast > 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, GoalController::MIN_FRACTION, goal.update(at(0, 0, 0), false, DT).ly);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, goal.state());
}

// The arc's slowdown is on its remaining angle and slows the forward and turn sticks together.
void test_an_arc_slows_on_its_remaining_angle() {
    constexpr float RADIUS = 1.0f;
    constexpr float TURN_PER_FORWARD = TOP_SPEED_MM / 1000 / RADIUS / TOP_TURN; // the turn stick per forward stick
    const auto onArc = [&](float angle) {
        const Local p = arcEnd(RADIUS, angle);
        return at(p.right, p.forward, angle, RADIUS * angle);
    };
    GoalController goal;
    goal.start(arc(RADIUS, 90 * DEG, 0.8f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    GoalSticks sticks = goal.update(onArc(30 * DEG), false, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, sticks.ly);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f * TURN_PER_FORWARD, sticks.rx);
    sticks = goal.update(onArc(75 * DEG), false, DT); // 15 deg left: half the speed
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f, sticks.ly);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.4f * TURN_PER_FORWARD, sticks.rx);
    sticks = goal.update(onArc(88 * DEG), false, DT); // the floor
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.16f, sticks.ly);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.16f * TURN_PER_FORWARD, sticks.rx);
    TEST_ASSERT_TRUE(centred(goal.update(onArc(91 * DEG), false, DT)));
    TEST_ASSERT_EQUAL(GoalState::SETTLING, goal.state());
    TEST_ASSERT_TRUE(centred(goal.update(onArc(91 * DEG), true, DT)));
    TEST_ASSERT_EQUAL(GoalState::REACHED, goal.state());
}

// A straight walk steers its turn stick against the fused heading's error from the start heading, 2 per rad, at most
// 0.3, across the yaw's wrap, and its stick across the line against its offset from the start line.
void test_heading_hold_steers_against_a_drifting_yaw() {
    GoalController goal;
    goal.start(forward(1, 0.5f), at(0.3f, 0.3f, 3.1f), TOP_SPEED_MM, TOP_TURN);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.1f, goal.update(at(0.3f, 0.3f, 3.15f), false, DT).rx);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, goal.update(at(0.3f, 0.3f, 2.9f), false, DT).rx);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.3f, goal.update(at(0.3f, 0.3f, 3.5f), false, DT).rx);

    goal.start(sideways(0.5f, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    const GoalSticks hold = goal.update(at(0, 0, 0.1f), false, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.5f, hold.lx);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.2f, hold.rx);

    // A gait drifting 0.15 rad/s would turn well over a radian on these walks; held, they end near their heading,
    // and the heading's steady error, about 0.06 rad, no longer carries them off the line.
    for (const Goal walk : {forward(1, 0.5f), sideways(-0.6f, 0.5f)}) {
        Body body(0, 0, 1.0f);
        body.drift = 0.15f;
        const OdometryEstimate origin = body.pose;
        goal.start(walk, body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
        TEST_ASSERT_TRUE(body.drift * run.seconds > 1);
        TEST_ASSERT_FLOAT_WITHIN(0.1f, 0, body.heading - origin.yaw);
        const Local end = local(origin, body.pose);
        const bool ahead = walk.kind == GoalKind::FORWARD;
        TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_M, walk.distance, ahead ? end.forward : end.right);
        TEST_ASSERT_FLOAT_WITHIN(GoalController::CROSS_REACHED_M, 0, ahead ? end.right : end.forward);
    }
}

// The stick across the line is 10 per m of offset, at most 0.3: right of a forward walk's line walks left (lx > 0),
// ahead of a sideways walk's line walks back (ly < 0).
void test_straight_walks_hold_their_line_with_signs_by_the_stick_convention() {
    GoalController goal;
    constexpr float YAW = 0.5f;
    const float c = std::cos(YAW), s = std::sin(YAW);
    goal.start(forward(1, 0.5f), at(0.3f, 0.3f, YAW), TOP_SPEED_MM, TOP_TURN);
    GoalSticks sticks = goal.update(at(0.3f + 0.02f * c, 0.3f + 0.02f * s, YAW), false, DT); // 20 mm right
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, sticks.lx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, sticks.ly);
    sticks = goal.update(at(0.3f - 0.02f * c, 0.3f - 0.02f * s, YAW), false, DT); // 20 mm left
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.2f, sticks.lx);
    sticks = goal.update(at(0.3f + 0.05f * c, 0.3f + 0.05f * s, YAW), false, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, sticks.lx);

    goal.start(sideways(0.5f, 0.5f), at(0, 0, -YAW), TOP_SPEED_MM, TOP_TURN);
    sticks = goal.update(at(-0.02f * std::sin(-YAW), 0.02f * std::cos(-YAW), -YAW), false, DT); // 20 mm ahead
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.2f, sticks.ly);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.5f, sticks.lx);
    sticks = goal.update(at(0.02f * std::sin(-YAW), -0.02f * std::cos(-YAW), -YAW), false, DT); // 20 mm behind
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, sticks.ly);
}

// Past the target but off its line, a straight walk walks only across until it is within 20 mm; settled on the target
// but off the line, its next pass walks only across.
void test_a_pass_walks_the_remaining_cross_track() {
    GoalController goal;
    goal.start(forward(0.2f, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    GoalSticks sticks = goal.update(at(0.03f, 0.203f, 0), false, DT);
    TEST_ASSERT_EQUAL(GoalState::RUNNING, goal.state());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, sticks.lx);
    TEST_ASSERT_EQUAL_FLOAT(0, sticks.ly);
    TEST_ASSERT_TRUE(centred(goal.update(at(0.015f, 0.203f, 0), false, DT)));
    TEST_ASSERT_EQUAL(GoalState::SETTLING, goal.state());
    sticks = goal.update(at(0.025f, 0.197f, 0), true, DT); // settled 3 mm short and 25 mm right
    TEST_ASSERT_EQUAL(GoalState::RUNNING, goal.state());
    TEST_ASSERT_EQUAL(2, goal.passes());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.25f, sticks.lx);
    TEST_ASSERT_EQUAL_FLOAT(0, sticks.ly);
    TEST_ASSERT_TRUE(centred(goal.update(at(0.01f, 0.197f, 0), false, DT)));
    TEST_ASSERT_TRUE(centred(goal.update(at(0.01f, 0.197f, 0), true, DT)));
    TEST_ASSERT_EQUAL(GoalState::REACHED, goal.state());
    TEST_ASSERT_EQUAL_FLOAT(1, goal.progress());
}

// The arc adds the same gain on its heading's error from the ideal heading at the distance travelled, so a gait that
// turns short of its stick still lands on the arc's end.
void test_the_arc_corrects_its_yaw_toward_the_ideal_heading() {
    constexpr float RADIUS = 0.5f;
    constexpr float TURN_PER_FORWARD = TOP_SPEED_MM / 1000 / RADIUS / TOP_TURN;
    GoalController goal;
    goal.start(arc(RADIUS, 90 * DEG, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    // 0.2 m along, the ideal heading is 0.4 rad: 0.1 rad short asks 0.2 more.
    GoalSticks sticks = goal.update(at(0, 0, 0.3f, 0.2f), false, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, sticks.ly);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f * TURN_PER_FORWARD + 0.2f, sticks.rx);
    // 0.2 rad ahead asks 0.4 less, clipped to 0.3.
    sticks = goal.update(at(0, 0, 0.6f, 0.2f), false, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f * TURN_PER_FORWARD - 0.3f, sticks.rx);

    struct Case {
        float turnScale, drift;
    };
    for (const Case c : {Case {0.7f, 0}, Case {1, -0.1f}}) {
        Body body(0, 0, -2.0f);
        body.turnScale = c.turnScale;
        body.drift = c.drift;
        const OdometryEstimate origin = body.pose;
        goal.start(arc(0.3f, 90 * DEG, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
        const Local end = local(origin, body.pose), ideal = arcEnd(0.3f, 90 * DEG);
        TEST_ASSERT_TRUE(std::hypot(end.right - ideal.right, end.forward - ideal.forward) <=
                         GoalController::ARC_REACHED_M);
        TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_RAD, 90 * DEG, body.heading - origin.yaw);
    }
}

// The sticks centre once past the target, but the goal is judged only once the robot is at rest.
void test_it_reaches_only_once_at_rest() {
    GoalController goal;
    goal.start(forward(0.2f, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 0.203f, 0), false, DT)));
    for (int i = 0; i < 100; i++) {
        TEST_ASSERT_TRUE(centred(goal.update(at(0, 0.205f, 0), false, DT)));
        TEST_ASSERT_EQUAL(GoalState::SETTLING, goal.state());
    }
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 0.195f, 0), true, DT))); // it rocked back 5 mm short
    TEST_ASSERT_EQUAL(GoalState::REACHED, goal.state());
    TEST_ASSERT_EQUAL(1, goal.passes());
    TEST_ASSERT_EQUAL_FLOAT(1, goal.progress());
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 0.1f, 0), true, DT))); // an ended goal asks for nothing more
}

// A coast past the tolerance is walked back on a second pass from the settled pose.
void test_a_settled_overshoot_gets_a_corrective_pass() {
    Body body;
    body.extraCoast = {1.0f}; // the first stop slides on a second at the floor speed, about 12 mm
    GoalController goal;
    goal.start(forward(0.3f, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
    const Run run = drive(goal, body);
    TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
    TEST_ASSERT_EQUAL(2, goal.passes());
    TEST_ASSERT_EQUAL(1, static_cast<int>(run.passPoses.size()));
    TEST_ASSERT_TRUE(run.passPoses[0].y > 0.3f + GoalController::REACHED_M); // the first pass settled outside
    TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_M, 0.3f, body.pose.y);
    TEST_ASSERT_TRUE(run.endedAtRest);
}

// A robot that slides past the tolerance at every stop gives up at rest after MAX_PASSES passes.
void test_it_times_out_after_max_passes() {
    Body body;
    body.extraCoast = {1.5f, 1.5f, 1.5f, 1.5f};
    GoalController goal;
    goal.start(turn(45 * DEG, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
    const Run run = drive(goal, body);
    TEST_ASSERT_EQUAL(GoalState::TIMED_OUT, run.end);
    TEST_ASSERT_EQUAL(GoalController::MAX_PASSES, goal.passes());
    TEST_ASSERT_EQUAL(GoalController::MAX_PASSES - 1, static_cast<int>(run.passPoses.size()));
    TEST_ASSERT_TRUE(run.endedAtRest); // judged on the settled pose, not cut off by the clock
    TEST_ASSERT_EQUAL(3, body.stops);
    TEST_ASSERT_TRUE(std::fabs(body.heading - 45 * DEG) > GoalController::REACHED_RAD);
}

// An arc that coasts past its angle walks back along the arc on its next pass; one that rocks back short walks on.
void test_an_arc_corrects_an_overshoot_backwards_and_an_undershoot_forwards() {
    struct Case {
        float extraCoast;
        bool over;
    };
    for (const Case c : {Case {2.0f, true}, Case {-3.0f, false}}) {
        Body body(0.1f, 0.2f, 0.4f);
        body.extraCoast = {c.extraCoast};
        const OdometryEstimate origin = body.pose;
        GoalController goal;
        goal.start(arc(0.3f, 90 * DEG, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::REACHED, run.end);
        TEST_ASSERT_EQUAL(2, goal.passes());
        const float settled = std::remainder(run.passPoses[0].yaw - origin.yaw, TWO_PI);
        if (c.over) {
            TEST_ASSERT_TRUE(settled > 90 * DEG + GoalController::REACHED_RAD);
            TEST_ASSERT_TRUE(run.minLy < 0);
        } else {
            TEST_ASSERT_TRUE(settled < 90 * DEG - GoalController::REACHED_RAD);
            TEST_ASSERT_EQUAL_FLOAT(0, run.minLy);
        }
        const Local end = local(origin, body.pose), ideal = arcEnd(0.3f, 90 * DEG);
        TEST_ASSERT_TRUE(std::hypot(end.right - ideal.right, end.forward - ideal.forward) <=
                         GoalController::ARC_REACHED_M);
        TEST_ASSERT_FLOAT_WITHIN(GoalController::REACHED_RAD, 90 * DEG, body.heading - origin.yaw);
    }
}

// An arc whose heading lands but whose end does not has no remaining angle for a pass to walk: it fails at once.
void test_an_arc_off_its_end_with_its_angle_reached_fails() {
    GoalController goal;
    goal.start(arc(0.5f, 90 * DEG, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    goal.update(at(0, 0, 91 * DEG, 0.8f), false, DT);
    TEST_ASSERT_EQUAL(GoalState::SETTLING, goal.state());
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 0, 91 * DEG, 0.8f), true, DT)));
    TEST_ASSERT_EQUAL(GoalState::TIMED_OUT, goal.state());
    TEST_ASSERT_EQUAL(1, goal.passes());
}

// Each pass may take twice its expected time plus 2 s, the expected time being the remainder at full pace outside the
// slow zone and at the floor inside it.
void test_a_goal_that_gets_nowhere_times_out() {
    const float speedPace = 0.5f * TOP_SPEED_MM / 1000, turnPace = 0.5f * TOP_TURN;
    {
        Body body;
        body.forwardScale = body.turnScale = 0;
        GoalController goal;
        goal.start(forward(0.2f, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::TIMED_OUT, run.end);
        TEST_ASSERT_FLOAT_WITHIN(2 * DT, deadline(0.2f, GoalController::SLOW_DOWN_M, speedPace), run.seconds);
        TEST_ASSERT_EQUAL(1, goal.passes());
    }
    {
        // A short turn is all slow zone.
        Body body;
        body.forwardScale = body.turnScale = 0;
        GoalController goal;
        goal.start(turn(-10 * DEG, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::TIMED_OUT, run.end);
        TEST_ASSERT_FLOAT_WITHIN(2 * DT, deadline(10 * DEG, GoalController::SLOW_DOWN_RAD, turnPace), run.seconds);
    }
    {
        // An arc's pace is the yaw rate of its sticks, as MotionService shares a combined stick out (L1-normalized).
        constexpr float RADIUS = 0.1f, SPEED = 0.8f;
        const float turnStick = SPEED * TOP_SPEED_MM / 1000 / RADIUS / TOP_TURN; // 0.8: both sticks fit
        const float yawPace = turnStick / (SPEED + turnStick) * TOP_TURN;
        Body body;
        body.forwardScale = body.turnScale = 0;
        GoalController goal;
        goal.start(arc(RADIUS, 90 * DEG, SPEED), body.pose, TOP_SPEED_MM, TOP_TURN);
        const Run run = drive(goal, body);
        TEST_ASSERT_EQUAL(GoalState::TIMED_OUT, run.end);
        TEST_ASSERT_FLOAT_WITHIN(2 * DT, deadline(90 * DEG, GoalController::SLOW_DOWN_RAD, yawPace), run.seconds);
    }
    {
        // The clock restarts with each pass, on the remainder that pass walks.
        Body body;
        body.extraCoast = {1.0f};
        GoalController goal;
        goal.start(forward(0.3f, 0.5f), body.pose, TOP_SPEED_MM, TOP_TURN);
        GoalState last = goal.state();
        float passStart = 0, remainder = 0, seconds = 0;
        for (int tick = 0; active(goal) && tick < 100000; tick++) {
            const GoalSticks sticks = goal.update(body.pose, body.atRest(), DT);
            seconds = (tick + 1) * DT;
            if (last == GoalState::SETTLING && goal.state() == GoalState::RUNNING) {
                passStart = seconds;
                remainder = 0.3f - body.pose.y;
                body.forwardScale = 0; // stuck from here on
            }
            last = goal.state();
            body.step(sticks);
        }
        TEST_ASSERT_EQUAL(GoalState::TIMED_OUT, goal.state());
        TEST_ASSERT_EQUAL(2, goal.passes());
        TEST_ASSERT_TRUE(passStart > 0);
        TEST_ASSERT_FLOAT_WITHIN(2 * DT, deadline(remainder, GoalController::SLOW_DOWN_M, speedPace),
                                 seconds - passStart);
    }
}

void test_cancel_ends_only_an_active_goal() {
    GoalController goal;
    goal.cancel();
    TEST_ASSERT_EQUAL(GoalState::IDLE, goal.state());

    goal.start(sideways(0.1f, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    TEST_ASSERT_FALSE(centred(goal.update(at(0, 0, 0), false, DT)));
    goal.cancel();
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, goal.state());
    TEST_ASSERT_TRUE(centred(goal.update(at(0.01f, 0, 0), false, DT)));
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, goal.state());

    goal.start(turn(0.5f, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    goal.update(at(0, 0, 0.52f), false, DT);
    TEST_ASSERT_EQUAL(GoalState::SETTLING, goal.state());
    goal.cancel();
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, goal.state());
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 0, 0.52f), true, DT)));
    TEST_ASSERT_EQUAL(GoalState::CANCELLED, goal.state());

    goal.start(turn(0.5f, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    goal.update(at(0, 0, 0.51f), false, DT);
    goal.update(at(0, 0, 0.51f), true, DT);
    TEST_ASSERT_EQUAL(GoalState::REACHED, goal.state());
    goal.cancel();
    TEST_ASSERT_EQUAL(GoalState::REACHED, goal.state());
}

void test_valid_refuses_malformed_goals_and_start_reports_refused() {
    TEST_ASSERT_TRUE(GoalController::valid(forward(-0.2f, 0.5f)));
    TEST_ASSERT_TRUE(GoalController::valid(sideways(0.1f, 1)));
    TEST_ASSERT_TRUE(GoalController::valid(turn(-3, 0.01f)));
    TEST_ASSERT_TRUE(GoalController::valid(arc(0.4f, -TWO_PI, 0.5f)));

    TEST_ASSERT_FALSE(GoalController::valid(forward(0, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(sideways(0, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(turn(0, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(arc(0.4f, 0, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(arc(0, 1, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(arc(-0.4f, 1, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(forward(NAN, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(forward(INFINITY, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(turn(NAN, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(arc(NAN, 1, 0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid({GoalKind::FORWARD, 0.2f, NAN, 0, 0.5f})); // unused, but not a number
    TEST_ASSERT_FALSE(GoalController::valid(forward(0.2f, 0)));
    TEST_ASSERT_FALSE(GoalController::valid(forward(0.2f, -0.5f)));
    TEST_ASSERT_FALSE(GoalController::valid(forward(0.2f, 1.01f)));
    TEST_ASSERT_FALSE(GoalController::valid(forward(0.2f, NAN)));
    TEST_ASSERT_FALSE(GoalController::valid({static_cast<GoalKind>(7), 0.2f, 0.2f, 0.2f, 0.5f}));

    GoalController goal;
    for (const float top : {0.0f, -1.0f, NAN, INFINITY}) {
        goal.start(forward(0.2f, 0.5f), at(0, 0, 0), top, TOP_TURN);
        TEST_ASSERT_EQUAL(GoalState::REFUSED, goal.state());
        goal.start(arc(0.3f, 1, 0.5f), at(0, 0, 0), TOP_SPEED_MM, top);
        TEST_ASSERT_EQUAL(GoalState::REFUSED, goal.state());
    }

    goal.start(turn(0.5f, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    goal.update(at(0, 0, 0.51f), false, DT);
    goal.update(at(0, 0, 0.51f), true, DT);
    TEST_ASSERT_EQUAL(GoalState::REACHED, goal.state());
    TEST_ASSERT_EQUAL_FLOAT(1, goal.progress());
    goal.start(arc(0, 1, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    TEST_ASSERT_EQUAL(GoalState::REFUSED, goal.state());
    TEST_ASSERT_EQUAL(0, goal.passes());
    TEST_ASSERT_TRUE(centred(goal.update(at(0, 0, 0), true, DT)));
    goal.cancel();
    TEST_ASSERT_EQUAL(GoalState::REFUSED, goal.state());
    TEST_ASSERT_EQUAL_FLOAT(0, goal.progress());
}

// Progress is the share of the target covered, from 0 to 1, whatever the sign of the target.
void test_progress_runs_from_zero_to_one() {
    GoalController goal;
    TEST_ASSERT_EQUAL_FLOAT(0, goal.progress());
    goal.start(forward(-0.4f, 0.5f), at(0, 0, PI / 2), TOP_SPEED_MM, TOP_TURN);
    TEST_ASSERT_EQUAL_FLOAT(0, goal.progress());
    goal.update(at(0.1f, 0, PI / 2), false, DT); // facing -x, a step backwards is +x
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.25f, goal.progress());
    goal.update(at(-0.1f, 0, PI / 2), false, DT); // the wrong way
    TEST_ASSERT_EQUAL_FLOAT(0, goal.progress());
    goal.update(at(0.41f, 0, PI / 2), false, DT); // past
    TEST_ASSERT_EQUAL_FLOAT(1, goal.progress());

    goal.start(arc(0.3f, -180 * DEG, 0.5f), at(0, 0, 0), TOP_SPEED_MM, TOP_TURN);
    goal.update(at(0, 0, -45 * DEG, 0.3f * PI / 4), false, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.25f, goal.progress());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_a_forward_goal_lands_within_its_tolerance);
    RUN_TEST(test_a_sideways_goal_signs_its_stick_by_motion_services_convention);
    RUN_TEST(test_turns_land_in_both_directions_across_the_wrap);
    RUN_TEST(test_arcs_land_on_their_end_pose_and_follow_their_circle);
    RUN_TEST(test_a_tight_arc_slows_its_forward_stick_to_keep_its_curvature);
    RUN_TEST(test_it_slows_over_the_last_stretch_down_to_a_floor);
    RUN_TEST(test_a_fast_goal_slows_early_and_stops_its_coast_short);
    RUN_TEST(test_an_arc_slows_on_its_remaining_angle);
    RUN_TEST(test_heading_hold_steers_against_a_drifting_yaw);
    RUN_TEST(test_straight_walks_hold_their_line_with_signs_by_the_stick_convention);
    RUN_TEST(test_a_pass_walks_the_remaining_cross_track);
    RUN_TEST(test_the_arc_corrects_its_yaw_toward_the_ideal_heading);
    RUN_TEST(test_it_reaches_only_once_at_rest);
    RUN_TEST(test_a_settled_overshoot_gets_a_corrective_pass);
    RUN_TEST(test_it_times_out_after_max_passes);
    RUN_TEST(test_an_arc_corrects_an_overshoot_backwards_and_an_undershoot_forwards);
    RUN_TEST(test_an_arc_off_its_end_with_its_angle_reached_fails);
    RUN_TEST(test_a_goal_that_gets_nowhere_times_out);
    RUN_TEST(test_cancel_ends_only_an_active_goal);
    RUN_TEST(test_valid_refuses_malformed_goals_and_start_reports_refused);
    RUN_TEST(test_progress_runs_from_zero_to_one);
    return UNITY_END();
}
