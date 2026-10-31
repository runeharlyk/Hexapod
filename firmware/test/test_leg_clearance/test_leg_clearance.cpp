// Host test of the leg-to-leg clearance check (leg_clearance.h) against platform_shared/leg_clearance_fixtures.txt,
// which simulation/gen_leg_clearance_fixtures.py writes from simulation/src/robot/clearance.py and
// simulation/test_leg_clearance.py holds to it, so the firmware and the simulation judge a pose alike.

#include <unity.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <leg_clearance.h>

namespace {

struct Case {
    std::string name;
    int pair[2];
    bool clear;
    float clearance;
    float angles[18];
};

struct Segment {
    std::string name;
    leg_clearance::Vec3 ends[4]; // p0 p1 q0 q1
    float distance;
};

struct Fixtures {
    float radii[4] = {0, 0, 0, 0};
    float margin = 0;
    std::vector<Case> cases;
    std::vector<Segment> segments;
};

Fixtures readFixtures() {
    Fixtures f;
    std::ifstream file("platform_shared/leg_clearance_fixtures.txt");
    TEST_ASSERT_TRUE_MESSAGE(file.good(), "platform_shared/leg_clearance_fixtures.txt");
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream words(line);
        std::string kind;
        words >> kind;
        if (kind == "radii") {
            for (float& r : f.radii) words >> r;
        } else if (kind == "margin") {
            words >> f.margin;
        } else if (kind == "case") {
            Case c;
            int clear = 0;
            words >> c.name >> c.pair[0] >> c.pair[1] >> clear >> c.clearance;
            c.clear = clear == 1;
            for (float& a : c.angles) words >> a;
            TEST_ASSERT_FALSE_MESSAGE(words.fail(), c.name.c_str());
            f.cases.push_back(c);
        } else if (kind == "segment") {
            Segment g;
            words >> g.name;
            for (leg_clearance::Vec3& v : g.ends) words >> v.x >> v.y >> v.z;
            words >> g.distance;
            TEST_ASSERT_FALSE_MESSAGE(words.fail(), g.name.c_str());
            f.segments.push_back(g);
        }
    }
    return f;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_the_fixtures_state_the_radii_and_the_margin_the_check_uses() {
    const Fixtures f = readFixtures();
    TEST_ASSERT_EQUAL_FLOAT(leg_clearance::COXA_RADIUS_MM, f.radii[0]);
    TEST_ASSERT_EQUAL_FLOAT(leg_clearance::FEMUR_RADIUS_MM, f.radii[1]);
    TEST_ASSERT_EQUAL_FLOAT(leg_clearance::TIBIA_RADIUS_MM, f.radii[2]);
    TEST_ASSERT_EQUAL_FLOAT(leg_clearance::FOOT_RADIUS_MM, f.radii[3]);
    TEST_ASSERT_EQUAL_FLOAT(LEG_CLEARANCE_MM, f.margin);
}

// The fixtures cover every pair touching, 0.1 mm inside the margin and 0.1 mm clear of it as the closest, legs through
// each other, the stance, the crab and poses drawn within travel (test_leg_clearance.py checks that they do). The
// float check agrees with the float64 reference within 0.01 mm: the chain's coordinates are under 250 mm, where a
// float's rounding is 1.5e-5 mm, and the about thirty operations from angle to distance stay well inside 0.01 mm, which
// is still ten times finer than the 0.1 mm the threshold cases sit from the margin. The closest pair is the one a
// refusal names.
void test_the_clearance_of_every_fixture_matches_the_simulation() {
    const Fixtures f = readFixtures();
    TEST_ASSERT_GREATER_THAN(20, f.cases.size());
    for (const Case& c : f.cases) {
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, c.clearance, legClearance(c.angles), c.name.c_str());
        TEST_ASSERT_EQUAL_MESSAGE(c.clear, legsClear(c.angles), c.name.c_str());
        const int pair = leg_clearance::closest(c.angles).pair;
        TEST_ASSERT_EQUAL_MESSAGE(c.pair[0], leg_clearance::PAIRS[pair][0], c.name.c_str());
        TEST_ASSERT_EQUAL_MESSAGE(c.pair[1], leg_clearance::PAIRS[pair][1], c.name.c_str());
    }
}

// Every branch of the segment distance on its own: a point against a segment either way and against a point, crossing,
// skew, parallel and collinear segments, segments 1e-2 to 1e-7 rad from parallel at a tibia's length, and segments
// drawn at random, their coordinates exact in float. Within 1e-3 mm of the float64 reference: float rounds coordinates
// of up to 200 mm to 1.5e-5 mm, and the cross products the skew branch is solved with keep their relative precision
// however near parallel the segments.
void test_every_segment_distance_matches_the_simulation() {
    const Fixtures f = readFixtures();
    TEST_ASSERT_GREATER_THAN(20, f.segments.size());
    for (const Segment& g : f.segments)
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1e-3f, g.distance,
                                         leg_clearance::segmentDistance(g.ends[0], g.ends[1], g.ends[2], g.ends[3]),
                                         g.name.c_str());
}

namespace {

// A fixture's angles by name.
std::vector<float> fixturePose(const char* name) {
    for (const Case& c : readFixtures().cases)
        if (c.name == name) return std::vector<float>(c.angles, c.angles + 18);
    TEST_FAIL_MESSAGE(name);
    return {};
}

} // namespace

// From a clear pose, a move may not bring two legs within the margin; from a pose already within it, any move that
// brings no pair closer than the closest is taken, so legs inside the margin can always move apart.
void test_a_move_keeps_the_legs_apart_from_the_margin_or_from_where_they_are() {
    const std::vector<float> stance = fixturePose("stance"), inside = fixturePose("inside_0_1"),
                             clear = fixturePose("clear_0_1"), touching = fixturePose("touching_0_1");
    TEST_ASSERT_TRUE(keepsLegsApart(stance.data(), clear.data()));
    TEST_ASSERT_FALSE(keepsLegsApart(stance.data(), inside.data()));
    TEST_ASSERT_TRUE(keepsLegsApart(touching.data(), inside.data()));
    TEST_ASSERT_FALSE(keepsLegsApart(inside.data(), touching.data()));
    std::vector<float> elsewhere = touching;
    elsewhere[10] += 10.0f; // leg 3's femur, far from legs 0 and 1
    TEST_ASSERT_TRUE(keepsLegsApart(touching.data(), elsewhere.data()));
}

// The rule holds pair by pair: legs 0 and 1 through each other do not let legs 3 and 4 come together. From
// `through_0_1` with leg 4's coxa turned 40 deg toward leg 3, which keeps that pair apart, leg 3's turned 40 deg
// toward leg 4 would bring them through each other (they meet at about 38.7 deg each) and is refused, while legs 0
// and 1 may still part.
void test_a_pair_already_touching_lets_no_other_pair_come_together() {
    std::vector<float> from = fixturePose("through_0_1");
    const std::vector<float> touching34 = fixturePose("touching_3_4");
    from[12] = touching34[12] < 0.0f ? -40.0f : 40.0f; // leg 4 toward leg 3
    TEST_ASSERT_TRUE(keepsLegsApart(fixturePose("through_0_1").data(), from.data()));
    std::vector<float> to = from;
    to[9] = -from[12];                                  // leg 3 toward leg 4
    TEST_ASSERT_TRUE(legClearance(from.data()) < 0.0f); // legs 0 and 1 through each other
    TEST_ASSERT_FALSE(keepsLegsApart(from.data(), to.data()));
    std::vector<float> parting = from;
    parting[0] += from[0] < 0.0f ? 2.0f : -2.0f; // leg 0's coxa back toward its stance, away from leg 1
    TEST_ASSERT_TRUE(keepsLegsApart(from.data(), parting.data()));
}

// An angle that is no number makes no clearance, and keeps nothing apart.
void test_a_non_finite_angle_keeps_nothing_apart() {
    const std::vector<float> stance = fixturePose("stance");
    for (const float bad : {NAN, INFINITY}) {
        std::vector<float> pose = stance;
        pose[0] = bad;
        TEST_ASSERT_TRUE(std::isnan(legClearance(pose.data())));
        TEST_ASSERT_FALSE(legsClear(pose.data()));
        TEST_ASSERT_FALSE(keepsLegsApart(stance.data(), pose.data()));
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_the_fixtures_state_the_radii_and_the_margin_the_check_uses);
    RUN_TEST(test_the_clearance_of_every_fixture_matches_the_simulation);
    RUN_TEST(test_every_segment_distance_matches_the_simulation);
    RUN_TEST(test_a_move_keeps_the_legs_apart_from_the_margin_or_from_where_they_are);
    RUN_TEST(test_a_pair_already_touching_lets_no_other_pair_come_together);
    RUN_TEST(test_a_non_finite_angle_keeps_nothing_apart);
    return UNITY_END();
}
