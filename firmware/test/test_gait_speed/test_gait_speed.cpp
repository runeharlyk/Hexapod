// Host-side tests for the speed envelope lookup, the stick mapping and the posture clamp. The
// invariants that matter are that the stick lands on a fixed fraction of the measured top speed,
// that no commanded stroke leaves the table, and that a clamped posture is one the table covers.

#include <unity.h>

#include <algorithm>
#include <cmath>
#include <gait_speed.h>

namespace {

constexpr GaitType WALKING_GAITS[4] = {GaitType::TRI_GATE, GaitType::BI_GATE, GaitType::WAVE, GaitType::RIPPLE};

float gridValue(int i) { return -1.0f + 0.25f * (float)i; }

float standFrac(GaitType gait) { return gait_envelope::STAND_FRAC[(int)gait]; }

// Body speed of an axis follows v = stroke * f / d: a stroke every cycle, f cycles/s, d the stance share.
float axisSpeed(GaitType gait, float stroke, const WalkTarget& target) {
    return stroke * target.phaseRate / standFrac(gait);
}

float tableVMax(GaitType gait, int axis, int i, int j, int k) { return gait_envelope::V_MAX[(int)gait][axis][i][j][k]; }

float tableStroke(GaitType gait, int axis, int i, int j, int k) {
    return gait_envelope::STROKE[(int)gait][axis][i][j][k];
}

void assertZeroTarget(const WalkTarget& target) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepX);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepY);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepAngle);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.phaseRate);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.speed01);
}

} // namespace

void setUp() {}
void tearDown() {}

void test_full_stick_forward_reaches_v_max() {
    const AxisEnvelope forward = envelopeAt(GaitType::TRI_GATE, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(forward.vMax > 0.0f);

    const WalkTarget target = classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, 0, 0, 0);

    TEST_ASSERT_EQUAL_FLOAT(forward.stroke, target.stepY);
    TEST_ASSERT_FLOAT_WITHIN(0.005f * forward.vMax, forward.vMax, target.stepY * target.phaseRate / (3.1f / 6));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepX);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepAngle);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, target.speed01);
}

void test_default_speed_is_sixty_percent() {
    for (GaitType gait : WALKING_GAITS) {
        const AxisEnvelope forward = envelopeAt(gait, 0, 0, 0, 0);
        const WalkTarget target = classicWalkTarget(gait, 0, 1, 0, 0, 0, 0, 0);
        TEST_ASSERT_FLOAT_WITHIN(0.005f * 0.6f * forward.vMax, 0.6f * forward.vMax,
                                 axisSpeed(gait, target.stepY, target));
    }
}

void test_slower_command_shortens_stroke_and_cadence_alike() {
    // Speed is stroke x cadence, so a command at a fraction F of the top speed takes sqrt(F) of both:
    // neither a full stroke at a crawl nor a flailing cadence on a short one.
    for (GaitType gait : WALKING_GAITS) {
        const AxisEnvelope forward = envelopeAt(gait, 0, 0, 0, 0);
        const float topCadence = forward.vMax * standFrac(gait) / forward.stroke;
        const WalkTarget target = classicWalkTarget(gait, 0, 1, 0, 0, 0, 0, 0);
        const float root = std::sqrt(0.6f);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f * forward.stroke, root * forward.stroke, target.stepY);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f * topCadence, root * topCadence, target.phaseRate);
    }
}

void test_stop_commands_nothing() {
    assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, -1, 0, 0, 0));
    assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 1, 1, 1, -1, 0, 0, 0));
    assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 0, 0, 1, 0, 0, 0));
    assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 0, 0, 0, 0, 0, 0));
}

void test_signs_follow_the_convention() {
    const WalkTarget left = classicWalkTarget(GaitType::TRI_GATE, -1, 0, 0, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(left.stepX > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, left.stepY);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, left.stepAngle);

    const WalkTarget right = classicWalkTarget(GaitType::TRI_GATE, 1, 0, 0, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(right.stepX < 0.0f);

    const WalkTarget forward = classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(forward.stepY > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, forward.stepX);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, forward.stepAngle);

    const WalkTarget backward = classicWalkTarget(GaitType::TRI_GATE, 0, -1, 0, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(backward.stepY < 0.0f);

    const WalkTarget turnLeft = classicWalkTarget(GaitType::TRI_GATE, 0, 0, 1, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(turnLeft.stepAngle > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, turnLeft.stepX);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, turnLeft.stepY);

    const WalkTarget turnRight = classicWalkTarget(GaitType::TRI_GATE, 0, 0, -1, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(turnRight.stepAngle < 0.0f);
}

void test_combined_stick_is_l1_normalized() {
    const GaitType gait = GaitType::TRI_GATE;
    const WalkTarget target = classicWalkTarget(gait, 1, 1, 1, 1, 0, 0, 0);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, target.speed01);
    const float share[3] = {axisSpeed(gait, target.stepY, target) / envelopeAt(gait, 0, 0, 0, 0).vMax,
                            axisSpeed(gait, -target.stepX, target) / envelopeAt(gait, 1, 0, 0, 0).vMax,
                            axisSpeed(gait, target.stepAngle, target) / envelopeAt(gait, 2, 0, 0, 0).vMax};
    for (float s : share) TEST_ASSERT_FLOAT_WITHIN(0.005f / 3, 1.0f / 3, s);
}

void test_stick_scales_every_axis_by_its_own_share() {
    // An unequal stick: the shares follow |stick| / L, whatever the axes' own limits are.
    const GaitType gait = GaitType::RIPPLE;
    const float lx = -0.5f, ly = 1.0f, rx = 0.25f, s = 0.5f;
    const float frac = speedFraction(s);
    const WalkTarget target = classicWalkTarget(gait, lx, ly, rx, s, 0, 0, 0);
    const float length = std::fabs(lx) + std::fabs(ly) + std::fabs(rx);

    TEST_ASSERT_FLOAT_WITHIN(0.005f * frac * ly / length, frac * ly / length,
                             axisSpeed(gait, target.stepY, target) / envelopeAt(gait, 0, 0, 0, 0).vMax);
    TEST_ASSERT_FLOAT_WITHIN(0.005f * frac * 0.5f / length, frac * 0.5f / length,
                             axisSpeed(gait, target.stepX, target) / envelopeAt(gait, 1, 0, 0, 0).vMax);
    TEST_ASSERT_FLOAT_WITHIN(0.005f * frac * 0.25f / length, frac * 0.25f / length,
                             axisSpeed(gait, target.stepAngle, target) / envelopeAt(gait, 2, 0, 0, 0).vMax);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, frac, target.speed01);
}

void test_partial_stick_commands_a_fraction_of_the_speed() {
    // A half stick below the L1 limit is half the speed, not a full-speed command at a shorter stroke.
    const GaitType gait = GaitType::BI_GATE;
    const float vMax = envelopeAt(gait, 0, 0, 0, 0).vMax;
    const WalkTarget target = classicWalkTarget(gait, 0, 0.5f, 0, 1, 0, 0, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.005f * 0.5f * vMax, 0.5f * vMax, axisSpeed(gait, target.stepY, target));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, target.speed01);
}

void test_speed_rises_with_the_speed_input() {
    const GaitType gait = GaitType::TRI_GATE;
    float previous = 0.0f;
    for (float s : {-0.5f, 0.0f, 0.5f, 1.0f}) {
        const WalkTarget target = classicWalkTarget(gait, 0, 1, 0, s, 0, 0, 0);
        const float speed = axisSpeed(gait, target.stepY, target);
        TEST_ASSERT_TRUE(speed > previous);
        previous = speed;
    }
}

void test_speed_fraction_anchors() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, speedFraction(-1.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.6f, speedFraction(0.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, speedFraction(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, speedFraction(-0.5f));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, speedFraction(0.5f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, speedFraction(-3.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, speedFraction(3.0f));
}

void test_commands_stay_inside_the_table() {
    // The stroke sweep bounded the joints and the cadence cap bounded the servos, so a command that
    // leaves either bound would walk outside what the table measured.
    const float sticks[][3] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}, {1, 1, 1}, {-0.5f, 1, 0.25f}, {0.3f, -0.7f, 0.9f}};
    const float postures[][3] = {{0, 0, 0}, {-0.5f, 0.25f, 0.5f}, {0.5f, -0.5f, -0.25f}};
    for (GaitType gait : WALKING_GAITS) {
        for (const auto& posture : postures) {
            for (const auto& stick : sticks) {
                for (float s : {0.0f, 0.4f, 1.0f}) {
                    const WalkTarget target =
                        classicWalkTarget(gait, stick[0], stick[1], stick[2], s, posture[0], posture[1], posture[2]);
                    TEST_ASSERT_TRUE(target.phaseRate > 0.0f);
                    TEST_ASSERT_TRUE(target.phaseRate <= gait_envelope::CADENCE_MAX * 1.0001f);
                    const AxisEnvelope fwd = envelopeAt(gait, 0, posture[0], posture[1], posture[2]);
                    const AxisEnvelope lat = envelopeAt(gait, 1, posture[0], posture[1], posture[2]);
                    const AxisEnvelope yaw = envelopeAt(gait, 2, posture[0], posture[1], posture[2]);
                    TEST_ASSERT_TRUE(std::fabs(target.stepY) <= fwd.stroke * 1.0001f);
                    TEST_ASSERT_TRUE(std::fabs(target.stepX) <= lat.stroke * 1.0001f);
                    TEST_ASSERT_TRUE(std::fabs(target.stepAngle) <= yaw.stroke * 1.0001f);
                    TEST_ASSERT_TRUE(target.speed01 > 0.0f && target.speed01 <= 1.0f);
                }
            }
        }
    }
}

void test_unwalkable_posture_and_unsupported_gaits_command_nothing() {
    TEST_ASSERT_FALSE(postureWalkable(-1.0f, -1.0f, 1.0f));
    assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, -1.0f, -1.0f, 1.0f));

    assertZeroTarget(classicWalkTarget(GaitType::TUNED, 0, 1, 0, 1, 0, 0, 0));
    assertZeroTarget(classicWalkTarget(GaitType::AUTO, 0, 1, 0, 1, 0, 0, 0));
}

void test_clamp_keeps_walkable_postures() {
    int checked = 0;
    for (int i = 0; i < gait_envelope::GRID_N; ++i)
        for (int j = 0; j < gait_envelope::GRID_N; ++j)
            for (int k = 0; k < gait_envelope::GRID_N; ++k) {
                float s1 = gridValue(i), h = gridValue(j), fd = gridValue(k);
                if (!postureWalkable(s1, h, fd)) continue;
                const float before[3] = {s1, h, fd};
                clampPosture(s1, h, fd);
                TEST_ASSERT_EQUAL_FLOAT(before[0], s1);
                TEST_ASSERT_EQUAL_FLOAT(before[1], h);
                TEST_ASSERT_EQUAL_FLOAT(before[2], fd);
                ++checked;
            }
    TEST_ASSERT_TRUE(checked > 100);
}

void test_clamp_pulls_in_unwalkable_ones() {
    // The walkable region is not star-shaped about the neutral posture, so a ray can leave it and
    // re-enter. The result must be walkable and no scan step (k/32 of the command) beyond it may be.
    int pulled = 0;
    for (int i = 0; i <= 10; ++i)
        for (int j = 0; j <= 10; ++j)
            for (int k = 0; k <= 10; ++k) {
                const float s1 = -1.0f + 0.2f * (float)i, h = -1.0f + 0.2f * (float)j, fd = -1.0f + 0.2f * (float)k;
                if (postureWalkable(s1, h, fd)) continue;
                float a = s1, b = h, c = fd;
                clampPosture(a, b, c);
                TEST_ASSERT_TRUE(postureWalkable(a, b, c));
                // The command keeps its direction: nothing moves outward and the result is collinear.
                TEST_ASSERT_TRUE(std::fabs(a) <= std::fabs(s1) && std::fabs(b) <= std::fabs(h) &&
                                 std::fabs(c) <= std::fabs(fd));
                TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, a * h - b * s1);
                TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, b * fd - c * h);

                const float largest = std::max({std::fabs(s1), std::fabs(h), std::fabs(fd)});
                const float scale = std::max({std::fabs(a), std::fabs(b), std::fabs(c)}) / largest;
                for (int step = 1; step <= 32; ++step) {
                    const float t = (float)step / 32;
                    if (t <= scale + 1e-4f) continue;
                    TEST_ASSERT_FALSE(postureWalkable(s1 * t, h * t, fd * t));
                }
                ++pulled;
            }
    TEST_ASSERT_TRUE(pulled > 500);
}

void test_clamp_reaches_the_boundary_of_a_ray_that_reenters() {
    // (-1, 1, 0) leaves the walkable region at about a quarter of the way out and re-enters it further
    // out, so only a scan from the outside gets past the inner boundary.
    float s1 = -1.0f, h = 1.0f, fd = 0.0f;
    TEST_ASSERT_FALSE(postureWalkable(s1, h, fd));
    clampPosture(s1, h, fd);
    TEST_ASSERT_TRUE(postureWalkable(s1, h, fd));
    TEST_ASSERT_TRUE(h > 0.6f);
}

void test_clamp_clips_commands_outside_the_unit_cube() {
    float s1 = 0.0f, h = -7.0f, fd = 0.0f;
    clampPosture(s1, h, fd);
    TEST_ASSERT_TRUE(h >= -1.0f);
    TEST_ASSERT_TRUE(postureWalkable(s1, h, fd));
}

void test_interpolation_hits_grid_values() {
    int checked = 0;
    for (GaitType gait : WALKING_GAITS)
        for (int axis = 0; axis < 3; ++axis)
            for (int i = 0; i < gait_envelope::GRID_N - 1; ++i)
                for (int j = 0; j < gait_envelope::GRID_N - 1; ++j)
                    for (int k = 0; k < gait_envelope::GRID_N - 1; ++k) {
                        if (!postureWalkable(gridValue(i), gridValue(j), gridValue(k))) continue;
                        const AxisEnvelope e = envelopeAt(gait, axis, gridValue(i), gridValue(j), gridValue(k));
                        TEST_ASSERT_EQUAL_FLOAT(tableVMax(gait, axis, i, j, k), e.vMax);
                        TEST_ASSERT_EQUAL_FLOAT(tableStroke(gait, axis, i, j, k), e.stroke);
                        ++checked;
                    }
    TEST_ASSERT_TRUE(checked > 1000);
}

void test_interpolation_between_grid_points_is_trilinear() {
    // The centre of a cell is the mean of its eight corners; a lookup that snapped to the nearest
    // node or interpolated one axis only would miss it.
    const int i = 2, j = 3, k = 2;
    TEST_ASSERT_TRUE(postureWalkable(gridValue(i) + 0.125f, gridValue(j) + 0.125f, gridValue(k) + 0.125f));
    for (GaitType gait : WALKING_GAITS)
        for (int axis = 0; axis < 3; ++axis) {
            float vMax = 0.0f, stroke = 0.0f;
            for (int di = 0; di < 2; ++di)
                for (int dj = 0; dj < 2; ++dj)
                    for (int dk = 0; dk < 2; ++dk) {
                        vMax += tableVMax(gait, axis, i + di, j + dj, k + dk) / 8.0f;
                        stroke += tableStroke(gait, axis, i + di, j + dj, k + dk) / 8.0f;
                    }
            const AxisEnvelope e =
                envelopeAt(gait, axis, gridValue(i) + 0.125f, gridValue(j) + 0.125f, gridValue(k) + 0.125f);
            TEST_ASSERT_FLOAT_WITHIN(1e-4f * std::fabs(vMax) + 1e-6f, vMax, e.vMax);
            TEST_ASSERT_FLOAT_WITHIN(1e-4f * std::fabs(stroke) + 1e-6f, stroke, e.stroke);
        }

    // A quarter of the way along one axis weighs the two corners 3:1.
    const AxisEnvelope quarter = envelopeAt(GaitType::TRI_GATE, 0, gridValue(i), gridValue(j), gridValue(k) + 0.0625f);
    const float lo = tableVMax(GaitType::TRI_GATE, 0, i, j, k), hi = tableVMax(GaitType::TRI_GATE, 0, i, j, k + 1);
    TEST_ASSERT_TRUE(lo != hi);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f * std::fabs(lo), lo + 0.25f * (hi - lo), quarter.vMax);
}

void test_lookup_clips_inputs_and_is_empty_when_unwalkable() {
    const AxisEnvelope edge = envelopeAt(GaitType::TRI_GATE, 0, -1.0f, 0.0f, 0.0f);
    const AxisEnvelope beyond = envelopeAt(GaitType::TRI_GATE, 0, -4.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(edge.vMax > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(edge.vMax, beyond.vMax);
    TEST_ASSERT_EQUAL_FLOAT(edge.stroke, beyond.stroke);

    const AxisEnvelope none = envelopeAt(GaitType::TRI_GATE, 0, -1.0f, -1.0f, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, none.vMax);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, none.stroke);
}

void test_walkable_needs_every_corner_of_the_cell() {
    // Counted independently of postureWalkable, from the cell centre so that no corner is favoured.
    int open = 0, partial = 0;
    for (int i = 0; i < gait_envelope::GRID_N - 1; ++i)
        for (int j = 0; j < gait_envelope::GRID_N - 1; ++j)
            for (int k = 0; k < gait_envelope::GRID_N - 1; ++k) {
                int walkable = 0;
                for (int c = 0; c < 8; ++c)
                    walkable += gait_envelope::WALKABLE[i + (c & 1)][j + ((c >> 1) & 1)][k + (c >> 2)];
                const bool all = postureWalkable(gridValue(i) + 0.125f, gridValue(j) + 0.125f, gridValue(k) + 0.125f);
                TEST_ASSERT_EQUAL(walkable == 8, all);
                open += walkable == 8;
                partial += walkable > 0 && walkable < 8;
            }
    TEST_ASSERT_TRUE(open > 0);
    TEST_ASSERT_TRUE(partial > 0);
}

// The stand clamp shares clampPosture's ray scan over standFeasible instead: a standable posture is kept as is, and
// any other lands on a standable one along its own ray with no scan step beyond it standable.
void test_stand_clamp_keeps_standable_postures_and_pulls_in_the_rest() {
    Kinematics kin;
    int kept = 0, pulled = 0;
    for (int i = 0; i <= 10; ++i)
        for (int j = 0; j <= 10; ++j) {
            const float h = -1.0f + 0.2f * (float)i, fd = -1.0f + 0.2f * (float)j;
            float a = h, b = fd;
            clampStandPosture(kin, a, b);
            if (standFeasible(kin, h, fd)) {
                TEST_ASSERT_EQUAL_FLOAT(h, a);
                TEST_ASSERT_EQUAL_FLOAT(fd, b);
                ++kept;
                continue;
            }
            TEST_ASSERT_TRUE(standFeasible(kin, a, b));
            TEST_ASSERT_TRUE(std::fabs(a) <= std::fabs(h) && std::fabs(b) <= std::fabs(fd));
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, a * fd - b * h);
            const float scale = std::max(std::fabs(a), std::fabs(b)) / std::max(std::fabs(h), std::fabs(fd));
            for (int step = 1; step <= 32; ++step) {
                const float t = (float)step / 32;
                if (t > scale + 1e-4f) TEST_ASSERT_FALSE(standFeasible(kin, h * t, fd * t));
            }
            ++pulled;
        }
    TEST_ASSERT_TRUE(kept > 0 && pulled > 0);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_full_stick_forward_reaches_v_max);
    RUN_TEST(test_default_speed_is_sixty_percent);
    RUN_TEST(test_slower_command_shortens_stroke_and_cadence_alike);
    RUN_TEST(test_stop_commands_nothing);
    RUN_TEST(test_signs_follow_the_convention);
    RUN_TEST(test_combined_stick_is_l1_normalized);
    RUN_TEST(test_stick_scales_every_axis_by_its_own_share);
    RUN_TEST(test_partial_stick_commands_a_fraction_of_the_speed);
    RUN_TEST(test_speed_rises_with_the_speed_input);
    RUN_TEST(test_speed_fraction_anchors);
    RUN_TEST(test_commands_stay_inside_the_table);
    RUN_TEST(test_unwalkable_posture_and_unsupported_gaits_command_nothing);
    RUN_TEST(test_clamp_keeps_walkable_postures);
    RUN_TEST(test_clamp_pulls_in_unwalkable_ones);
    RUN_TEST(test_clamp_reaches_the_boundary_of_a_ray_that_reenters);
    RUN_TEST(test_clamp_clips_commands_outside_the_unit_cube);
    RUN_TEST(test_stand_clamp_keeps_standable_postures_and_pulls_in_the_rest);
    RUN_TEST(test_interpolation_hits_grid_values);
    RUN_TEST(test_interpolation_between_grid_points_is_trilinear);
    RUN_TEST(test_lookup_clips_inputs_and_is_empty_when_unwalkable);
    RUN_TEST(test_walkable_needs_every_corner_of_the_cell);
    return UNITY_END();
}
