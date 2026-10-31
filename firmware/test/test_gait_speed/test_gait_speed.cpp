// Host-side tests for the speed envelope lookup, the stick mapping, the posture clamp and the stances. The
// invariants that matter are that the stride picks the stroke from the measured geometry, that the
// stick sets the cadence as a fixed fraction of the stroke's top one, that no command leaves the
// stroke's travel or its servo bound, that a clamped posture is one the table covers, and that each
// stance's feet, table and standing check are its own.

#include <unity.h>

#include <algorithm>
#include <cmath>
#include <gait.h>
#include <gait_speed.h>
#include <leg_clearance.h>

namespace {

constexpr GaitType WALKING_GAITS[4] = {GaitType::TRI_GATE, GaitType::BI_GATE, GaitType::WAVE, GaitType::RIPPLE};
constexpr int SAMPLES = gait_envelope::STROKE_SAMPLES;
constexpr float STRIDES[5] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
constexpr Stance STANCES[2] = {Stance::DEFAULT, Stance::CRAB};

float gridValue(int i) { return -1.0f + 0.25f * (float)i; }

float standFrac(GaitType gait) { return gait_envelope::STAND_FRAC[(int)gait]; }

// Body speed of an axis follows v = stroke * f / d: a stroke every cycle, f cycles/s, d the stance share.
float axisSpeed(GaitType gait, float stroke, const WalkTarget& target) {
    return stroke * target.phaseRate / standFrac(gait);
}

// The stroke a walk target takes along an axis, in envelope order (forward, lateral, yaw).
float axisStroke(const WalkTarget& target, int axis) {
    const float stroke[3] = {target.stepY, -target.stepX, target.stepAngle};
    return std::fabs(stroke[axis]);
}

// The servo bound on a gait's cadence: the swing's peak joint rate over its share of the cycle, the stance's over its.
float topCadence(GaitType gait, float kappaSwing, float kappaStance) {
    const float d = standFrac(gait);
    return std::min(gait_envelope::SERVO_RAD_S / std::max(kappaSwing / (1.0f - d), kappaStance / d),
                    gait_envelope::CADENCE_MAX);
}

// Kappa at a stroke fraction from its samples at (k + 1) / SAMPLES: their running maximum, linear between them, and the
// first one below them.
float kappaAt(const float (&samples)[SAMPLES], float fraction) {
    float peak[SAMPLES];
    for (int n = 0; n < SAMPLES; ++n) peak[n] = n == 0 ? samples[0] : std::max(peak[n - 1], samples[n]);
    const float position = fraction * SAMPLES - 1.0f;
    if (position <= 0.0f) return peak[0];
    const int below = std::min((int)position, SAMPLES - 2);
    return peak[below] + (peak[below + 1] - peak[below]) * (position - (float)below);
}

// The table's values at grid point (i, j, k).
struct GridPoint {
    int axis, i, j, k;
    Stance feet = Stance::DEFAULT;
    float strokeMax() const { return gait_envelope::STROKE_MAX[(int)feet][axis][i][j][k]; }
    const float (&swing() const)[SAMPLES] { return gait_envelope::KAPPA_SWING[(int)feet][axis][i][j][k]; }
    const float (&stance() const)[SAMPLES] { return gait_envelope::KAPPA_STANCE[(int)feet][axis][i][j][k]; }
    float cadenceAt(GaitType gait, float fraction) const {
        return topCadence(gait, kappaAt(swing(), fraction), kappaAt(stance(), fraction));
    }
    float sampleSpeed(GaitType gait, int n) const {
        const float fraction = (float)(n + 1) / SAMPLES;
        return fraction * strokeMax() * cadenceAt(gait, fraction) / standFrac(gait);
    }
    float fastestSampleSpeed(GaitType gait) const {
        float fastest = 0.0f;
        for (int n = 0; n < SAMPLES; ++n) fastest = std::max(fastest, sampleSpeed(gait, n));
        return fastest;
    }
    AxisEnvelope envelope(GaitType gait, float stride) const {
        return envelopeAt(gait, axis, gridValue(i), gridValue(j), gridValue(k), stride, feet);
    }
};

constexpr GridPoint NEUTRAL[3] = {{0, 4, 4, 4}, {1, 4, 4, 4}, {2, 4, 4, 4}};

// A full stick along one axis, in envelope order.
WalkTarget fullStickAlong(GaitType gait, int axis, float s, float stride, float s1 = 0, float h = 0, float fd = 0) {
    const float stick[3] = {axis == 1 ? 1.0f : 0.0f, axis == 0 ? 1.0f : 0.0f, axis == 2 ? 1.0f : 0.0f};
    return classicWalkTarget(gait, stick[0], stick[1], stick[2], s, s1, h, fd, stride, Stance::DEFAULT);
}

void assertZeroTarget(const WalkTarget& target) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepX);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepY);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepAngle);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, target.phaseRate);
}

} // namespace

void setUp() {}
void tearDown() {}

// The stride's three anchors at full stick: 0 the stroke that walks fastest at its top cadence, +1 the longest the
// joints can travel, -1 three tenths of the fastest; each stepped at its own top cadence.
void test_the_stride_anchors_the_stroke() {
    for (GaitType gait : WALKING_GAITS)
        for (const GridPoint& point : NEUTRAL) {
            const WalkTarget optimal = fullStickAlong(gait, point.axis, 1, 0);
            const float fraction = axisStroke(optimal, point.axis) / point.strokeMax();
            const int n = (int)std::lround(fraction * SAMPLES) - 1;
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, (float)(n + 1) / SAMPLES, fraction);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * point.fastestSampleSpeed(gait), point.fastestSampleSpeed(gait),
                                     point.sampleSpeed(gait, n));
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, point.cadenceAt(gait, fraction), optimal.phaseRate);

            const WalkTarget longest = fullStickAlong(gait, point.axis, 1, 1);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * point.strokeMax(), point.strokeMax(), axisStroke(longest, point.axis));
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, point.cadenceAt(gait, 1.0f), longest.phaseRate);

            const WalkTarget shortest = fullStickAlong(gait, point.axis, 1, -1);
            const float shortStroke = 0.3f * axisStroke(optimal, point.axis);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * shortStroke, shortStroke, axisStroke(shortest, point.axis));
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, point.cadenceAt(gait, 0.3f * fraction), shortest.phaseRate);

            // The speed-optimal stroke can be the longest (BI forward at neutral is), but never the shortest.
            TEST_ASSERT_TRUE(axisStroke(shortest, point.axis) < axisStroke(optimal, point.axis));
        }
}

// Over the whole walkable grid: the stroke is linear in the stride between its anchors, stepped at the top cadence
// its interpolated kappa allows, and the top speed is that stroke at that cadence.
void test_the_envelope_follows_the_tables_on_the_grid() {
    int checked = 0, shortOfTheFirstSample = 0;
    for (Stance feet : STANCES)
        for (GaitType gait : WALKING_GAITS)
            for (int axis = 0; axis < 3; ++axis)
                for (int i = 0; i < gait_envelope::GRID_N - 1; ++i)
                    for (int j = 0; j < gait_envelope::GRID_N - 1; ++j)
                        for (int k = 0; k < gait_envelope::GRID_N - 1; ++k) {
                            if (!postureWalkable(gridValue(i), gridValue(j), gridValue(k), feet)) continue;
                            const GridPoint point {axis, i, j, k, feet};
                            if (point.strokeMax() <= 0.0f) {
                                const AxisEnvelope none = point.envelope(gait, 1);
                                TEST_ASSERT_EQUAL_FLOAT(0.0f, none.stroke);
                                TEST_ASSERT_EQUAL_FLOAT(0.0f, none.cadence);
                                TEST_ASSERT_EQUAL_FLOAT(0.0f, none.vMax);
                                continue;
                            }
                            const float optimal = point.envelope(gait, 0).stroke;
                            TEST_ASSERT_FLOAT_WITHIN(1e-5f * point.fastestSampleSpeed(gait),
                                                     point.fastestSampleSpeed(gait), point.envelope(gait, 0).vMax);
                            for (const float stride : STRIDES) {
                                const AxisEnvelope e = point.envelope(gait, stride);
                                const float expected = stride >= 0.0f ? optimal + stride * (point.strokeMax() - optimal)
                                                                      : optimal * (1.0f + stride * 0.7f);
                                TEST_ASSERT_FLOAT_WITHIN(1e-5f * point.strokeMax(), expected, e.stroke);
                                const float fraction = e.stroke / point.strokeMax();
                                shortOfTheFirstSample += fraction < 1.0f / SAMPLES;
                                TEST_ASSERT_FLOAT_WITHIN(1e-5f, point.cadenceAt(gait, fraction), e.cadence);
                                TEST_ASSERT_FLOAT_WITHIN(1e-5f * e.vMax, e.stroke * e.cadence / standFrac(gait),
                                                         e.vMax);
                                ++checked;
                            }
                        }
    TEST_ASSERT_TRUE(checked > 5000);
    TEST_ASSERT_TRUE(shortOfTheFirstSample > 0);
}

// Kappa is the running maximum of the samples up to the stroke: between two samples their linear blend where kappa
// rises, and below the first sample the first sample's.
void test_kappa_is_linear_between_samples_and_flat_below_them() {
    const float samples[SAMPLES] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512};
    using gait_speed_detail::kappaAtStroke;
    TEST_ASSERT_EQUAL_FLOAT(1.0f, kappaAtStroke(samples, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, kappaAtStroke(samples, 0.05f));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, kappaAtStroke(samples, 0.1f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, kappaAtStroke(samples, 0.25f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 128.0f + 0.25f * 128.0f, kappaAtStroke(samples, 0.825f));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 512.0f, kappaAtStroke(samples, 1.0f));

    // A dip reads as the higher sample before it, and a rise after it from there.
    const float dipping[SAMPLES] = {4, 3, 2, 5, 5, 5, 5, 5, 5, 5};
    TEST_ASSERT_EQUAL_FLOAT(4.0f, kappaAtStroke(dipping, 0.2f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.0f, kappaAtStroke(dipping, 0.3f));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 4.5f, kappaAtStroke(dipping, 0.35f));
    TEST_ASSERT_EQUAL_FLOAT(4.0f, kappaAtStroke(dipping, NAN));
}

// The table's own dips: where a sample falls below an earlier one, the envelope at that sample's stroke takes the
// running maximum, which the raw sample would understate.
void test_a_dip_in_the_table_reads_as_the_running_maximum() {
    int dips = 0, understated = 0;
    for (Stance feet : STANCES)
        for (GaitType gait : WALKING_GAITS)
            for (int axis = 0; axis < 3; ++axis)
                for (int i = 0; i < gait_envelope::GRID_N - 1; ++i)
                    for (int j = 0; j < gait_envelope::GRID_N - 1; ++j)
                        for (int k = 0; k < gait_envelope::GRID_N - 1; ++k) {
                            const GridPoint point {axis, i, j, k, feet};
                            if (!postureWalkable(gridValue(i), gridValue(j), gridValue(k), feet) ||
                                point.strokeMax() <= 0.0f)
                                continue;
                            const float optimal = point.envelope(gait, 0).stroke;
                            float peak = point.swing()[0];
                            for (int n = 1; n < SAMPLES; ++n) {
                                const float raw = point.swing()[n];
                                const bool dip = raw < peak;
                                peak = std::max(peak, raw);
                                if (!dip) continue;
                                // The stride whose stroke is sample n's.
                                const float stroke = (float)(n + 1) / SAMPLES * point.strokeMax();
                                const float stride =
                                    stroke < optimal ? (stroke / optimal - 1.0f) / (1.0f - MIN_STRIDE_SHARE)
                                    : optimal < point.strokeMax() ? (stroke - optimal) / (point.strokeMax() - optimal)
                                                                  : 1.0f;
                                if (stride < -1.0f || stride > 1.0f) continue;
                                const AxisEnvelope e = point.envelope(gait, stride);
                                const float fraction = (float)(n + 1) / SAMPLES;
                                TEST_ASSERT_FLOAT_WITHIN(1e-4f, point.cadenceAt(gait, fraction), e.cadence);
                                ++dips;
                                const float rawCadence = topCadence(gait, raw, kappaAt(point.stance(), fraction));
                                understated += rawCadence > e.cadence + 1e-4f;
                            }
                        }
    TEST_ASSERT_TRUE(dips > 0);
    TEST_ASSERT_TRUE(understated > 0);
}

void test_full_stick_reaches_the_top_speed() {
    for (const float stride : STRIDES) {
        const AxisEnvelope forward = envelopeAt(GaitType::TRI_GATE, 0, 0, 0, 0, stride, Stance::DEFAULT);
        TEST_ASSERT_TRUE(forward.vMax > 0.0f);

        const WalkTarget target = classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, 0, 0, 0, stride, Stance::DEFAULT);

        TEST_ASSERT_EQUAL_FLOAT(forward.stroke, target.stepY);
        TEST_ASSERT_EQUAL_FLOAT(forward.cadence, target.phaseRate);
        TEST_ASSERT_FLOAT_WITHIN(0.005f * forward.vMax, forward.vMax, target.stepY * target.phaseRate / (3.1f / 6));
        TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepX);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, target.stepAngle);
    }
}

void test_default_speed_is_sixty_percent() {
    for (GaitType gait : WALKING_GAITS) {
        const AxisEnvelope forward = envelopeAt(gait, 0, 0, 0, 0, 0, Stance::DEFAULT);
        const WalkTarget target = classicWalkTarget(gait, 0, 1, 0, 0, 0, 0, 0, 0, Stance::DEFAULT);
        TEST_ASSERT_FLOAT_WITHIN(0.005f * 0.6f * forward.vMax, 0.6f * forward.vMax,
                                 axisSpeed(gait, target.stepY, target));
    }
}

// The stride holds the stroke and the stick sets the cadence: a half stick takes the full stick's stroke at half its
// cadence, and the speed input scales the cadence alone.
void test_a_slower_stick_keeps_the_stroke_and_slows_the_cadence() {
    for (GaitType gait : WALKING_GAITS)
        for (const float stride : STRIDES)
            for (int axis = 0; axis < 3; ++axis) {
                const WalkTarget full = fullStickAlong(gait, axis, 1, stride);
                const float stick[3] = {axis == 1 ? 0.5f : 0.0f, axis == 0 ? 0.5f : 0.0f, axis == 2 ? 0.5f : 0.0f};
                const WalkTarget half =
                    classicWalkTarget(gait, stick[0], stick[1], stick[2], 1, 0, 0, 0, stride, Stance::DEFAULT);
                const WalkTarget slow = fullStickAlong(gait, axis, -0.5f, stride);
                TEST_ASSERT_TRUE(axisStroke(full, axis) > 0.0f);
                TEST_ASSERT_EQUAL_FLOAT(axisStroke(full, axis), axisStroke(half, axis));
                TEST_ASSERT_EQUAL_FLOAT(axisStroke(full, axis), axisStroke(slow, axis));
                TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f * full.phaseRate, half.phaseRate);
                TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f * full.phaseRate, slow.phaseRate);
            }
}

void test_stop_commands_nothing() {
    for (const float stride : STRIDES) {
        assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, -1, 0, 0, 0, stride, Stance::DEFAULT));
        assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 1, 1, 1, -1, 0, 0, 0, stride, Stance::DEFAULT));
        assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 0, 0, 1, 0, 0, 0, stride, Stance::DEFAULT));
        assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 0, 0, 0, 0, 0, 0, stride, Stance::DEFAULT));
    }
}

// docs/conventions.md: +lx strafes right (+x), +ly walks forward (+y) and +rx turns clockwise (negative yaw).
void test_signs_follow_the_convention() {
    const WalkTarget right = classicWalkTarget(GaitType::TRI_GATE, 1, 0, 0, 0, 0, 0, 0, 0, Stance::DEFAULT);
    TEST_ASSERT_TRUE(right.stepX > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, right.stepY);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, right.stepAngle);

    const WalkTarget left = classicWalkTarget(GaitType::TRI_GATE, -1, 0, 0, 0, 0, 0, 0, 0, Stance::DEFAULT);
    TEST_ASSERT_TRUE(left.stepX < 0.0f);

    const WalkTarget forward = classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 0, 0, 0, 0, 0, Stance::DEFAULT);
    TEST_ASSERT_TRUE(forward.stepY > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, forward.stepX);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, forward.stepAngle);

    const WalkTarget backward = classicWalkTarget(GaitType::TRI_GATE, 0, -1, 0, 0, 0, 0, 0, 0, Stance::DEFAULT);
    TEST_ASSERT_TRUE(backward.stepY < 0.0f);

    const WalkTarget turnRight = classicWalkTarget(GaitType::TRI_GATE, 0, 0, 1, 0, 0, 0, 0, 0, Stance::DEFAULT);
    TEST_ASSERT_TRUE(turnRight.stepAngle < 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, turnRight.stepX);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, turnRight.stepY);

    const WalkTarget turnLeft = classicWalkTarget(GaitType::TRI_GATE, 0, 0, -1, 0, 0, 0, 0, 0, Stance::DEFAULT);
    TEST_ASSERT_TRUE(turnLeft.stepAngle > 0.0f);
}

// A combined stick past an L1 norm of 1 is normalised by it: each axis takes a third of its stroke, and the gait steps
// at the slowest of the three axes' top cadences, so no axis runs faster than its own.
void test_combined_stick_is_l1_normalized() {
    const GaitType gait = GaitType::TRI_GATE;
    for (const float stride : STRIDES) {
        const WalkTarget target = classicWalkTarget(gait, 1, 1, 1, 1, 0, 0, 0, stride, Stance::DEFAULT);
        float slowest = INFINITY;
        for (int axis = 0; axis < 3; ++axis) {
            const AxisEnvelope e = envelopeAt(gait, axis, 0, 0, 0, stride, Stance::DEFAULT);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f / 3, axisStroke(target, axis) / e.stroke);
            slowest = std::min(slowest, e.cadence);
        }
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, slowest, target.phaseRate);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, autoDemand(1, 1, 1, 1, 0, 0, 0, stride).speed01);
    }
}

// An unequal stick: the strokes follow |stick| / L of each axis's own, and the cadence the speed fraction of the
// slowest axis's top cadence, whatever the axes' own limits are.
void test_stick_scales_every_axis_by_its_own_share() {
    const GaitType gait = GaitType::RIPPLE;
    const float lx = -0.5f, ly = 1.0f, rx = 0.25f, s = 0.5f;
    const float frac = speedFraction(s);
    const float length = std::fabs(lx) + std::fabs(ly) + std::fabs(rx);
    const float stick[3] = {ly, lx, rx};
    for (const float stride : STRIDES) {
        const WalkTarget target = classicWalkTarget(gait, lx, ly, rx, s, 0, 0, 0, stride, Stance::DEFAULT);
        float slowest = INFINITY;
        for (int axis = 0; axis < 3; ++axis) {
            const AxisEnvelope e = envelopeAt(gait, axis, 0, 0, 0, stride, Stance::DEFAULT);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, std::fabs(stick[axis]) / length, axisStroke(target, axis) / e.stroke);
            slowest = std::min(slowest, e.cadence);
        }
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, frac * slowest, target.phaseRate);
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, frac, autoDemand(lx, ly, rx, s, 0, 0, 0, stride).speed01);
    }
}

void test_partial_stick_commands_a_fraction_of_the_speed() {
    // A half stick below the L1 limit is half the speed, not a full-speed command at a shorter stroke.
    const GaitType gait = GaitType::BI_GATE;
    for (const float stride : STRIDES) {
        const float vMax = envelopeAt(gait, 0, 0, 0, 0, stride, Stance::DEFAULT).vMax;
        const WalkTarget target = classicWalkTarget(gait, 0, 0.5f, 0, 1, 0, 0, 0, stride, Stance::DEFAULT);
        TEST_ASSERT_FLOAT_WITHIN(0.005f * 0.5f * vMax, 0.5f * vMax, axisSpeed(gait, target.stepY, target));
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, autoDemand(0, 0.5f, 0, 1, 0, 0, 0, stride).speed01);
    }
}

void test_speed_rises_with_the_speed_input() {
    const GaitType gait = GaitType::TRI_GATE;
    float previous = 0.0f;
    for (float s : {-0.5f, 0.0f, 0.5f, 1.0f}) {
        const WalkTarget target = classicWalkTarget(gait, 0, 1, 0, s, 0, 0, 0, 0, Stance::DEFAULT);
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

// The stroke sweep bounded the joints and kappa the servos, so a command whose stroke on some axis passes that axis's
// longest, or whose cadence passes the top cadence of some axis it walks, would leave what the tables measured.
void test_commands_stay_inside_the_table() {
    const float sticks[][3] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}, {1, 1, 1}, {-0.5f, 1, 0.25f}, {0.3f, -0.7f, 0.9f}};
    const float postures[][3] = {{0, 0, 0}, {-0.5f, 0.25f, 0.5f}, {0.5f, -0.5f, -0.25f}, {1, -1, -1}};
    int mixed = 0;
    for (GaitType gait : WALKING_GAITS)
        for (const auto& p : postures)
            for (const auto& stick : sticks)
                for (float s : {0.0f, 0.4f, 1.0f})
                    for (const float stride : STRIDES) {
                        const WalkTarget target = classicWalkTarget(gait, stick[0], stick[1], stick[2], s, p[0], p[1],
                                                                    p[2], stride, Stance::DEFAULT);
                        TEST_ASSERT_TRUE(target.phaseRate > 0.0f);
                        TEST_ASSERT_TRUE(target.phaseRate <= gait_envelope::CADENCE_MAX * 1.0001f);
                        const float axisStick[3] = {stick[1], stick[0], stick[2]};
                        int used = 0;
                        for (int axis = 0; axis < 3; ++axis) {
                            const AxisEnvelope e = envelopeAt(gait, axis, p[0], p[1], p[2], stride, Stance::DEFAULT);
                            TEST_ASSERT_TRUE(e.stroke <=
                                             gait_speed_detail::strokeMaxAt(axis, p[0], p[1], p[2], Stance::DEFAULT) *
                                                 1.0001f);
                            TEST_ASSERT_TRUE(axisStroke(target, axis) <= e.stroke * 1.0001f);
                            if (axisStick[axis] == 0.0f || e.stroke <= 0.0f) continue;
                            TEST_ASSERT_TRUE(target.phaseRate <= e.cadence * 1.0001f);
                            ++used;
                        }
                        mixed += used > 1;
                    }
    TEST_ASSERT_TRUE(mixed > 0);
}

void test_unwalkable_posture_and_unsupported_gaits_command_nothing() {
    TEST_ASSERT_FALSE(postureWalkable(-1.0f, -1.0f, 1.0f, Stance::DEFAULT));
    assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, -1.0f, -1.0f, 1.0f, 0, Stance::DEFAULT));
    assertZeroTarget(classicWalkTarget(GaitType::TRI_GATE, 0, 1, 0, 1, -1.0f, -1.0f, 1.0f, 1, Stance::DEFAULT));

    assertZeroTarget(classicWalkTarget(GaitType::TUNED, 0, 1, 0, 1, 0, 0, 0, 0, Stance::DEFAULT));
    assertZeroTarget(classicWalkTarget(GaitType::AUTO, 0, 1, 0, 1, 0, 0, 0, 0, Stance::DEFAULT));
}

void test_clamp_keeps_walkable_postures() {
    int checked = 0;
    for (Stance feet : STANCES)
        for (int i = 0; i < gait_envelope::GRID_N; ++i)
            for (int j = 0; j < gait_envelope::GRID_N; ++j)
                for (int k = 0; k < gait_envelope::GRID_N; ++k) {
                    float s1 = gridValue(i), h = gridValue(j), fd = gridValue(k);
                    if (!postureWalkable(s1, h, fd, feet)) continue;
                    const float before[3] = {s1, h, fd};
                    clampPosture(s1, h, fd, feet);
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
    for (Stance feet : STANCES)
        for (int i = 0; i <= 10; ++i)
            for (int j = 0; j <= 10; ++j)
                for (int k = 0; k <= 10; ++k) {
                    const float s1 = -1.0f + 0.2f * (float)i, h = -1.0f + 0.2f * (float)j, fd = -1.0f + 0.2f * (float)k;
                    if (postureWalkable(s1, h, fd, feet)) continue;
                    float a = s1, b = h, c = fd;
                    clampPosture(a, b, c, feet);
                    TEST_ASSERT_TRUE(postureWalkable(a, b, c, feet));
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
                        TEST_ASSERT_FALSE(postureWalkable(s1 * t, h * t, fd * t, feet));
                    }
                    ++pulled;
                }
    TEST_ASSERT_TRUE(pulled > 500);
}

void test_clamp_reaches_the_boundary_of_a_ray_that_reenters() {
    // (-1, 0.75, 0) leaves the walkable region about two thirds of the way out and re-enters it further
    // out, so only a scan from the outside gets past the inner boundary.
    float s1 = -1.0f, h = 0.75f, fd = 0.0f;
    TEST_ASSERT_FALSE(postureWalkable(s1, h, fd, Stance::DEFAULT));
    clampPosture(s1, h, fd, Stance::DEFAULT);
    TEST_ASSERT_TRUE(postureWalkable(s1, h, fd, Stance::DEFAULT));
    TEST_ASSERT_TRUE(h > 0.6f);
}

void test_clamp_clips_commands_outside_the_unit_cube() {
    float s1 = 0.0f, h = -7.0f, fd = 0.0f;
    clampPosture(s1, h, fd, Stance::DEFAULT);
    TEST_ASSERT_TRUE(h >= -1.0f);
    TEST_ASSERT_TRUE(postureWalkable(s1, h, fd, Stance::DEFAULT));
}

void test_interpolation_between_grid_points_is_trilinear() {
    // The centre of a cell is the mean of its eight corners, for the longest stroke and for each kappa sample; a
    // lookup that snapped to the nearest node or interpolated one axis only would miss it.
    const int i = 2, j = 3, k = 2;
    const float s1 = gridValue(i) + 0.125f, h = gridValue(j) + 0.125f, fd = gridValue(k) + 0.125f;
    TEST_ASSERT_TRUE(postureWalkable(s1, h, fd, Stance::DEFAULT));
    for (GaitType gait : WALKING_GAITS)
        for (int axis = 0; axis < 3; ++axis) {
            float strokeMax = 0.0f, swing[SAMPLES] = {}, stance[SAMPLES] = {};
            for (int di = 0; di < 2; ++di)
                for (int dj = 0; dj < 2; ++dj)
                    for (int dk = 0; dk < 2; ++dk) {
                        const GridPoint corner {axis, i + di, j + dj, k + dk};
                        strokeMax += corner.strokeMax() / 8.0f;
                        for (int n = 0; n < SAMPLES; ++n) {
                            swing[n] += corner.swing()[n] / 8.0f;
                            stance[n] += corner.stance()[n] / 8.0f;
                        }
                    }
            TEST_ASSERT_FLOAT_WITHIN(1e-4f * strokeMax, strokeMax,
                                     gait_speed_detail::strokeMaxAt(axis, s1, h, fd, Stance::DEFAULT));
            const AxisEnvelope longest = envelopeAt(gait, axis, s1, h, fd, 1, Stance::DEFAULT);
            TEST_ASSERT_FLOAT_WITHIN(1e-4f * strokeMax, strokeMax, longest.stroke);
            TEST_ASSERT_FLOAT_WITHIN(1e-4f, topCadence(gait, kappaAt(swing, 1.0f), kappaAt(stance, 1.0f)),
                                     longest.cadence);
        }

    // A quarter of the way along one axis weighs the two corners 3:1.
    const float lo = GridPoint {0, i, j, k}.strokeMax(), hi = GridPoint {0, i, j, k + 1}.strokeMax();
    TEST_ASSERT_TRUE(lo != hi);
    TEST_ASSERT_FLOAT_WITHIN(
        1e-4f * std::fabs(lo), lo + 0.25f * (hi - lo),
        gait_speed_detail::strokeMaxAt(0, gridValue(i), gridValue(j), gridValue(k) + 0.0625f, Stance::DEFAULT));
}

void test_lookup_clips_inputs_and_is_empty_when_unwalkable() {
    const AxisEnvelope edge = envelopeAt(GaitType::TRI_GATE, 0, -1.0f, 0.0f, 0.0f, 0, Stance::DEFAULT);
    const AxisEnvelope beyond = envelopeAt(GaitType::TRI_GATE, 0, -4.0f, 0.0f, 0.0f, 0, Stance::DEFAULT);
    TEST_ASSERT_TRUE(edge.vMax > 0.0f);
    TEST_ASSERT_EQUAL_FLOAT(edge.vMax, beyond.vMax);
    TEST_ASSERT_EQUAL_FLOAT(edge.stroke, beyond.stroke);
    TEST_ASSERT_EQUAL_FLOAT(edge.cadence, beyond.cadence);
    TEST_ASSERT_EQUAL_FLOAT(envelopeAt(GaitType::TRI_GATE, 0, 0, 0, 0, 1, Stance::DEFAULT).stroke,
                            envelopeAt(GaitType::TRI_GATE, 0, 0, 0, 0, 5, Stance::DEFAULT).stroke);
    TEST_ASSERT_EQUAL_FLOAT(envelopeAt(GaitType::TRI_GATE, 0, 0, 0, 0, -1, Stance::DEFAULT).stroke,
                            envelopeAt(GaitType::TRI_GATE, 0, 0, 0, 0, -5, Stance::DEFAULT).stroke);

    for (const float stride : STRIDES) {
        const AxisEnvelope none = envelopeAt(GaitType::TRI_GATE, 0, -1.0f, -1.0f, 1.0f, stride, Stance::DEFAULT);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, none.vMax);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, none.stroke);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, none.cadence);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, gait_speed_detail::strokeMaxAt(0, -1.0f, -1.0f, 1.0f, Stance::DEFAULT));
}

void test_walkable_needs_every_corner_of_the_cell() {
    // Counted independently of postureWalkable, from the cell centre so that no corner is favoured.
    int open = 0, partial = 0;
    for (Stance feet : STANCES)
        for (int i = 0; i < gait_envelope::GRID_N - 1; ++i)
            for (int j = 0; j < gait_envelope::GRID_N - 1; ++j)
                for (int k = 0; k < gait_envelope::GRID_N - 1; ++k) {
                    int walkable = 0;
                    for (int c = 0; c < 8; ++c)
                        walkable += gait_envelope::WALKABLE[(int)feet][i + (c & 1)][j + ((c >> 1) & 1)][k + (c >> 2)];
                    const bool all =
                        postureWalkable(gridValue(i) + 0.125f, gridValue(j) + 0.125f, gridValue(k) + 0.125f, feet);
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
    for (Stance feet : STANCES)
        for (int i = 0; i <= 10; ++i)
            for (int j = 0; j <= 10; ++j) {
                const float h = -1.0f + 0.2f * (float)i, fd = -1.0f + 0.2f * (float)j;
                float a = h, b = fd;
                clampStandPosture(kin, a, b, feet);
                if (standFeasible(kin, h, fd, feet)) {
                    TEST_ASSERT_EQUAL_FLOAT(h, a);
                    TEST_ASSERT_EQUAL_FLOAT(fd, b);
                    ++kept;
                    continue;
                }
                TEST_ASSERT_TRUE(standFeasible(kin, a, b, feet));
                TEST_ASSERT_TRUE(std::fabs(a) <= std::fabs(h) && std::fabs(b) <= std::fabs(fd));
                TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, a * fd - b * h);
                const float scale = std::max(std::fabs(a), std::fabs(b)) / std::max(std::fabs(h), std::fabs(fd));
                for (int step = 1; step <= 32; ++step) {
                    const float t = (float)step / 32;
                    if (t > scale + 1e-4f) TEST_ASSERT_FALSE(standFeasible(kin, h * t, fd * t, feet));
                }
                ++pulled;
            }
    TEST_ASSERT_TRUE(kept > 0 && pulled > 0);
}

// The crab: every foot on the line through its coxa mount across the body (the lateral x axis), outward, as far from
// the mount as the default stance's foot at the same scale and height, so each coxa turns its leg straight out to its
// side and the femur and tibia stand as in the default stance.
void test_the_crab_points_every_leg_straight_out_at_the_default_stance_s_reach() {
    Kinematics kin;
    const float coxa[6] = {-45.0f, 0.0f, 45.0f, 45.0f, 0.0f, -45.0f};
    for (const float scale : {0.75f, 1.0f, 1.25f}) {
        BodyStateMsg standing {}, crab {};
        stanceFeet(Stance::DEFAULT, scale, standing.feet);
        stanceFeet(Stance::CRAB, scale, crab.feet);
        for (int i = 0; i < 6; ++i) {
            const float mx = hexapodConfig.mountX[i], my = hexapodConfig.mountY[i];
            TEST_ASSERT_EQUAL_FLOAT(DEFAULT_STANCE[i][0] * scale, standing.feet[i][0]);
            TEST_ASSERT_EQUAL_FLOAT(DEFAULT_STANCE[i][1] * scale, standing.feet[i][1]);
            TEST_ASSERT_EQUAL_FLOAT(DEFAULT_STANCE[i][2], crab.feet[i][2]);
            TEST_ASSERT_EQUAL_FLOAT(1.0f, crab.feet[i][3]);
            TEST_ASSERT_EQUAL_FLOAT(my, crab.feet[i][1]);
            TEST_ASSERT_TRUE((crab.feet[i][0] - mx) * mx > 0.0f);
            TEST_ASSERT_FLOAT_WITHIN(1e-4f, std::hypot(standing.feet[i][0] - mx, standing.feet[i][1] - my),
                                     std::hypot(crab.feet[i][0] - mx, crab.feet[i][1] - my));
        }
        float standingAngles[18], crabAngles[18];
        kin.inverseKinematics(standing, standingAngles);
        kin.inverseKinematics(crab, crabAngles);
        for (int leg = 0; leg < 6; ++leg) {
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, coxa[leg], crabAngles[leg * 3]);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, standingAngles[leg * 3 + 1], crabAngles[leg * 3 + 1]);
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, standingAngles[leg * 3 + 2], crabAngles[leg * 3 + 2]);
        }
    }
}

// A stance stands only with every leg pair LEG_CLEARANCE_MM apart. Neither stance comes near it at any posture (both
// keep at least 33 mm at rest), so the check is shown with kinematics whose legs 0 and 1 are mounted turned 45 deg away
// from each other: the IK turns their coxas 45 deg toward each other to reach the same feet, within travel and reach,
// and the legs as they are mounted cross.
void test_a_stance_whose_legs_touch_is_not_standable() {
    constexpr HexapodConfig TURNED = {{44.82, 61.03, 44.82, -44.82, -61.03, -44.82},
                                      {74.82, 0, -74.82, 74.82, 0, -74.82},
                                      {90, -45, -45, -225, -180, -135},
                                      0,
                                      38,
                                      54.06,
                                      97};
    Kinematics turned(TURNED);
    BodyStateMsg body {};
    stanceFeet(Stance::DEFAULT, feetScale(0), body.feet);
    float angles[18];
    turned.inverseKinematics(body, angles);
    for (int i = 0; i < 6; ++i) TEST_ASSERT_TRUE(turned.footReachable(body, i));
    for (int j = 0; j < 18; ++j) TEST_ASSERT_TRUE(std::fabs(angles[j]) <= JOINT_LIMIT_DEG[j % 3]);
    TEST_ASSERT_FALSE(legsClear(angles));

    Kinematics kin;
    TEST_ASSERT_TRUE(standFeasible(kin, 0, 0, Stance::DEFAULT));
    TEST_ASSERT_FALSE(standFeasible(turned, 0, 0, Stance::DEFAULT));
}

// The crab walks sideways: its table judges a posture walkable on its lateral stroke, which it strides at neutral,
// while its coxas, turned 45 deg at the corners, leave it a shorter forward stroke than a walk needs.
void test_the_crab_is_walkable_by_its_sideways_stroke() {
    using gait_speed_detail::strokeMaxAt;
    TEST_ASSERT_TRUE(postureWalkable(0, 0, 0, Stance::CRAB));
    TEST_ASSERT_TRUE(strokeMaxAt(1, 0, 0, 0, Stance::CRAB) >= 20.0f);
    TEST_ASSERT_TRUE(strokeMaxAt(0, 0, 0, 0, Stance::CRAB) < 20.0f);
    for (GaitType gait : WALKING_GAITS) {
        const WalkTarget sideways = classicWalkTarget(gait, 1, 0, 0, 1, 0, 0, 0, 1, Stance::CRAB);
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, strokeMaxAt(1, 0, 0, 0, Stance::CRAB), sideways.stepX);
        TEST_ASSERT_TRUE(sideways.phaseRate > 0.0f);
    }
    // The tables are the stance's own: the default stance strides further forward than the crab.
    TEST_ASSERT_TRUE(strokeMaxAt(0, 0, 0, 0, Stance::DEFAULT) > strokeMaxAt(0, 0, 0, 0, Stance::CRAB));
}

// A stroke cap scales every stroke of the walk alike, so a combined walk keeps its direction, and steps faster in
// proportion, so each axis keeps its speed, as long as the shorter strokes' top cadence allows; a walk within the cap
// is left as it is, and the capped axis is the cap itself, so a cap at the deadband keeps the gait commanded.
void test_a_stroke_cap_shortens_every_stroke_alike_and_keeps_the_speed() {
    for (const GaitType gait : WALKING_GAITS)
        for (int axis = 0; axis < 3; ++axis) {
            const WalkTarget w = classicWalkTarget(gait, 0.4f, 1.0f, 0.3f, -0.5f, 0, 0, 0, 0, Stance::DEFAULT);
            const float stroke = axisStroke(w, axis);
            TEST_ASSERT_TRUE(stroke > 0.0f);
            const WalkTarget same = withStrokeCap(gait, w, axis, stroke * 1.01f, 0, 0, 0, Stance::DEFAULT);
            TEST_ASSERT_EQUAL_FLOAT(w.stepX, same.stepX);
            TEST_ASSERT_EQUAL_FLOAT(w.stepY, same.stepY);
            TEST_ASSERT_EQUAL_FLOAT(w.stepAngle, same.stepAngle);
            TEST_ASSERT_EQUAL_FLOAT(w.phaseRate, same.phaseRate);

            const WalkTarget half = withStrokeCap(gait, w, axis, 0.5f * stroke, 0, 0, 0, Stance::DEFAULT);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * stroke, 0.5f * stroke, axisStroke(half, axis));
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * std::fabs(w.stepX), 0.5f * w.stepX, half.stepX);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * std::fabs(w.stepY), 0.5f * w.stepY, half.stepY);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * std::fabs(w.stepAngle), 0.5f * w.stepAngle, half.stepAngle);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * w.phaseRate, 2.0f * w.phaseRate, half.phaseRate);

            // Full speed at a hundredth of the stroke asks for more cadence than the gait has: it steps at the top
            // cadence of the shorter strokes, below the first sample that of the first sample, the slowest axis's.
            const WalkTarget fast = classicWalkTarget(gait, 0.4f, 1.0f, 0.3f, 1.0f, 0, 0, 0, 0, Stance::DEFAULT);
            const WalkTarget tiny =
                withStrokeCap(gait, fast, axis, 0.01f * axisStroke(fast, axis), 0, 0, 0, Stance::DEFAULT);
            float top = INFINITY;
            for (const GridPoint& point : NEUTRAL) top = std::min(top, point.cadenceAt(gait, 0.0f));
            TEST_ASSERT_TRUE(top < 100.0f * fast.phaseRate);
            TEST_ASSERT_FLOAT_WITHIN(1e-5f, top, tiny.phaseRate);

            const float deadband[3] = {GAIT_STRIDE_DEADBAND_MM, GAIT_STRIDE_DEADBAND_MM, GAIT_YAW_DEADBAND_RAD};
            const WalkTarget least = withStrokeCap(gait, w, axis, deadband[axis], 0, 0, 0, Stance::DEFAULT);
            TEST_ASSERT_EQUAL_FLOAT(deadband[axis], axisStroke(least, axis));
        }
}

namespace {

constexpr GaitType AUTO_GAITS[3] = {GaitType::RIPPLE, GaitType::TRI_GATE, GaitType::BI_GATE};

// The share of `gait`'s top speeds at `stride` the demand takes, summed over the axes like the stick; infinite when the
// gait cannot walk an axis the demand asks for.
float shareOfTop(GaitType gait, const AutoDemand& demand, float s1, float h, float fd, float stride) {
    float share = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
        if (demand.speed[axis] <= 0.0f) continue;
        const float vMax = envelopeAt(gait, axis, s1, h, fd, stride, Stance::DEFAULT).vMax;
        if (vMax <= 0.0f) return INFINITY;
        share += demand.speed[axis] / vMax;
    }
    return share;
}

// The body speed a walk target makes along an axis, in the axis's unit per second.
float walkedSpeed(GaitType gait, const WalkTarget& target, int axis) {
    return axisStroke(target, axis) * target.phaseRate / standFrac(gait);
}

} // namespace

// AUTO's speed signal is the stick's share of BI's top speed at the stride, the fastest classic gait's, whichever gait
// walks it.
void test_auto_demand_is_the_sticks_share_of_bi() {
    const float lx = -0.5f, ly = 1.0f, rx = 0.25f, s = 0.5f;
    const float length = std::fabs(lx) + std::fabs(ly) + std::fabs(rx), frac = speedFraction(s);
    const float stick[3] = {ly, lx, rx};
    for (const float stride : STRIDES) {
        const AutoDemand demand = autoDemand(lx, ly, rx, s, 0, 0, 0, stride);
        for (int axis = 0; axis < 3; ++axis) {
            const float expected = frac * std::fabs(stick[axis]) / length *
                                   envelopeAt(GaitType::BI_GATE, axis, 0, 0, 0, stride, Stance::DEFAULT).vMax;
            TEST_ASSERT_FLOAT_WITHIN(1e-5f * expected, expected, demand.speed[axis]);
        }
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, frac, demand.speed01);
    }
    // The stride changes BI's top speed, and the demand with it.
    TEST_ASSERT_TRUE(autoDemand(0, 1, 0, 1, 0, 0, 0, -1).speed[0] < autoDemand(0, 1, 0, 1, 0, 0, 0, 0).speed[0]);
}

// AUTO picks the slowest of RIPPLE, TRI and BI whose top speed at the stride covers the demand, and that gait walks the
// demand: exactly along one axis, and on a combined stick at most the demand on every axis, since the gait steps at
// the slowest axis's top cadence. The speed rises with the stick across both band edges. The old bands, set on the
// share of the gait in effect's own top speed, jumped from 21 to 103 mm/s at half stick.
void test_auto_walks_the_stick_monotonically_across_the_bands() {
    const float postures[][3] = {{0, 0, 0}, {1, -1, -1}, {-0.5f, 0.25f, 0.5f}};
    const float sticks[][3] = {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}, {0.4f, -1, 0.3f}};
    for (const auto& p : postures) {
        for (const auto& stick : sticks) {
            const bool single = (stick[0] != 0.0f) + (stick[1] != 0.0f) + (stick[2] != 0.0f) == 1;
            for (const float s : {0.0f, 1.0f}) {
                for (const float stride : {-1.0f, 0.0f, 1.0f}) {
                    float previous = 0.0f;
                    int switches = 0;
                    GaitType last = GaitType::RIPPLE;
                    for (int k = 1; k <= 200; ++k) {
                        const float scale = (float)k / 200;
                        const float lx = stick[0] * scale, ly = stick[1] * scale, rx = stick[2] * scale;
                        const AutoDemand demand = autoDemand(lx, ly, rx, s, p[0], p[1], p[2], stride);
                        // From a stop in RIPPLE, the gait AUTO stops into.
                        const GaitType gait = selectAutoGait(demand, GaitType::RIPPLE, p[0], p[1], p[2], stride);
                        TEST_ASSERT_TRUE(shareOfTop(gait, demand, p[0], p[1], p[2], stride) <= 1.0f + 1e-5f);
                        for (const GaitType slower : AUTO_GAITS) {
                            if (slower == gait) break;
                            const float margin = slower == GaitType::RIPPLE ? 1.0f : 1.0f - 0.06f;
                            TEST_ASSERT_TRUE(shareOfTop(slower, demand, p[0], p[1], p[2], stride) > margin - 1e-5f);
                        }
                        const WalkTarget target = autoWalkTarget(gait, demand, lx, ly, rx, p[0], p[1], p[2], stride);
                        float speed = 0.0f;
                        for (int axis = 0; axis < 3; ++axis) {
                            const float walked = walkedSpeed(gait, target, axis);
                            const float tolerance = 0.005f * demand.speed[axis] + 1e-5f;
                            if (single) TEST_ASSERT_FLOAT_WITHIN(tolerance, demand.speed[axis], walked);
                            TEST_ASSERT_TRUE(walked <= demand.speed[axis] + tolerance);
                            speed +=
                                walked /
                                envelopeAt(GaitType::BI_GATE, axis, p[0], p[1], p[2], stride, Stance::DEFAULT).vMax;
                        }
                        // A combined stick past an L1 norm of 1 asks for the same speed from there on.
                        TEST_ASSERT_TRUE(speed >= previous - 1e-5f);
                        previous = speed;
                        switches += gait != last;
                        last = gait;
                    }
                    TEST_ASSERT_TRUE(switches >= 1);
                }
            }
        }
    }
}

// A gait other than the one in effect must cover the demand with 6 % to spare, so a demand at a gait's top does not
// flap between two schedules; the gait in effect keeps a demand up to its own top.
void test_auto_resists_flapping_at_a_band_edge() {
    for (const float stride : {-1.0f, 0.0f, 1.0f}) {
        const float rippleTop = envelopeAt(GaitType::RIPPLE, 0, 0, 0, 0, stride, Stance::DEFAULT).vMax;
        const float biTop = envelopeAt(GaitType::BI_GATE, 0, 0, 0, 0, stride, Stance::DEFAULT).vMax;
        const auto atShareOfRipple = [&](float share) {
            return autoDemand(0, share * rippleTop / biTop, 0, 1, 0, 0, 0, stride);
        };
        const auto select = [&](float share, GaitType current) {
            return selectAutoGait(atShareOfRipple(share), current, 0, 0, 0, stride);
        };
        TEST_ASSERT_EQUAL(GaitType::RIPPLE, select(0.97f, GaitType::RIPPLE));
        TEST_ASSERT_EQUAL(GaitType::TRI_GATE, select(0.97f, GaitType::TRI_GATE));
        TEST_ASSERT_EQUAL(GaitType::RIPPLE, select(0.9f, GaitType::TRI_GATE));
        TEST_ASSERT_EQUAL(GaitType::TRI_GATE, select(1.03f, GaitType::RIPPLE));
        TEST_ASSERT_EQUAL(GaitType::RIPPLE,
                          selectAutoGait(autoDemand(0, 0, 0, 1, 0, 0, 0, stride), GaitType::BI_GATE, 0, 0, 0, stride));
    }
}

// The gait changes only at a stop, so a stick pushed past the top of the gait in effect walks at that gait's top until
// then: the demand's shares of the gait's top speeds scaled to sum to 1, each axis taking that share of its stroke at
// the slowest axis's top cadence.
void test_auto_caps_a_demand_above_the_gaits_top() {
    for (const float stride : STRIDES) {
        const AutoDemand demand = autoDemand(1, 1, 1, 1, 0, 0, 0, stride);
        const float need = shareOfTop(GaitType::RIPPLE, demand, 0, 0, 0, stride);
        TEST_ASSERT_TRUE(need > 2.0f);
        const WalkTarget capped = autoWalkTarget(GaitType::RIPPLE, demand, 1, 1, 1, 0, 0, 0, stride);
        TEST_ASSERT_TRUE(capped.stepX > 0.0f && capped.stepY > 0.0f && capped.stepAngle < 0.0f);
        float slowest = INFINITY;
        for (int axis = 0; axis < 3; ++axis) {
            const AxisEnvelope e = envelopeAt(GaitType::RIPPLE, axis, 0, 0, 0, stride, Stance::DEFAULT);
            const float share = demand.speed[axis] / e.vMax / need;
            TEST_ASSERT_FLOAT_WITHIN(1e-4f * share, share, axisStroke(capped, axis) / e.stroke);
            TEST_ASSERT_TRUE(walkedSpeed(GaitType::RIPPLE, capped, axis) <= demand.speed[axis] / need * 1.0001f);
            slowest = std::min(slowest, e.cadence);
        }
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, slowest, capped.phaseRate);
    }
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_the_stride_anchors_the_stroke);
    RUN_TEST(test_the_envelope_follows_the_tables_on_the_grid);
    RUN_TEST(test_kappa_is_linear_between_samples_and_flat_below_them);
    RUN_TEST(test_a_dip_in_the_table_reads_as_the_running_maximum);
    RUN_TEST(test_full_stick_reaches_the_top_speed);
    RUN_TEST(test_default_speed_is_sixty_percent);
    RUN_TEST(test_a_slower_stick_keeps_the_stroke_and_slows_the_cadence);
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
    RUN_TEST(test_interpolation_between_grid_points_is_trilinear);
    RUN_TEST(test_lookup_clips_inputs_and_is_empty_when_unwalkable);
    RUN_TEST(test_walkable_needs_every_corner_of_the_cell);
    RUN_TEST(test_the_crab_points_every_leg_straight_out_at_the_default_stance_s_reach);
    RUN_TEST(test_a_stance_whose_legs_touch_is_not_standable);
    RUN_TEST(test_the_crab_is_walkable_by_its_sideways_stroke);
    RUN_TEST(test_a_stroke_cap_shortens_every_stroke_alike_and_keeps_the_speed);
    RUN_TEST(test_auto_demand_is_the_sticks_share_of_bi);
    RUN_TEST(test_auto_walks_the_stick_monotonically_across_the_bands);
    RUN_TEST(test_auto_resists_flapping_at_a_band_edge);
    RUN_TEST(test_auto_caps_a_demand_above_the_gaits_top);
    return UNITY_END();
}
