#pragma once

#include <algorithm>
#include <cmath>
#include <gait_climb.h>
#include <gait_envelope.h>
#include <kinematics.h>
#include <leg_clearance.h>
#include <message_types.h>
#include <servo_travel.h>

// Stick -> walk command for the classic gaits, from the stroke geometry the generator measured (gait_envelope.h), one
// table per stance. The stride picks each axis's stroke, from the longest the joints can travel with the legs apart
// and the one that walks fastest, and the servos' peak joint rate over that stroke bounds its cadence. The stick is a
// fraction of that cadence, so a slower command keeps its stroke and steps it more slowly, and no command asks for more
// than the joints and servos can follow. Posture commands are pulled into the stance's walkable region by clampPosture,
// and stand postures into the standable one by clampStandPosture.

static_assert(gait_envelope::CLEARANCE_MM == LEG_CLEARANCE_MM,
              "gait_envelope.h keeps another leg clearance than leg_clearance.h: run simulation/export_envelope.py");
static_assert(
    gait_envelope::COXA_RADIUS_MM == leg_clearance::COXA_RADIUS_MM &&
        gait_envelope::FEMUR_RADIUS_MM == leg_clearance::FEMUR_RADIUS_MM &&
        gait_envelope::TIBIA_RADIUS_MM == leg_clearance::TIBIA_RADIUS_MM &&
        gait_envelope::FOOT_RADIUS_MM == leg_clearance::FOOT_RADIUS_MM,
    "gait_envelope.h was judged with other leg radii than leg_clearance.h's: run simulation/export_envelope.py");
static_assert(gait_envelope::STANCES == (int)Stance::CRAB + 1, "gait_envelope.h has a table per Stance");

// The least distance from the body centre to the nearest edge of the feet in stance that every classic stroke keeps on
// the statically stable timings (TRI_GATE, WAVE, RIPPLE), the owner's ruling of 2026-11-01; the generator applies it.
constexpr float STATIC_MARGIN_MM = 25.0f;
static_assert(gait_envelope::STATIC_MARGIN_MM == STATIC_MARGIN_MM,
              "gait_envelope.h keeps another static margin than gait_speed.h: run simulation/export_envelope.py");
static_assert(gait_climb::STROKE_SAMPLES == gait_envelope::STROKE_SAMPLES &&
                  gait_climb::TRAVEL_MARGIN_DEG == gait_envelope::TRAVEL_MARGIN_DEG &&
                  gait_climb::CLEARANCE_MM == gait_envelope::CLEARANCE_MM &&
                  gait_climb::STATIC_MARGIN_MM == gait_envelope::STATIC_MARGIN_MM,
              "gait_climb.h was measured by other checks than gait_envelope.h: run simulation/export_climb.py");

struct AxisEnvelope {
    float vMax;    // mm/s (forward, lateral) or rad/s (yaw): the stroke at its top cadence
    float stroke;  // mm or rad per cycle
    float cadence; // cycles/s, the top the servos follow at this stroke
};

// The stroke at stride -1, as a share of the speed-optimal one at stride 0.
constexpr float MIN_STRIDE_SHARE = 0.3f;

// The speed-optimal stroke is the shortest sample within this share of the fastest sample's speed. Where the servos
// bind, kappa grows with the stroke and several samples walk within 1e-4 of each other, which float and the
// simulation's double order differently; a shorter stroke that walks as fast is the better step, and a share a
// thousand times float's 1e-6 rounding of the lookup makes it unlikely, though not impossible, that the firmware and
// the port pick different samples: a sample whose speed falls within float's rounding of the share's edge can still
// split them.
constexpr float SPEED_TIE = 1e-3f;

namespace gait_speed_detail {

constexpr int CELLS = gait_envelope::GRID_N - 1;
constexpr float CELLS_PER_UNIT = CELLS / 2.0f;
constexpr int SAMPLES = gait_envelope::STROKE_SAMPLES;

inline bool isClassic(GaitType gait) { return (int)gait >= 0 && (int)gait <= (int)GaitType::RIPPLE; }

struct Cell {
    int index;
    float frac;
};

// A grid point belongs to the cell above it, except the last, which closes the cell below.
inline Cell locate(float command) {
    const float pos = (std::clamp(command, -1.0f, 1.0f) + 1.0f) * CELLS_PER_UNIT;
    const int index = std::min((int)pos, CELLS - 1);
    return {index, pos - (float)index};
}

inline bool cellWalkable(const Cell& s1, const Cell& h, const Cell& fd, Stance stance) {
    const auto& walkable = gait_envelope::WALKABLE[(int)stance];
    for (int corner = 0; corner < 8; ++corner) {
        if (!walkable[s1.index + (corner & 1)][h.index + ((corner >> 1) & 1)][fd.index + (corner >> 2)]) return false;
    }
    return true;
}

// `at(i, j, k)` is the table's value at grid point [i][j][k].
template <typename At>
float trilinear(const Cell& s1, const Cell& h, const Cell& fd, At at) {
    const auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
    float along_fd[2][2];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            along_fd[i][j] =
                lerp(at(s1.index + i, h.index + j, fd.index), at(s1.index + i, h.index + j, fd.index + 1), fd.frac);
    return lerp(lerp(along_fd[0][0], along_fd[0][1], h.frac), lerp(along_fd[1][0], along_fd[1][1], h.frac), s1.frac);
}

// The longest stroke an axis's joints can travel at the posture in the stance, 0 where the posture cannot walk.
// Walkability gates the lookup: some grid points outside the walkable cells carry a stroke, so the table is read only
// in a cell whose eight corners all walk.
inline float strokeMaxAt(int axis, float s1, float h, float fd, Stance stance) {
    const Cell cs1 = locate(s1), ch = locate(h), cfd = locate(fd);
    if (axis < 0 || axis > 2 || !cellWalkable(cs1, ch, cfd, stance)) return 0.0f;
    const auto& strokeMax = gait_envelope::STROKE_MAX[(int)stance][axis];
    return trilinear(cs1, ch, cfd, [&](int i, int j, int k) { return strokeMax[i][j][k]; });
}

// Kappa at a stroke, as a fraction of the longest, from its samples at (k + 1) / SAMPLES: the running maximum of the
// samples up to the stroke, linear between them, and the first sample below it. Kappa mostly rises with the stroke but
// the swing samples dip in places, below the highest before them by up to 1.9 % forward, 1.9 % lateral and 18.1 % in
// yaw in the default stance and 5.1 % forward, 5.2 % lateral and 1.3 % in yaw in the crab, and a stroke inside a dip
// or short of the first sample is not taken to turn the joints any slower than a shorter one.
inline float kappaAtStroke(const float (&samples)[SAMPLES], float fraction) {
    const float position = fraction * SAMPLES - 1.0f;
    float below = samples[0];
    // Written so that a NaN stroke reads the first sample rather than an index far outside the samples.
    if (!(position > 0.0f)) return below;
    const int index = std::min((int)position, SAMPLES - 2);
    for (int k = 1; k <= index; ++k) below = std::max(below, samples[k]);
    const float above = std::max(below, samples[index + 1]);
    return below + (above - below) * (position - (float)index);
}

// The fastest cadence at which no joint passes the servo's speed: a stroke's peak joint rate per unit of swing and of
// stance progress, over the share of the cycle each takes, capped where the generator stopped measuring.
inline float topCadence(GaitType gait, float kappaSwing, float kappaStance) {
    const float d = gait_envelope::STAND_FRAC[(int)gait];
    return std::min(gait_envelope::SERVO_RAD_S / std::max(kappaSwing / (1.0f - d), kappaStance / d),
                    gait_envelope::CADENCE_MAX);
}

// An axis's kappa samples at the posture, interpolated between the grid points.
struct Kappas {
    float swing[SAMPLES], stance[SAMPLES];
};

inline Kappas kappasAt(int axis, const Cell& s1, const Cell& h, const Cell& fd, Stance feet) {
    const auto& kappaSwing = gait_envelope::KAPPA_SWING[(int)feet][axis];
    const auto& kappaStance = gait_envelope::KAPPA_STANCE[(int)feet][axis];
    Kappas kappas;
    for (int n = 0; n < SAMPLES; ++n) {
        kappas.swing[n] = trilinear(s1, h, fd, [&](int i, int j, int k) { return kappaSwing[i][j][k][n]; });
        kappas.stance[n] = trilinear(s1, h, fd, [&](int i, int j, int k) { return kappaStance[i][j][k][n]; });
    }
    return kappas;
}

} // namespace gait_speed_detail

// True when the posture sits in a cell whose eight corners can all walk in the stance.
inline bool postureWalkable(float s1, float h, float fd, Stance stance) {
    using namespace gait_speed_detail;
    return cellWalkable(locate(s1), locate(h), locate(fd), stance);
}

// The stroke `stride` picks for a classic gait on an axis whose longest stroke is `strokeMax` and whose kappa samples
// are `swing` and `stance`, with its top cadence and the top speed the two make (envelopeAt).
inline AxisEnvelope envelopeOf(GaitType gait, float strokeMax, const float (&swing)[gait_speed_detail::SAMPLES],
                               const float (&stance)[gait_speed_detail::SAMPLES], float stride) {
    using namespace gait_speed_detail;
    float speed[SAMPLES], fastest = 0.0f;
    for (int n = 0; n < SAMPLES; ++n) {
        const float fraction = (float)(n + 1) / SAMPLES;
        // The top speed is stroke * cadence / d, and d is the gait's alone.
        speed[n] = (float)(n + 1) * topCadence(gait, kappaAtStroke(swing, fraction), kappaAtStroke(stance, fraction));
        fastest = std::max(fastest, speed[n]);
    }
    int optimal = 0;
    while (speed[optimal] < fastest * (1.0f - SPEED_TIE)) ++optimal;
    const float optimalStroke = (float)(optimal + 1) / SAMPLES * strokeMax;
    stride = std::clamp(stride, -1.0f, 1.0f);
    const float stroke = stride >= 0.0f ? optimalStroke + stride * (strokeMax - optimalStroke)
                                        : optimalStroke * (1.0f + stride * (1.0f - MIN_STRIDE_SHARE));
    const float fraction = stroke / strokeMax;
    const float cadence = topCadence(gait, kappaAtStroke(swing, fraction), kappaAtStroke(stance, fraction));
    return {stroke * cadence / gait_envelope::STAND_FRAC[(int)gait], stroke, cadence};
}

// The stroke `stride` picks on an axis (0 forward, 1 lateral, 2 yaw) at a posture in a stance, with its top cadence and
// the top speed the two make. Stride 0 is the speed-optimal stroke, the shortest stroke sample whose top speed is
// within SPEED_TIE of the highest, +1 the longest stroke, and -1 MIN_STRIDE_SHARE of the speed-optimal one, linear in
// between. The envelope is empty where the posture cannot walk the axis, and for GaitType::TUNED/AUTO.
inline AxisEnvelope envelopeAt(GaitType gait, int axis, float s1, float h, float fd, float stride, Stance feet) {
    using namespace gait_speed_detail;
    if (!isClassic(gait)) return {};
    const float strokeMax = strokeMaxAt(axis, s1, h, fd, feet);
    if (strokeMax <= 0.0f) return {};
    const Kappas kappas = kappasAt(axis, locate(s1), locate(h), locate(fd), feet);
    return envelopeOf(gait, strokeMax, kappas.swing, kappas.stance, stride);
}

// The climb walk's envelope on an axis: envelopeAt at the climb's one posture (gait_climb.h), in the default stance.
inline AxisEnvelope climbEnvelopeAt(GaitType gait, int axis, float stride) {
    if (!gait_speed_detail::isClassic(gait) || axis < 0 || axis > 2) return {};
    return envelopeOf(gait, gait_climb::STROKE_MAX[axis], gait_climb::KAPPA_SWING[axis], gait_climb::KAPPA_STANCE[axis],
                      stride);
}

// Scales a posture's deviation from the neutral one along its own direction, to the outermost point `feasible`
// accepts. A feasible region need not be star-shaped about neutral, so a plain bisection can settle on an inner
// boundary; a coarse scan from the outside finds the outermost feasible step first, and the bisection refines the
// boundary above it.
template <typename Feasible>
void clampAlongRay(float& s1, float& h, float& fd, Feasible feasible) {
    constexpr int SCAN_STEPS = 32;
    constexpr int REFINE_STEPS = 5;
    s1 = std::clamp(s1, -1.0f, 1.0f);
    h = std::clamp(h, -1.0f, 1.0f);
    fd = std::clamp(fd, -1.0f, 1.0f);
    if (feasible(s1, h, fd)) return;
    int step = SCAN_STEPS - 1;
    while (step > 0 && !feasible(s1 * step / SCAN_STEPS, h * step / SCAN_STEPS, fd * step / SCAN_STEPS)) --step;
    float inside = (float)step / SCAN_STEPS, outside = (float)(step + 1) / SCAN_STEPS;
    for (int i = 0; i < REFINE_STEPS; ++i) {
        const float mid = 0.5f * (inside + outside);
        (feasible(s1 * mid, h * mid, fd * mid) ? inside : outside) = mid;
    }
    s1 *= inside;
    h *= inside;
    fd *= inside;
}

// Pulls a posture the stance cannot walk into its walkable region, which keeps the legs apart: a walkable grid point
// strides with every leg pair LEG_CLEARANCE_MM apart over every classic timing's cycle, and both resting stances keep
// more than 30 mm at every posture (simulation/test_leg_clearance.py).
inline void clampPosture(float& s1, float& h, float& fd, Stance stance) {
    clampAlongRay(s1, h, fd, [stance](float a, float b, float c) { return postureWalkable(a, b, c, stance); });
}

// What the posture sliders set, as the envelope was measured: the body height and the stance width.
inline float bodyHeightMm(float h) { return h * 50.0f; }
inline float feetScale(float fd) { return 0.75f + (std::clamp(fd, -1.0f, 1.0f) + 1.0f) * 0.25f; }

// The neutral stance, the one fd = 0 stands on and the envelope's feet scale multiplies.
constexpr float DEFAULT_STANCE[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                        {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

// A stance's resting feet at a feet scale. The default stance is DEFAULT_STANCE with x and y scaled. The crab puts each
// foot on the line through its coxa mount across the body, the lateral x axis, outward, at the reach from the mount the
// default stance has at the same scale and height, so every leg points straight out to its side and the femur and tibia
// stand as in the default stance. Ported as stance_feet in simulation/src/robot/firmware_gait.py.
inline void stanceFeet(Stance stance, float scale, float out[6][4]) {
    for (int i = 0; i < 6; ++i) {
        out[i][0] = DEFAULT_STANCE[i][0] * scale;
        out[i][1] = DEFAULT_STANCE[i][1] * scale;
        out[i][2] = DEFAULT_STANCE[i][2];
        out[i][3] = DEFAULT_STANCE[i][3];
        if (stance == Stance::CRAB) {
            const float mountX = hexapodConfig.mountX[i], mountY = hexapodConfig.mountY[i];
            out[i][0] = mountX + std::copysign(std::hypot(out[i][0] - mountX, out[i][1] - mountY), mountX);
            out[i][1] = mountY;
        }
    }
}

// Whether the body can stand level and unshifted at height h over the stance's feet at fd: every foot within reach,
// every joint within travel and every leg pair LEG_CLEARANCE_MM apart. Standing asks less than walking, so a stand may
// take postures a walk cannot.
inline bool standFeasible(Kinematics& kin, float h, float fd, Stance stance) {
    BodyStateMsg body {};
    body.zm = bodyHeightMm(h);
    stanceFeet(stance, feetScale(fd), body.feet);
    for (int i = 0; i < 6; ++i)
        if (!kin.footReachable(body, i)) return false;
    float angles[18];
    kin.inverseKinematics(body, angles);
    for (int j = 0; j < 18; ++j)
        if (std::fabs(angles[j]) > JOINT_LIMIT_DEG[j / 3][j % 3]) return false;
    return legsClear(angles);
}

// The largest coxa angle the crab walks a corner coxa to on level ground, measured on its walks: the forward and
// turning strokes of its table, alone and combined, at the speed-optimal and the longest stride, in every classic
// timing at the neutral posture and the corners of the posture cube, take a corner coxa out to 60.96 deg from the 45 it
// rests at, the static margin ending most forward strokes and the legs' clearance the turning ones, not the coxa's
// 90 deg travel (test_stance holds the walks within half a degree under it). The self-levelling's tilt adds to it on a
// slope, held at the calibration's travel by the servo controller. Regenerating the envelope can move it.
constexpr float CRAB_COXA_MAX_DEG = 61.0f;

// Each coxa's angle in the crab at rest, the same at every posture, since each foot keeps the default stance's reach:
// the corner legs' at +-45 deg, the middle legs' at 0.
inline void crabRestCoxas(Kinematics& kin, float out[6]) {
    BodyStateMsg body {};
    stanceFeet(Stance::CRAB, feetScale(0.0f), body.feet);
    float angles[18];
    kin.inverseKinematics(body, angles);
    for (int leg = 0; leg < 6; ++leg) out[leg] = angles[leg * 3];
}

// Whether each corner coxa, within `travel`, its own calibration's (ServoController::coxaTravelOf), turns out to the
// crab's largest angle on the side the crab points it: one that cannot would rest the crab off its line, and drag its
// feet. `rest` is crabRestCoxas'.
inline bool crabWithinTravel(const float (&rest)[6], const JointTravel (&travel)[6]) {
    for (int leg = 0; leg < 6; ++leg) {
        if (rest[leg] > 1.0f && travel[leg].max < CRAB_COXA_MAX_DEG) return false;
        if (rest[leg] < -1.0f && -travel[leg].min < CRAB_COXA_MAX_DEG) return false;
    }
    return true;
}

// Pulls a posture the body cannot stand in on the stance toward neutral; the lift does not enter a stand.
inline void clampStandPosture(Kinematics& kin, float& h, float& fd, Stance stance) {
    float s1 = 0.0f;
    clampAlongRay(s1, h, fd, [&](float, float height, float feet) { return standFeasible(kin, height, feet, stance); });
}

// Fraction of the top speed for the speed input s: stop at -1, the default at 0, the top at +1.
inline float speedFraction(float s) {
    s = std::clamp(s, -1.0f, 1.0f);
    const float def = gait_envelope::DEFAULT_SPEED_FRACTION;
    return s < 0.0f ? def * (1.0f + s) : def + (1.0f - def) * s;
}

struct WalkTarget {
    float stepX, stepY, stepAngle; // mm, mm, rad per cycle, in the body frame (docs/conventions.md)
    float phaseRate;               // cycles/s
};

namespace gait_speed_detail {

// The stick as a share of each axis's top speed, in envelope order (forward, lateral, yaw): |stick| over the stick's
// L1 norm, or over 1 when the norm is smaller, times the speed fraction.
inline void stickShares(float lx, float ly, float rx, float s, float (&share)[3]) {
    const float stick[3] = {ly, lx, rx};
    const float length = std::max(1.0f, std::fabs(lx) + std::fabs(ly) + std::fabs(rx));
    for (int axis = 0; axis < 3; ++axis) share[axis] = speedFraction(s) * std::fabs(stick[axis]) / length;
}

// The walk at share[axis] of each axis's top speed at the stride, the shares summing to at most 1, signed as lx and
// ly on its lateral and forward axes and against rx on its yaw.
// Each axis walks its weight, its share over the total share, of its stroke, the weights summing to 1, and a cycle
// takes the weighted sum of the axes' own top cycle times: the gait steps at the total share of the weighted harmonic
// mean of their top cadences. To first order a foot's motion is the weighted sum of the motions each axis's stroke
// would give it, so a joint turns at most at the weighted sum of the axes' peak rates per cycle (kappa), which that
// cadence keeps within the servo's speed; test_virtual_robot measures the combined sticks' joint steps on the
// firmware's own walk. The axes' speeds as shares of their top speeds sum to the total share, the faster axis below its
// own share and the slower one above it, and an axis of small weight, as a goal's cross-track correction, slows the
// others by about its weight alone.
// `envelope(axis)` is the axis's AxisEnvelope.
template <typename Envelope>
WalkTarget walkAtSharesOf(const float (&share)[3], float lx, float ly, float rx, Envelope envelope) {
    float used[3] = {}; // the shares of the axes the posture can walk
    float stroke[3] = {}, cadence[3] = {};
    float total = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
        if (share[axis] <= 0.0f) continue;
        const AxisEnvelope e = envelope(axis);
        // Defensive: a walkable posture promises only the axis its stance walks, though no walkable cell of either
        // stance's table has an empty one.
        if (e.stroke <= 0.0f) continue;
        used[axis] = share[axis];
        stroke[axis] = e.stroke;
        cadence[axis] = e.cadence;
        total += used[axis];
    }
    if (total <= 0.0f) return {};
    float cycleTime = 0.0f; // s per cycle at the axes' own top cadences, weighted
    for (int axis = 0; axis < 3; ++axis) {
        if (used[axis] <= 0.0f) continue;
        const float weight = used[axis] / total;
        stroke[axis] *= weight;
        cycleTime += weight / cadence[axis];
    }
    // +lx strafes right (+x), +ly walks forward (+y) and +rx turns clockwise (negative step_angle).
    return {std::copysign(stroke[1], lx), std::copysign(stroke[0], ly), -std::copysign(stroke[2], rx),
            total / cycleTime};
}

inline WalkTarget walkAtShares(GaitType gait, const float (&share)[3], float lx, float ly, float rx, float s1, float h,
                               float fd, float stride, Stance stance) {
    return walkAtSharesOf(share, lx, ly, rx,
                          [&](int axis) { return envelopeAt(gait, axis, s1, h, fd, stride, stance); });
}

} // namespace gait_speed_detail

inline WalkTarget classicWalkTarget(GaitType gait, float lx, float ly, float rx, float s, float s1, float h, float fd,
                                    float stride, Stance stance) {
    using namespace gait_speed_detail;
    if (!isClassic(gait)) return {};
    float share[3];
    stickShares(lx, ly, rx, s, share);
    return walkAtShares(gait, share, lx, ly, rx, s1, h, fd, stride, stance);
}

// classicWalkTarget at the climb's one posture (gait_climb.h), in the default stance.
inline WalkTarget climbWalkTarget(GaitType gait, float lx, float ly, float rx, float s, float stride) {
    using namespace gait_speed_detail;
    if (!isClassic(gait)) return {};
    float share[3];
    stickShares(lx, ly, rx, s, share);
    return walkAtSharesOf(share, lx, ly, rx, [&](int axis) { return climbEnvelopeAt(gait, axis, stride); });
}

// The walk `w` with every stroke scaled down alike until its stroke along `axis` (0 forward and 1 lateral in mm, 2 yaw
// in rad) is the positive `cap`, and stepped faster in proportion, so each axis keeps its speed, up to the shorter
// strokes' top cadence. A goal's approach walks so, so that no step carries the robot past its target; a walk within
// the cap is left as it is. The capped axis is set to the cap itself, so a cap at the gait's deadband still steps.
inline WalkTarget withStrokeCap(GaitType gait, const WalkTarget& w, int axis, float cap, float s1, float h, float fd,
                                Stance stance) {
    using namespace gait_speed_detail;
    const float strokes[3] = {std::fabs(w.stepY), std::fabs(w.stepX), std::fabs(w.stepAngle)};
    if (!(strokes[axis] > cap)) return w;
    const float scale = cap / strokes[axis];
    float rate = w.phaseRate / scale;
    for (int a = 0; a < 3; ++a) {
        if (strokes[a] <= 0.0f) continue;
        const float strokeMax = strokeMaxAt(a, s1, h, fd, stance);
        const Kappas kappas = kappasAt(a, locate(s1), locate(h), locate(fd), stance);
        const float fraction = scale * strokes[a] / strokeMax;
        rate = std::min(
            rate, topCadence(gait, kappaAtStroke(kappas.swing, fraction), kappaAtStroke(kappas.stance, fraction)));
    }
    WalkTarget capped {w.stepX * scale, w.stepY * scale, w.stepAngle * scale, rate};
    float& alongAxis = axis == 0 ? capped.stepY : axis == 1 ? capped.stepX : capped.stepAngle;
    alongAxis = std::copysign(cap, alongAxis);
    return capped;
}

// AUTO walks the speed the stick asks of the fastest classic gait, BI: the stick's share of BI's top speed at the
// posture and stride, on each axis, whichever gait walks it. AUTO walks the default stance alone.
struct AutoDemand {
    float speed[3]; // forward and lateral in mm/s, yaw in rad/s
    float speed01;  // its share of BI's top speed, summed over the axes like the stick
};

inline AutoDemand autoDemand(float lx, float ly, float rx, float s, float s1, float h, float fd, float stride) {
    float share[3];
    gait_speed_detail::stickShares(lx, ly, rx, s, share);
    AutoDemand demand {};
    for (int axis = 0; axis < 3; ++axis) {
        const AxisEnvelope e = envelopeAt(GaitType::BI_GATE, axis, s1, h, fd, stride, Stance::DEFAULT);
        if (e.vMax <= 0.0f) continue;
        demand.speed[axis] = share[axis] * e.vMax;
        demand.speed01 += share[axis];
    }
    return demand;
}

namespace gait_speed_detail {

// The demand as a share of each of the gait's top speeds; false when the gait cannot walk an axis the demand asks for.
inline bool demandShares(GaitType gait, const AutoDemand& demand, float s1, float h, float fd, float stride,
                         float (&share)[3]) {
    for (int axis = 0; axis < 3; ++axis) {
        share[axis] = 0.0f;
        if (demand.speed[axis] <= 0.0f) continue;
        const AxisEnvelope e = envelopeAt(gait, axis, s1, h, fd, stride, Stance::DEFAULT);
        if (e.vMax <= 0.0f) return false;
        share[axis] = demand.speed[axis] / e.vMax;
    }
    return true;
}

} // namespace gait_speed_detail

// The slowest of AUTO's gaits, RIPPLE, TRI and BI, whose top speed at the stride covers the demand: the shares of its
// top speeds sum to at most 1, as the stick's do. A gait other than the one in effect must cover it with
// AUTO_HYSTERESIS to spare, so a demand near a gait's top does not flap between two schedules.
inline GaitType selectAutoGait(const AutoDemand& demand, GaitType current, float s1, float h, float fd, float stride) {
    constexpr float AUTO_HYSTERESIS = 0.06f;
    for (const GaitType gait : {GaitType::RIPPLE, GaitType::TRI_GATE}) {
        float share[3];
        if (!gait_speed_detail::demandShares(gait, demand, s1, h, fd, stride, share)) continue;
        const float limit = gait == current ? 1.0f : 1.0f - AUTO_HYSTERESIS;
        if (share[0] + share[1] + share[2] <= limit) return gait;
    }
    return GaitType::BI_GATE;
}

// The walk `gait` takes for AUTO's demand: the demand's share of the gait's own top speeds, capped at its top. An axis
// the gait cannot walk at the posture is dropped, as classicWalkTarget drops it.
inline WalkTarget autoWalkTarget(GaitType gait, const AutoDemand& demand, float lx, float ly, float rx, float s1,
                                 float h, float fd, float stride) {
    using namespace gait_speed_detail;
    if (!isClassic(gait)) return {};
    float share[3];
    for (int axis = 0; axis < 3; ++axis) {
        const float vMax = envelopeAt(gait, axis, s1, h, fd, stride, Stance::DEFAULT).vMax;
        share[axis] = vMax > 0.0f ? demand.speed[axis] / vMax : 0.0f;
    }
    const float total = share[0] + share[1] + share[2];
    if (total > 1.0f)
        for (float& axisShare : share) axisShare /= total;
    return walkAtShares(gait, share, lx, ly, rx, s1, h, fd, stride, Stance::DEFAULT);
}
