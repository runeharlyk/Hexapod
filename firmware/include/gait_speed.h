#pragma once

#include <algorithm>
#include <cmath>
#include <gait_envelope.h>
#include <kinematics.h>
#include <message_types.h>

// Stick -> walk command for the classic gaits, from the stroke geometry the generator measured (gait_envelope.h). The
// stride picks each axis's stroke, from the longest the joints can travel and the one that walks fastest, and the
// servos' peak joint rate over that stroke bounds its cadence. The stick is a fraction of that cadence, so a slower
// command keeps its stroke and steps it more slowly, and no command asks for more than the joints and servos can
// follow. Posture commands are pulled into the walkable region by clampPosture, and stand postures into the standable
// one by clampStandPosture.

struct AxisEnvelope {
    float vMax;    // mm/s (forward, lateral) or rad/s (yaw): the stroke at its top cadence
    float stroke;  // mm or rad per cycle
    float cadence; // cycles/s, the top the servos follow at this stroke
};

// The stroke at stride -1, as a share of the speed-optimal one at stride 0.
constexpr float MIN_STRIDE_SHARE = 0.3f;

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

inline bool cellWalkable(const Cell& s1, const Cell& h, const Cell& fd) {
    for (int corner = 0; corner < 8; ++corner) {
        if (!gait_envelope::WALKABLE[s1.index + (corner & 1)][h.index + ((corner >> 1) & 1)][fd.index + (corner >> 2)])
            return false;
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

// The longest stroke an axis's joints can travel at the posture, 0 where the posture cannot walk. Walkability gates the
// lookup: some grid points outside the walkable cells carry a stroke, so the table is read only in a cell whose eight
// corners all walk.
inline float strokeMaxAt(int axis, float s1, float h, float fd) {
    const Cell cs1 = locate(s1), ch = locate(h), cfd = locate(fd);
    if (axis < 0 || axis > 2 || !cellWalkable(cs1, ch, cfd)) return 0.0f;
    return trilinear(cs1, ch, cfd, [axis](int i, int j, int k) { return gait_envelope::STROKE_MAX[axis][i][j][k]; });
}

// Kappa at a stroke, as a fraction of the longest, from its samples at (k + 1) / SAMPLES: the running maximum of the
// samples up to the stroke, linear between them, and the first sample below it. Kappa mostly rises with the stroke but
// the swing samples dip in places, below the highest before them by up to 2.2 % forward and lateral and 15 % in yaw,
// and a stroke inside a dip or short of the first sample is not taken to turn the joints any slower than a shorter one.
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

} // namespace gait_speed_detail

// True when the posture sits in a cell whose eight corners can all walk.
inline bool postureWalkable(float s1, float h, float fd) {
    using namespace gait_speed_detail;
    return cellWalkable(locate(s1), locate(h), locate(fd));
}

// The stroke `stride` picks on an axis (0 forward, 1 lateral, 2 yaw) at a posture, with its top cadence and the top
// speed the two make. Stride 0 is the speed-optimal stroke, the stroke sample whose top speed is the highest, +1 the
// longest stroke, and -1 MIN_STRIDE_SHARE of the speed-optimal one, linear in between. The envelope is empty where the
// posture cannot walk the axis, and for GaitType::TUNED/AUTO.
inline AxisEnvelope envelopeAt(GaitType gait, int axis, float s1, float h, float fd, float stride) {
    using namespace gait_speed_detail;
    if (!isClassic(gait)) return {};
    const float strokeMax = strokeMaxAt(axis, s1, h, fd);
    if (strokeMax <= 0.0f) return {};
    const Cell cs1 = locate(s1), ch = locate(h), cfd = locate(fd);
    float swing[SAMPLES], stance[SAMPLES];
    int optimal = 0;
    float fastest = 0.0f;
    for (int n = 0; n < SAMPLES; ++n) {
        swing[n] =
            trilinear(cs1, ch, cfd, [&](int i, int j, int k) { return gait_envelope::KAPPA_SWING[axis][i][j][k][n]; });
        stance[n] =
            trilinear(cs1, ch, cfd, [&](int i, int j, int k) { return gait_envelope::KAPPA_STANCE[axis][i][j][k][n]; });
    }
    for (int n = 0; n < SAMPLES; ++n) {
        const float fraction = (float)(n + 1) / SAMPLES;
        // The top speed is stroke * cadence / d, and d is the gait's alone.
        const float speed =
            (float)(n + 1) * topCadence(gait, kappaAtStroke(swing, fraction), kappaAtStroke(stance, fraction));
        if (speed > fastest) {
            fastest = speed;
            optimal = n;
        }
    }
    const float optimalStroke = (float)(optimal + 1) / SAMPLES * strokeMax;
    stride = std::clamp(stride, -1.0f, 1.0f);
    const float stroke = stride >= 0.0f ? optimalStroke + stride * (strokeMax - optimalStroke)
                                        : optimalStroke * (1.0f + stride * (1.0f - MIN_STRIDE_SHARE));
    const float fraction = stroke / strokeMax;
    const float cadence = topCadence(gait, kappaAtStroke(swing, fraction), kappaAtStroke(stance, fraction));
    return {stroke * cadence / gait_envelope::STAND_FRAC[(int)gait], stroke, cadence};
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

// Pulls an unwalkable posture into the walkable region.
inline void clampPosture(float& s1, float& h, float& fd) { clampAlongRay(s1, h, fd, postureWalkable); }

// What the posture sliders set, as the envelope was measured: the body height and the stance width.
inline float bodyHeightMm(float h) { return h * 50.0f; }
inline float feetScale(float fd) { return 0.75f + (std::clamp(fd, -1.0f, 1.0f) + 1.0f) * 0.25f; }

// The neutral stance, the one fd = 0 stands on and the envelope's feet scale multiplies.
constexpr float DEFAULT_STANCE[6][4] = {{122, 152, -66, 1},  {171, 0, -66, 1},  {122, -152, -66, 1},
                                        {-122, 152, -66, 1}, {-171, 0, -66, 1}, {-122, -152, -66, 1}};

inline void scaleStance(float scale, float out[6][4]) {
    for (int i = 0; i < 6; ++i) {
        out[i][0] = DEFAULT_STANCE[i][0] * scale;
        out[i][1] = DEFAULT_STANCE[i][1] * scale;
        out[i][2] = DEFAULT_STANCE[i][2];
        out[i][3] = DEFAULT_STANCE[i][3];
    }
}

// Whether the body can stand level and unshifted at height h over the stance scaled by fd: every foot within reach
// and every joint within travel. Standing asks less than walking, so a stand may take postures a walk cannot.
inline bool standFeasible(Kinematics& kin, float h, float fd) {
    BodyStateMsg body {};
    body.zm = bodyHeightMm(h);
    scaleStance(feetScale(fd), body.feet);
    for (int i = 0; i < 6; ++i)
        if (!kin.footReachable(body, i)) return false;
    float angles[18];
    kin.inverseKinematics(body, angles);
    for (int j = 0; j < 18; ++j)
        if (std::fabs(angles[j]) > JOINT_LIMIT_DEG[j % 3]) return false;
    return true;
}

// Pulls a posture the body cannot stand in toward neutral; the lift does not enter a stand.
inline void clampStandPosture(Kinematics& kin, float& h, float& fd) {
    float s1 = 0.0f;
    clampAlongRay(s1, h, fd, [&](float, float height, float feet) { return standFeasible(kin, height, feet); });
}

// Fraction of the top speed for the speed input s: stop at -1, the default at 0, the top at +1.
inline float speedFraction(float s) {
    s = std::clamp(s, -1.0f, 1.0f);
    const float def = gait_envelope::DEFAULT_SPEED_FRACTION;
    return s < 0.0f ? def * (1.0f + s) : def + (1.0f - def) * s;
}

struct WalkTarget {
    float stepX, stepY, stepAngle; // mm, mm, rad per cycle, signed as MotionService's stick convention
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

// The walk at share[axis] of each axis's top speed at the stride, the shares summing to at most 1, signed as the stick.
// Each axis walks its share of the total share of its stroke, and the gait steps at the total share of the slowest top
// cadence among those axes: a combined stick never steps an axis faster than that axis's own top cadence, and so walks
// an axis with a faster one below its share of its top speed.
inline WalkTarget walkAtShares(GaitType gait, const float (&share)[3], float lx, float ly, float rx, float s1, float h,
                               float fd, float stride) {
    float used[3] = {}; // the shares of the axes the posture can walk
    float stroke[3] = {};
    float total = 0.0f, slowest = gait_envelope::CADENCE_MAX;
    for (int axis = 0; axis < 3; ++axis) {
        if (share[axis] <= 0.0f) continue;
        const AxisEnvelope e = envelopeAt(gait, axis, s1, h, fd, stride);
        // Defensive: walkable promises only the forward axis, though no walkable cell of the table has an empty one.
        if (e.stroke <= 0.0f) continue;
        used[axis] = share[axis];
        stroke[axis] = e.stroke;
        slowest = std::min(slowest, e.cadence);
        total += used[axis];
    }
    if (total <= 0.0f) return {};
    for (int axis = 0; axis < 3; ++axis) stroke[axis] *= used[axis] / total;
    return {-std::copysign(stroke[1], lx), std::copysign(stroke[0], ly), std::copysign(stroke[2], rx), total * slowest};
}

} // namespace gait_speed_detail

inline WalkTarget classicWalkTarget(GaitType gait, float lx, float ly, float rx, float s, float s1, float h, float fd,
                                    float stride) {
    using namespace gait_speed_detail;
    if (!isClassic(gait)) return {};
    float share[3];
    stickShares(lx, ly, rx, s, share);
    return walkAtShares(gait, share, lx, ly, rx, s1, h, fd, stride);
}

// AUTO walks the speed the stick asks of the fastest classic gait, BI: the stick's share of BI's top speed at the
// posture and stride, on each axis, whichever gait walks it.
struct AutoDemand {
    float speed[3]; // forward and lateral in mm/s, yaw in rad/s
    float speed01;  // its share of BI's top speed, summed over the axes like the stick
};

inline AutoDemand autoDemand(float lx, float ly, float rx, float s, float s1, float h, float fd, float stride) {
    float share[3];
    gait_speed_detail::stickShares(lx, ly, rx, s, share);
    AutoDemand demand {};
    for (int axis = 0; axis < 3; ++axis) {
        const AxisEnvelope e = envelopeAt(GaitType::BI_GATE, axis, s1, h, fd, stride);
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
        const AxisEnvelope e = envelopeAt(gait, axis, s1, h, fd, stride);
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
        const float vMax = envelopeAt(gait, axis, s1, h, fd, stride).vMax;
        share[axis] = vMax > 0.0f ? demand.speed[axis] / vMax : 0.0f;
    }
    const float total = share[0] + share[1] + share[2];
    if (total > 1.0f)
        for (float& axisShare : share) axisShare /= total;
    return walkAtShares(gait, share, lx, ly, rx, s1, h, fd, stride);
}
