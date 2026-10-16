#pragma once

#include <algorithm>
#include <cmath>
#include <gait_envelope.h>
#include <message_types.h>

// Stick -> walk command for the classic gaits, from the speed envelope the generator measured
// (gait_envelope.h). The stick is a fraction of the top speed the posture allows, and the stroke and
// cadence that produce it are picked here, so a command can never ask for more than the joints and servos
// can follow. Posture commands are pulled into the walkable region by clampPosture.

struct AxisEnvelope {
    float vMax;   // mm/s (forward, lateral) or rad/s (yaw)
    float stroke; // mm or rad per cycle that reaches vMax
};

namespace gait_speed_detail {

constexpr int CELLS = gait_envelope::GRID_N - 1;
constexpr float CELLS_PER_UNIT = CELLS / 2.0f;

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

inline float trilinear(const float (&table)[gait_envelope::GRID_N][gait_envelope::GRID_N][gait_envelope::GRID_N],
                       const Cell& s1, const Cell& h, const Cell& fd) {
    const auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };
    float along_fd[2][2];
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            along_fd[i][j] = lerp(table[s1.index + i][h.index + j][fd.index],
                                  table[s1.index + i][h.index + j][fd.index + 1], fd.frac);
    return lerp(lerp(along_fd[0][0], along_fd[0][1], h.frac), lerp(along_fd[1][0], along_fd[1][1], h.frac), s1.frac);
}

} // namespace gait_speed_detail

// True when the posture sits in a cell whose eight corners can all walk.
inline bool postureWalkable(float s1, float h, float fd) {
    using namespace gait_speed_detail;
    return cellWalkable(locate(s1), locate(h), locate(fd));
}

// Axis 0 forward, 1 lateral, 2 yaw. Interpolates only inside walkable cells, because the table holds raw
// values in the unwalkable ones; there and for GaitType::TUNED/AUTO the envelope is empty.
inline AxisEnvelope envelopeAt(GaitType gait, int axis, float s1, float h, float fd) {
    using namespace gait_speed_detail;
    if (!isClassic(gait) || axis < 0 || axis > 2) return {0.0f, 0.0f};
    const int g = (int)gait;
    const Cell cs1 = locate(s1), ch = locate(h), cfd = locate(fd);
    if (!cellWalkable(cs1, ch, cfd)) return {0.0f, 0.0f};
    return {trilinear(gait_envelope::V_MAX[g][axis], cs1, ch, cfd),
            trilinear(gait_envelope::STROKE[g][axis], cs1, ch, cfd)};
}

// An unwalkable posture is scaled toward the neutral one along its own direction. The walkable region is
// not star-shaped about neutral, so a plain bisection can settle on an inner boundary; a coarse scan from
// the outside finds the outermost walkable step first, and the bisection refines the boundary above it.
inline void clampPosture(float& s1, float& h, float& fd) {
    constexpr int SCAN_STEPS = 32;
    constexpr int REFINE_STEPS = 5;
    s1 = std::clamp(s1, -1.0f, 1.0f);
    h = std::clamp(h, -1.0f, 1.0f);
    fd = std::clamp(fd, -1.0f, 1.0f);
    if (postureWalkable(s1, h, fd)) return;
    int step = SCAN_STEPS - 1;
    while (step > 0 && !postureWalkable(s1 * step / SCAN_STEPS, h * step / SCAN_STEPS, fd * step / SCAN_STEPS)) --step;
    float walkable = (float)step / SCAN_STEPS, blocked = (float)(step + 1) / SCAN_STEPS;
    for (int i = 0; i < REFINE_STEPS; ++i) {
        const float mid = 0.5f * (walkable + blocked);
        (postureWalkable(s1 * mid, h * mid, fd * mid) ? walkable : blocked) = mid;
    }
    s1 *= walkable;
    h *= walkable;
    fd *= walkable;
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
    float speed01;                 // commanded share of the top speed, for gait selection
};

inline WalkTarget classicWalkTarget(GaitType gait, float lx, float ly, float rx, float s, float s1, float h, float fd) {
    using namespace gait_speed_detail;
    if (!isClassic(gait)) return {};

    // Stick axes in envelope order: forward, lateral, yaw.
    const float stick[3] = {ly, lx, rx};
    const float length = std::max(1.0f, std::fabs(lx) + std::fabs(ly) + std::fabs(rx));
    const float frac = speedFraction(s);
    const float standFrac = gait_envelope::STAND_FRAC[(int)gait];

    float share[3] = {}; // |stick| / L of the axes the posture can walk
    float vMax[3] = {};
    float cadence[3] = {}; // cycles/s at which the axis's full stroke reaches its top speed
    float shareSum = 0.0f;
    for (int axis = 0; axis < 3; ++axis) {
        const AxisEnvelope e = envelopeAt(gait, axis, s1, h, fd);
        // Walkable guarantees only the forward axis, so lateral or yaw can be empty in a walkable cell.
        if (std::fabs(stick[axis]) == 0.0f || e.vMax <= 0.0f || e.stroke <= 0.0f) continue;
        share[axis] = std::fabs(stick[axis]) / length;
        vMax[axis] = e.vMax;
        cadence[axis] = e.vMax * standFrac / e.stroke;
        shareSum += share[axis];
    }
    const float total = frac * shareSum;
    if (total <= 0.0f) return {};

    // One cadence for the whole gait, the share-weighted mean of the axes' own, scaled by sqrt(total) so
    // that a slower command takes both a shorter stroke and a lower cadence. Every stroke then stays below
    // its envelope stroke, and the cadence below the cap that the table already respects.
    float weighted = 0.0f;
    for (int axis = 0; axis < 3; ++axis) weighted += share[axis] / shareSum * cadence[axis];
    const float phaseRate = std::sqrt(total) * weighted;

    float stroke[3];
    for (int axis = 0; axis < 3; ++axis) stroke[axis] = frac * share[axis] * vMax[axis] * standFrac / phaseRate;
    return {-std::copysign(stroke[1], lx), std::copysign(stroke[0], ly), std::copysign(stroke[2], rx), phaseRate,
            std::min(total, 1.0f)};
}
