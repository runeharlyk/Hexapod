#pragma once

// Leg-to-leg clearance of a pose, the port of simulation/src/robot/clearance.py. Each leg is the chain mount -> hip ->
// knee -> tip its joint angles give, the coxa, femur and tibia segments with the foot a zero-length segment at the tip,
// each a capsule; two legs' clearance is the least exact segment-segment distance between their capsules' centre lines
// less the two radii, negative when they overlap. Both ports are checked against the same fixtures
// (platform_shared/leg_clearance_fixtures.txt, firmware/test/test_leg_clearance), so they cannot drift.
//
// Float math on the stack: 96 segment distances a pose.

#include <cmath>
#include <kinematics.h>
#include <utils/math_utils.h>

namespace leg_clearance {

// Capsule radii in mm: each the larger of the MuJoCo model's (simulation/src/resources/build_model.py: coxa 8, femur 7,
// tibia 5, the foot sphere 6) and half the part's thickness across the leg's plane, the way a neighbouring leg comes at
// it, measured from the meshes the app draws (app/static/hex) and the MG92B's size (TowerPro: 22.8 x 12 x 31 mm,
// https://www.towerpro.com.tw/product/mg92b/). None is measured on the robot.
// Half the 41.6 x 41.6 mm cross-section of the coxa bracket that carries the femur servo (joint.stl).
constexpr float COXA_RADIUS_MM = 20.8f;
// Half the MG92B's 31 mm along its output shaft, the femur's pitch axis, which the femur bracket (middle_joint.stl)
// matches at 30.5 mm across; its end cap holds the tibia's 16 mm horn boss at the knee.
constexpr float FEMUR_RADIUS_MM = 15.5f;
// The model's, over half the tibia blade's 5 to 7 mm (foot.stl); the blade's up to 40 mm depth within the leg's plane
// is not modelled.
constexpr float TIBIA_RADIUS_MM = 5.0f;
// The model's foot sphere, which holds foot.stl's 9 x 4 mm tip.
constexpr float FOOT_RADIUS_MM = 6.0f;
constexpr float RADIUS_MM[4] = {COXA_RADIUS_MM, FEMUR_RADIUS_MM, TIBIA_RADIUS_MM, FOOT_RADIUS_MM};

// The pairs that can meet: the ring 0-1-2-5-4-3, the same-side neighbours and the front and rear pairs across the body.
constexpr int PAIRS[6][2] = {{0, 1}, {1, 2}, {3, 4}, {4, 5}, {0, 3}, {2, 5}};

struct Vec3 {
    float x, y, z;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

// Exact distance between segments p0-p1 and q0-q1, either of which may be a point (Ericson, Real-Time Collision
// Detection, 5.1.9). The closest point's parameter on p0-p1 of two skew segments, (b f - c e) / (a e - b^2) in
// Ericson's terms, is taken as -(n . (r x d2)) / (n . n) with n = d1 x d2, the same quantity by the Binet-Cauchy
// identity: in float the difference of products loses all precision for nearly parallel segments and overestimated the
// distance by up to 0.18 mm, where the cross products keep their relative precision. Segments within 1e-6 rad of
// parallel are taken as parallel, at most 1e-4 mm off over a leg's 189 mm.
inline float segmentDistance(Vec3 p0, Vec3 p1, Vec3 q0, Vec3 q1) {
    constexpr float NEAR_ZERO = 1e-6f;      // mm^2: a segment shorter than 1e-3 mm is a point
    constexpr float NEAR_PARALLEL = 1e-12f; // sin^2 of the angle between the segments
    const Vec3 d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
    const float a = dot(d1, d1), e = dot(d2, d2), b = dot(d1, d2), c = dot(d1, r), f = dot(d2, r);
    float s = 0.0f, t = 0.0f;
    if (a <= NEAR_ZERO && e <= NEAR_ZERO) {
    } else if (a <= NEAR_ZERO) {
        t = CLIP(f / e, 0.0f, 1.0f);
    } else if (e <= NEAR_ZERO) {
        s = CLIP(-c / a, 0.0f, 1.0f);
    } else {
        const Vec3 n = cross(d1, d2);
        const float denom = dot(n, n);
        s = denom > NEAR_PARALLEL * a * e ? CLIP(-dot(n, cross(r, d2)) / denom, 0.0f, 1.0f) : 0.0f;
        t = (b * s + f) / e;
        if (t < 0.0f) {
            t = 0.0f;
            s = CLIP(-c / a, 0.0f, 1.0f);
        } else if (t > 1.0f) {
            t = 1.0f;
            s = CLIP((b - c) / a, 0.0f, 1.0f);
        }
    }
    const Vec3 gap = (p0 + d1 * s) - (q0 + d2 * t);
    return sqrtf(dot(gap, gap));
}

// Leg `leg`'s mount, hip, knee and tip from its coxa, femur and tibia angles in degrees, in the body frame
// Kinematics::inverseKinematics solves in: the coxa azimuth is -coxa in the leg's mount frame, the femur's pitch femur,
// the tibia's femur + tibia.
inline void legChain(int leg, const float deg[3], Vec3 out[4]) {
    constexpr float RAD = PI_F / 180.0f;
    const HexapodConfig &c = hexapodConfig;
    const float ca = cosf(c.mountAngle[leg] * RAD), sa = sinf(c.mountAngle[leg] * RAD);
    const float localX = cosf(deg[0] * RAD), localY = -sinf(deg[0] * RAD);
    const Vec3 radial {localX * ca + localY * sa, localX * sa - localY * ca, 0.0f};
    const Vec3 up {0.0f, 0.0f, 1.0f};
    const float femur = deg[1] * RAD, tibia = (deg[1] + deg[2]) * RAD;
    out[0] = {c.mountX[leg], c.mountY[leg], 0.0f};
    out[1] = out[0] + radial * (c.rootToJoint1 + c.joint1ToJoint2);
    out[2] = out[1] + (radial * cosf(femur) + up * sinf(femur)) * c.joint2ToJoint3;
    out[3] = out[2] + (radial * cosf(tibia) + up * sinf(tibia)) * c.joint3ToTip;
}

// Two legs' least surface clearance over their segment pairs, the foot the tip again; NaN from a non-finite chain.
inline float pairClearance(const Vec3 a[4], const Vec3 b[4]) {
    float least = INFINITY;
    for (int i = 0; i < 4; ++i) {
        const Vec3 a1 = a[i < 3 ? i + 1 : 3];
        for (int j = 0; j < 4; ++j) {
            const Vec3 b1 = b[j < 3 ? j + 1 : 3];
            const float gap = segmentDistance(a[i], a1, b[j], b1) - RADIUS_MM[i] - RADIUS_MM[j];
            if (std::isnan(gap)) return gap;
            least = fminf(least, gap);
        }
    }
    return least;
}

// The pair of PAIRS, by index, with the least surface clearance of 18 joint angles in degrees, coxa, femur and tibia
// per leg in leg order, and that clearance in mm; the first such pair on a tie, and the first pair with a NaN clearance
// when an angle is not finite.
struct Gap {
    int pair;
    float mm;
};

// Each pair of PAIRS' least surface clearance in mm, in PAIRS' order, of 18 joint angles in degrees.
inline void pairClearances(const float angles[18], float mm[6]) {
    Vec3 chains[6][4];
    for (int leg = 0; leg < 6; ++leg) legChain(leg, &angles[leg * 3], chains[leg]);
    for (int k = 0; k < 6; ++k) mm[k] = pairClearance(chains[PAIRS[k][0]], chains[PAIRS[k][1]]);
}

inline Gap closest(const float angles[18]) {
    float mm[6];
    pairClearances(angles, mm);
    Gap least {0, INFINITY};
    for (int k = 0; k < 6; ++k) {
        if (std::isnan(mm[k])) return {k, mm[k]};
        if (mm[k] < least.mm) least = {k, mm[k]};
    }
    return least;
}

} // namespace leg_clearance

// The least surface clearance a pose keeps: an allowance, chosen and not measured, for the round capsules against the
// real parts and for the servos' tracking error. It is of the order a servo's deadband and backlash of 1 to 2 deg give
// over the leg's lever of about 190 mm from the coxa axis to the foot, 3 to 6 mm; neither the servo's deadband in
// degrees nor its backlash is measured, which the bench session (roadmap 1.7) is to do.
constexpr float LEG_CLEARANCE_MM = 5.0f;

// The least surface clearance in mm over the pairs that can meet of 18 joint angles in degrees, coxa, femur and tibia
// per leg in leg order; negative when two legs overlap.
inline float legClearance(const float angles[18]) { return leg_clearance::closest(angles).mm; }

// Whether every pair that can meet keeps LEG_CLEARANCE_MM; a non-finite clearance keeps nothing apart.
inline bool legsClear(const float angles[18]) { return legClearance(angles) >= LEG_CLEARANCE_MM; }

// Whether moving the legs from pose `from` to pose `to` keeps them apart, pair by pair: each pair keeps
// LEG_CLEARANCE_MM in `to`, or, a pair already within it in `from`, comes no closer, so legs inside the margin can
// always move apart and no other pair may come together meanwhile. A non-finite `to` keeps nothing apart.
inline bool keepsLegsApart(const float from[18], const float to[18]) {
    float now[6], next[6];
    leg_clearance::pairClearances(from, now);
    leg_clearance::pairClearances(to, next);
    for (int k = 0; k < 6; ++k)
        if (!(next[k] >= (now[k] < LEG_CLEARANCE_MM ? now[k] : LEG_CLEARANCE_MM))) return false;
    return true;
}
