#pragma once

#include <algorithm>
#include <cmath>
#include <gait.h>
#include <gait_speed.h>

/*
 * AUTO changes gait while it walks. The speed picks a duty: the wave family from WAVE's 5/6 at a crawl, through
 * ripple-like duties, to TRI's at TRI's top speed, then BI from the least share of its top speed it stands up at to its
 * top. While it walks, a foot on the floor never moves on it, so no foot slips: the gait changes through the legs'
 * timing alone; once the stroke has eased away, the feet ease home as after any classic walk. A foot
 * lifts when the current pattern says, its swing stretched, or shortened as far as the servos allow, to land on the
 * current pattern's phase. Explored on a feet-only bench first; the firmware's virtual robot is the judge.
 */
namespace auto_morph {

constexpr int SAMPLES = gait_speed_detail::SAMPLES;
constexpr float WAVE_DUTY = gait_envelope::STAND_FRAC[(int)GaitType::WAVE];
constexpr float TRI_DUTY = gait_envelope::STAND_FRAC[(int)GaitType::TRI_GATE];
constexpr float BI_DUTY = gait_envelope::STAND_FRAC[(int)GaitType::BI_GATE];
// Each tripod lands this share of a cycle before the other lifts.
constexpr float TRI_OVERLAP = TRI_DUTY - 0.5f;
// Each duty's top speed held back, so a swing has time in hand to catch a pattern that moved.
constexpr float RESERVE = 0.1f;
// BI stands up from this share of its top speed, the speed slider's s = 0.25 (speedFraction); slower, it falls.
constexpr float BI_FLOOR = 0.7f;
constexpr float HYSTERESIS = 0.06f;
constexpr float BI_OFFSETS[6] = {0.0f, 1.0f / 3, 2.0f / 3, 2.0f / 3, 1.0f / 3, 0.0f};
// In cycles: how far a swing may stretch or shorten past its nominal 1 - duty, and a stance run on past its duty.
constexpr float SWING_STRETCH = 0.25f;
constexpr float SWING_SHRINK = 0.05f;
constexpr float STANCE_RUN_ON = 0.05f;
constexpr float LIFT_FLOOR = 0.02f; // the shortest swing, in cycles
// Outside BI a foot waits on the floor while lifting it would leave fewer standing, unless its stroke has run out.
constexpr int LEAST_FEET_DOWN = 3;

inline float wrap01(float x) { return x - std::floor(x); }
// The signed distance from b to a around the cycle, in [-0.5, 0.5).
inline float cyclic(float a, float b) { return wrap01(a - b + 0.5f) - 0.5f; }

/*
 * The wave family's leg offsets at duty d: k(1 - d) front to rear down each side, the left side half a cycle on. TRI's
 * pattern at TRI_DUTY and WAVE's at WAVE_DUTY, the overlap shrinking to none at WAVE, where five feet stand anyway. It
 * is anchored at the middle legs, so as the duty moves the front and rear legs retime in opposite directions, each half
 * as far as anchored at either end; anchored at the front, the bench lost a leg's support in every transition.
 */
inline void familyOffsets(float d, float (&out)[6]) {
    const float spacing = 1.0f - d + TRI_OVERLAP * (WAVE_DUTY - d) / (WAVE_DUTY - TRI_DUTY);
    for (int leg = 0; leg < 6; ++leg) out[leg] = wrap01((float)(leg % 3 - 1) * spacing + (float)(leg / 3) * 0.5f);
}

struct Plan {
    WalkTarget walk;
    float duty = WAVE_DUTY;
    float offsets[6] = {};
    // The longest stroke the joints travel over the walk's own, along it (>= 1 where the walk walks).
    float reach = 1.0f;
    // A swing's peak joint rate per unit of progress by its length's share of the longest stroke, the axes blended.
    float swingKappa[SAMPLES] = {};
    GaitType label = GaitType::WAVE; // the classic gait nearest, for whatever still reads one
};

// The plan with its walk capped as a goal caps it (withStrokeCapAtDuty), at the plan's own duty; the cap scales every
// stroke alike, and the joints' travel stays where it was.
inline Plan withStrokeCap(Plan plan, int axis, float cap, float s1, float h, float fd) {
    const auto along = [axis](const WalkTarget& w) {
        return std::fabs(axis == 0 ? w.stepY : axis == 1 ? w.stepX : w.stepAngle);
    };
    const WalkTarget capped = withStrokeCapAtDuty(plan.duty, plan.walk, axis, cap, s1, h, fd, Stance::DEFAULT);
    if (along(capped) > 0.0f) plan.reach *= along(plan.walk) / along(capped);
    plan.walk = capped;
    return plan;
}

// AUTO's stick to a plan. Its state is the side of the TRI-BI step it is on and the shift that keeps the legs'
// offsets continuous across a change of pattern.
class Planner {
  public:
    // A stick asking for no speed has no plan: a walk stopping keeps the plan it walked (demand.speed01 > 0).
    Plan plan(const AutoDemand& demand, float lx, float ly, float rx, float s1, float h, float fd, float stride) {
        using namespace gait_speed_detail;
        const Cell cs1 = locate(s1), ch = locate(h), cfd = locate(fd);
        float strokeMax[3];
        Kappas kappas[3];
        for (int axis = 0; axis < 3; ++axis) {
            strokeMax[axis] = strokeMaxAt(axis, s1, h, fd, Stance::DEFAULT);
            if (strokeMax[axis] > 0.0f) kappas[axis] = kappasAt(axis, cs1, ch, cfd, Stance::DEFAULT);
        }
        const auto envelope = [&](int axis, float d) -> AxisEnvelope {
            if (strokeMax[axis] <= 0.0f) return {};
            return envelopeOfDuty(d, strokeMax[axis], kappas[axis].swing, kappas[axis].stance, stride);
        };
        const float q = demand.speed01;
        // The share of BI's top speed that the duty d tops out at, along the demand: BI_DUTY's is 1.
        const auto topShare = [&](float d) {
            float sum = 0.0f;
            for (int axis = 0; axis < 3; ++axis) {
                if (demand.speed[axis] <= 0.0f) continue;
                const float v = envelope(axis, d).vMax;
                if (v <= 0.0f) return 0.0f;
                sum += demand.speed[axis] / q / v;
            }
            return sum > 0.0f ? 1.0f / sum : 0.0f;
        };

        Plan out;
        // The stick runs linearly over the wave family's speeds and BI's, skipping the gap between TRI's top and BI's
        // floor, where no gait walks.
        const float triMax = topShare(TRI_DUTY) * (1.0f - RESERVE);
        const float biMax = 1.0f - RESERVE;
        const float biMin = std::min(biMax, std::max(triMax, BI_FLOOR));
        const float asked = q * (triMax + biMax - biMin);
        if (!_bi && asked > triMax)
            _bi = true;
        else if (_bi && asked < triMax * (1.0f - HYSTERESIS))
            _bi = false;
        const float share = _bi ? biMin + std::max(0.0f, asked - triMax) : std::min(asked, triMax);
        // The most stable duty whose top, less the reserve, covers the share; the top falls as the duty rises.
        float lo = _bi ? BI_DUTY : TRI_DUTY, hi = _bi ? TRI_DUTY : WAVE_DUTY;
        const auto covers = [&](float d) { return topShare(d) * (1.0f - RESERVE) >= share; };
        if (covers(hi))
            lo = hi;
        else if (covers(lo))
            for (int i = 0; i < 20; ++i) {
                const float mid = 0.5f * (lo + hi);
                (covers(mid) ? lo : hi) = mid;
            }
        out.duty = lo;

        float axisShare[3] = {};
        float total = 0.0f;
        for (int axis = 0; axis < 3; ++axis) {
            const float v = envelope(axis, out.duty).vMax;
            if (demand.speed[axis] <= 0.0f || v <= 0.0f) continue;
            axisShare[axis] = demand.speed[axis] / q * share / v;
            total += axisShare[axis];
        }
        out.walk = walkAtSharesOf(axisShare, lx, ly, rx, [&](int axis) { return envelope(axis, out.duty); });
        // Each axis strokes its weight of its own stroke (walkAtSharesOf), so to first order the joints run out where
        // the weighted sum of each stroke's share of its axis's longest reaches 1: an axis of small weight, as a goal's
        // line hold, shortens the reach by about its weight, not to its own.
        float reachInverse = 0.0f;
        for (int axis = 0; axis < 3; ++axis) {
            if (axisShare[axis] <= 0.0f) continue;
            const float weight = axisShare[axis] / total;
            reachInverse += weight * envelope(axis, out.duty).stroke / strokeMax[axis];
            for (int n = 0; n < SAMPLES; ++n) out.swingKappa[n] += weight * kappas[axis].swing[n];
        }
        out.reach = reachInverse > 0.0f ? 1.0f / reachInverse : 1.0f;
        out.label = _bi ? GaitType::BI_GATE : out.duty >= 0.75f ? GaitType::WAVE : GaitType::TRI_GATE;

        float raw[6];
        if (_bi)
            std::copy(BI_OFFSETS, BI_OFFSETS + 6, raw);
        else
            familyOffsets(out.duty, raw);
        if (_have && _bi != _wasBi) _shift = bestShift(raw);
        _wasBi = _bi;
        _have = true;
        for (int leg = 0; leg < 6; ++leg) _last[leg] = out.offsets[leg] = wrap01(raw[leg] + _shift);
        return out;
    }

  private:
    // The shift of a new pattern that moves the legs least from the last one's offsets.
    float bestShift(const float (&raw)[6]) const {
        float best = 0.0f, bestCost = INFINITY;
        for (int step = 0; step < 720; ++step) {
            const float shift = (float)step / 720;
            float cost = 0.0f;
            for (int leg = 0; leg < 6; ++leg) cost += std::fabs(cyclic(raw[leg] + shift, _last[leg]));
            if (cost < bestCost) {
                bestCost = cost;
                best = shift;
            }
        }
        return best;
    }

    bool _bi = false, _wasBi = false, _have = false;
    float _shift = 0.0f;
    float _last[6] = {};
};

} // namespace auto_morph

// Steps the feet of AUTO's walk on the planner's last plan, from a standstill until the walk stops.
class GaitMorph {
  public:
    void setPlan(const auto_morph::Plan& plan) { _plan = plan; }
    bool running() const { return _running; }
    void stop() { _running = false; }

    // Whether the leg's foot is on the floor; every foot is while the morph is not walking.
    bool inStance(int leg) const { return !_running || _legs[leg].stance; }

    // The phase of the cycle left before the leg's foot lifts on the current plan: 0 in its swing, never while the
    // morph is not walking.
    float phaseToLift(int leg) const {
        if (!_running) return INFINITY;
        const Leg& l = _legs[leg];
        return l.stance
                   ? std::max(0.0f, std::min(patternLiftIn(leg, l), _plan.duty + auto_morph::STANCE_RUN_ON - l.elapsed))
                   : 0.0f;
    }

    // `neutral` are the feet's resting places, which the strokes are about.
    void step(const gait_state_t& g, BodyStateMsg& body, const float (&neutral)[6][4], float dt) {
        using namespace auto_morph;
        const float d = _plan.duty;
        const float dPhase = g.phase_rate * dt;
        if (!_running) start(body);
        _phase = wrap01(_phase + dPhase);
        int down = 0;
        for (const Leg& l : _legs) down += l.stance;
        const int leastDown = _plan.label == GaitType::BI_GATE ? 0 : LEAST_FEET_DOWN;
        float feet[6][4];
        for (int leg = 0; leg < 6; ++leg) {
            Leg& l = _legs[leg];
            const float nx = neutral[leg][0], ny = neutral[leg][1];
            // The leg's stroke per cycle, a turn's included, as generateFeet strokes it.
            const float sx = g.step_x + g.step_angle * -ny, sy = g.step_y + g.step_angle * nx;
            const float stroke = std::hypot(sx, sy);
            const float landX = nx + sx / 2, landY = ny + sy / 2;
            bool lifted = false;
            if (l.stance) {
                l.x -= sx * dPhase / d;
                l.y -= sy * dPhase / d;
                l.elapsed += dPhase;
                const float along = stroke > 0.0f ? ((l.x - nx) * sx + (l.y - ny) * sy) / stroke : 0.0f;
                // The back of the planned stroke, not of the command ramp's: from a start the ramp's is all but
                // nothing. A foot past the stroke and its run-on is due to lift, as on the pattern's word, and waits
                // for the others to land; only one at the end of the joints' travel lifts whatever stands.
                const float planned = std::hypot(_plan.walk.stepX + _plan.walk.stepAngle * -ny,
                                                 _plan.walk.stepY + _plan.walk.stepAngle * nx);
                const float travel = _plan.reach * planned / 2;
                const float back = std::min(planned / 2 * (1.0f + 2.0f * STANCE_RUN_ON / d), travel);
                const bool due = patternLiftIn(leg, l) <= 0.0f || l.elapsed >= d + STANCE_RUN_ON ||
                                 (stroke > 0.0f && along <= -back);
                if ((due && down > leastDown) || (stroke > 0.0f && along <= -travel)) {
                    down--;
                    l.stance = false;
                    l.fromX = l.x;
                    l.fromY = l.y;
                    l.fromZ = l.z;
                    l.progress = 0.0f;
                    const float nominal = 1.0f - d;
                    const float lo =
                        std::max({LIFT_FLOOR, nominal - SWING_SHRINK,
                                  swingKappa(leg, landX, landY, stroke) / gait_envelope::SERVO_RAD_S * g.phase_rate});
                    const float hi = std::max(lo, nominal + SWING_STRETCH);
                    // The nearest landing on the pattern's phase within reach, else the bound nearest it.
                    const float toPattern = wrap01(-_plan.offsets[leg] - _phase);
                    float pick = toPattern, pickCost = INFINITY;
                    for (int n = 0; n < 3; ++n) {
                        const float c = toPattern + (float)n;
                        const float cost = c >= lo && c <= hi ? std::fabs(c - nominal)
                                                              : 10.0f + std::min(std::fabs(c - lo), std::fabs(c - hi));
                        if (cost < pickCost) {
                            pickCost = cost;
                            pick = c;
                        }
                    }
                    l.left = l.span = std::clamp(pick, lo, hi);
                    lifted = true;
                }
            }
            if (!l.stance && !lifted) {
                // Progress runs on to land when planned; a swing the servos cannot finish in time lands late instead.
                const float needed = (1.0f - l.progress) * g.phase_rate * swingKappa(leg, landX, landY, stroke) /
                                     gait_envelope::SERVO_RAD_S;
                if (l.left < needed) {
                    l.span += needed - l.left;
                    l.left = needed;
                }
                const float full = l.left > 0.0f ? (1.0f - l.progress) * dPhase / l.left : INFINITY;
                const float dp = std::min(1.0f - l.progress, full);
                const float used = std::isfinite(full) && full > 0.0f ? dp / full : 1.0f;
                l.progress += dp;
                l.left = std::max(0.0f, l.left - dPhase);
                const float p = l.progress >= 1.0f - 1e-6f ? 1.0f : l.progress;
                // Leaving and meeting the floor at the stance's velocity, as the classic swing's Bezier does.
                const float tangent = -l.span / d;
                l.x = hermite(l.fromX, landX, sx * tangent, p);
                l.y = hermite(l.fromY, landY, sy * tangent, p);
                l.z = l.fromZ + (neutral[leg][2] - l.fromZ) * p + 16.0f * p * p * (1 - p) * (1 - p) * g.step_height;
                if (p >= 1.0f) {
                    // A foot landing within the tick stands the rest of it.
                    down++;
                    l.stance = true;
                    l.elapsed = dPhase * (1.0f - used);
                    l.x = landX - sx * l.elapsed / d;
                    l.y = landY - sy * l.elapsed / d;
                    l.z = neutral[leg][2];
                }
            }
            feet[leg][0] = l.x;
            feet[leg][1] = l.y;
            feet[leg][2] = l.z;
            feet[leg][3] = 1.0f;
        }
        body.updateFeet(feet);
    }

  private:
    struct Leg {
        bool stance = true;
        float elapsed = 0.0f;  // the phase since touchdown
        float progress = 0.0f; // through the swing
        float left = 0.0f;     // the phase the swing has left as planned
        float span = 0.0f;     // the swing's whole phase as planned
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float fromX = 0.0f, fromY = 0.0f, fromZ = 0.0f;
    };

    static float hermite(float a, float b, float m, float s) {
        const float s2 = s * s, s3 = s2 * s;
        return (2 * s3 - 3 * s2 + 1) * a + (s3 - 2 * s2 + s) * m + (-2 * s3 + 3 * s2) * b + (s3 - s2) * m;
    }

    // A walk starts with every foot down where it is, each as far into its stance as the pattern has it. A foot the
    // pattern has in the first half of its swing lifts at once; one in the second half is taken as just landed, or it
    // would lift with the feet the pattern lifts next and leave too few standing.
    void start(const BodyStateMsg& body) {
        _running = true;
        _phase = 0.0f;
        const float d = _plan.duty;
        for (int leg = 0; leg < 6; ++leg) {
            Leg& l = _legs[leg];
            l = Leg {};
            l.x = body.feet[leg][0];
            l.y = body.feet[leg][1];
            l.z = body.feet[leg][2];
            const float phase = auto_morph::wrap01(_plan.offsets[leg]);
            l.elapsed = phase < d ? phase : phase < d + (1.0f - d) / 2 ? d : 0.0f;
        }
    }

    // The phase left before the pattern lifts the foot: no earlier than half a duty into its stance, then at the
    // pattern's next lift.
    float patternLiftIn(int leg, const Leg& l) const {
        const float early = 0.5f * _plan.duty - l.elapsed;
        return early + auto_morph::wrap01(_plan.duty - _plan.offsets[leg] - _phase - early);
    }

    float swingKappa(int leg, float landX, float landY, float stroke) const {
        const Leg& l = _legs[leg];
        const float length = std::hypot(landX - (l.stance ? l.x : l.fromX), landY - (l.stance ? l.y : l.fromY));
        const float share = stroke > 0.0f ? std::min(1.0f, length / (stroke * _plan.reach)) : 0.0f;
        return gait_speed_detail::kappaAtStroke(_plan.swingKappa, share);
    }

    auto_morph::Plan _plan;
    Leg _legs[6];
    float _phase = 0.0f;
    bool _running = false;
};
