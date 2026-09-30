#pragma once

// Port of simulation/src/robot/animation.py: the animation clip model, its validator, the evaluator,
// the pose-to-servo-angle path and the player. The Python module is the reference and carries the
// rules in its docstrings; this file mirrors it line for line and is checked against the fixtures in
// animations/fixtures/ by firmware/test/test_animation. It knows nothing about nanopb or the ESP: the
// codec fills a Clip from the decoded file, and the runner drives the player from the control task.
//
// Units: body angles rad, lengths mm, joint angles deg in the IK output convention, time seconds.
// File values are 32-bit floats and so is every value here.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <kinematics.h>
#include <message_types.h>
#include <utils/math_utils.h>

namespace anim {

constexpr int KEYFRAME_MAX = 32;
constexpr int OVERLAY_MAX = 8;
constexpr int PARAM_MAX = 10;
constexpr int NAME_LEN_MAX = 32;
constexpr int DESCRIPTION_LEN_MAX = 96;
constexpr uint32_t SCHEMA_VERSION = 1;
constexpr float DEFAULT_ENTRY_S = 0.5f;
constexpr float DEFAULT_EXIT_S = 0.5f;
constexpr float STEP_ARC_MM = 45.0f;
constexpr float STEP_ARC_FULL_TRAVEL_MM = 40.0f;
constexpr float STEP_ARC_MIN_TRAVEL_MM = 2.0f;

enum Ease : int { LINEAR = 0, EASE_IN = 1, EASE_OUT = 2, EASE_IN_OUT = 3 };

enum ParamId : int {
    SPEED = 0,
    BODY_X = 1,
    BODY_Y = 2,
    BODY_Z = 3,
    BODY_ROLL = 4,
    BODY_PITCH = 5,
    BODY_YAW = 6,
    FOOT_LIFT = 7,
    OVERLAY_AMPLITUDE = 8,
    REPEAT = 9,
    PARAM_COUNT = 10
};

enum ChannelKind : int { CHANNEL_NONE = 0, CHANNEL_BODY = 1, CHANNEL_FOOT = 2 };

enum BodyAxis : int { ROLL = 0, PITCH = 1, YAW = 2, X = 3, Y = 4, Z = 5 };

constexpr ParamId BODY_PARAM_FOR_AXIS[6] = {BODY_ROLL, BODY_PITCH, BODY_YAW, BODY_X, BODY_Y, BODY_Z};

// A leg is either a foot offset from the standing foot (mm, +z lifts) or three joint angles (deg).
struct LegTarget {
    bool joints = false;
    float v[3] = {0.0f, 0.0f, 0.0f};
};

struct Keyframe {
    float time = 0.0f;
    int ease = LINEAR;
    float body[6] = {0, 0, 0, 0, 0, 0};  // roll, pitch, yaw, x, y, z offsets
    int legCount = 0;                    // 0 = every foot holds stance, else 6
    LegTarget legs[6];
};

struct Overlay {
    ChannelKind kind = CHANNEL_NONE;  // channel is a BodyAxis for CHANNEL_BODY, else leg*3 + axis
    int channel = 0;
    float amplitude = 0.0f;
    float frequency = 1.0f;
    float phase = 0.0f;
    float start = 0.0f;
    float end = 0.0f;
};

struct ParamSpec {
    int id = 0;
    float min = 0.0f;
    float defaultValue = 1.0f;
    float max = 1.0f;
};

struct ParamValue {
    int id;
    float value;
};

struct Clip {
    char name[NAME_LEN_MAX + 1] = {0};
    char description[DESCRIPTION_LEN_MAX + 1] = {0};
    uint32_t schema = SCHEMA_VERSION;
    bool loop = false;
    bool holdEnd = false;
    float entryTime = 0.0f;
    float exitTime = 0.0f;
    int keyframeCount = 0;
    Keyframe keyframes[KEYFRAME_MAX];
    int overlayCount = 0;
    Overlay overlays[OVERLAY_MAX];
    int paramCount = 0;
    ParamSpec params[PARAM_MAX];

    float duration() const { return keyframeCount > 0 ? keyframes[keyframeCount - 1].time : 0.0f; }
    float entrySeconds() const { return entryTime > 0.0f ? entryTime : DEFAULT_ENTRY_S; }
    float exitSeconds() const { return exitTime > 0.0f ? exitTime : DEFAULT_EXIT_S; }
};

inline LegTarget legTarget(const Keyframe &k, int leg) { return k.legCount > 0 ? k.legs[leg] : LegTarget{}; }

inline bool validName(const char *name) {
    const size_t n = strlen(name);
    if (n < 1 || n > NAME_LEN_MAX) return false;
    for (size_t i = 0; i < n; ++i) {
        const char c = name[i];
        const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

// The first structural error as a static string, or nullptr. Same rules and order as the reference's
// validate(); the description limit is bytes because the file buffer is bytes.
inline const char *validate(const Clip &c) {
    if (c.schema != SCHEMA_VERSION) return "schema is not 1";
    // Decoded input cannot trip the name and description rules because nanopb caps the strings; they mirror
    // the reference for hand-built clips.
    if (!validName(c.name)) return "name must be 1-32 characters of [a-z0-9_-]";
    if (strlen(c.description) > (size_t)DESCRIPTION_LEN_MAX) return "description longer than 96 bytes";
    if (c.loop && c.holdEnd) return "loop and hold_end cannot both be set";
    if (c.keyframeCount < 1) return "at least one keyframe is required";
    if (c.keyframeCount > KEYFRAME_MAX) return "more than 32 keyframes";
    if (c.overlayCount > OVERLAY_MAX) return "more than 8 overlays";
    if (c.paramCount > PARAM_MAX) return "more than 10 params";
    for (int i = 0; i < c.keyframeCount; ++i) {
        const Keyframe &k = c.keyframes[i];
        if (k.legCount != 0 && k.legCount != 6) return "keyframe must have 0 or 6 legs";
        if (k.ease < LINEAR || k.ease > EASE_IN_OUT) return "keyframe ease out of range";
    }
    if (!std::isfinite(c.entryTime) || !std::isfinite(c.exitTime)) return "entry/exit time must be finite";
    for (int i = 0; i < c.keyframeCount; ++i) {
        const Keyframe &k = c.keyframes[i];
        if (!std::isfinite(k.time)) return "keyframe time must be finite";
        for (float v : k.body)
            if (!std::isfinite(v)) return "keyframe body must be finite";
        for (int l = 0; l < k.legCount; ++l)
            for (float v : k.legs[l].v)
                if (!std::isfinite(v)) return "keyframe leg must be finite";
    }
    for (int i = 0; i < c.overlayCount; ++i) {
        const Overlay &o = c.overlays[i];
        if (!std::isfinite(o.amplitude) || !std::isfinite(o.frequency) || !std::isfinite(o.phase) ||
            !std::isfinite(o.start) || !std::isfinite(o.end))
            return "overlay must be finite";
    }
    for (int i = 0; i < c.paramCount; ++i) {
        const ParamSpec &p = c.params[i];
        if (!std::isfinite(p.min) || !std::isfinite(p.defaultValue) || !std::isfinite(p.max))
            return "param must be finite";
    }
    if (c.keyframes[0].time != 0.0f) return "first keyframe must be at time 0";
    for (int i = 1; i < c.keyframeCount; ++i)
        if (c.keyframes[i].time <= c.keyframes[i - 1].time) return "keyframe time must increase";
    for (int i = 0; i < c.overlayCount; ++i) {
        const Overlay &o = c.overlays[i];
        if (o.kind == CHANNEL_NONE) return "overlay needs exactly one channel";
        if (o.kind == CHANNEL_BODY && (o.channel < 0 || o.channel > 5)) return "overlay body_axis out of range";
        if (o.kind == CHANNEL_FOOT && (o.channel < 0 || o.channel > 17)) return "overlay foot_channel out of range";
        if (o.start < 0.0f || o.start >= o.end) return "overlay window must have 0 <= start < end";
        if (o.end > c.duration()) return "overlay end is after the last keyframe";
    }
    for (int i = 0; i < c.paramCount; ++i) {
        const ParamSpec &p = c.params[i];
        if (p.id < 0 || p.id >= PARAM_COUNT) return "param id out of range";
        for (int j = 0; j < i; ++j)
            if (c.params[j].id == p.id) return "param id is not unique";
        if (!(p.min <= p.defaultValue && p.defaultValue <= p.max)) return "param needs min <= default_value <= max";
        if (p.id == SPEED && p.min <= 0.0f) return "param SPEED needs a positive min";
        if (p.id == REPEAT && p.min < 1.0f) return "param REPEAT needs min >= 1";
    }
    return nullptr;
}

struct Pose {
    float body[6] = {0, 0, 0, 0, 0, 0};
    LegTarget legs[6];
};

inline float easeValue(int kind, float t) {
    switch (kind) {
        case EASE_IN: return t * t;
        case EASE_OUT: return t * (2.0f - t);
        case EASE_IN_OUT: return t < 0.5f ? 2.0f * t * t : -1.0f + (4.0f - 2.0f * t) * t;
        default: return t;
    }
}

// Every id gets a value: a declared id takes the caller's value clamped to its range, else its
// default; an undeclared id is 1 (the neutral multiplier and a single play).
inline void resolveParams(const Clip &c, const ParamValue *values, int count, float out[PARAM_COUNT]) {
    for (int i = 0; i < PARAM_COUNT; ++i) out[i] = 1.0f;
    for (int i = 0; i < c.paramCount; ++i) {
        const ParamSpec &spec = c.params[i];
        float v = spec.defaultValue;
        for (int j = 0; j < count; ++j)
            if (values[j].id == spec.id) v = values[j].value;
        out[spec.id] = CLIP(v, spec.min, spec.max);
    }
}

inline void bodyState(const float body6[6], const float stance[6][4], BodyStateMsg &b) {
    b.omega = body6[ROLL];
    b.phi = body6[PITCH];
    b.psi = body6[YAW];
    b.xm = body6[X];
    b.ym = body6[Y];
    b.zm = body6[Z];
    b.updateFeet(stance);
}

inline void legJointsDeg(Kinematics &kin, const float body6[6], const float foot[3], int leg,
                         const float stance[6][4], float out[3]) {
    BodyStateMsg b;
    bodyState(body6, stance, b);
    for (int k = 0; k < 3; ++k) b.feet[leg][k] += foot[k];
    float angles[18];
    kin.inverseKinematics(b, angles);
    for (int k = 0; k < 3; ++k) out[k] = angles[leg * 3 + k];
}

// The keyframe pair bracketing t and the eased fraction between them, with t clamped.
inline void segment(const Clip &c, float t, const Keyframe *&k0, const Keyframe *&k1, float &u) {
    if (t <= 0.0f || c.keyframeCount == 1) {
        k0 = k1 = &c.keyframes[0];
        u = 0.0f;
        return;
    }
    if (t >= c.keyframes[c.keyframeCount - 1].time) {
        k0 = k1 = &c.keyframes[c.keyframeCount - 1];
        u = 0.0f;
        return;
    }
    int i = 1;
    while (c.keyframes[i].time < t) ++i;
    k0 = &c.keyframes[i - 1];
    k1 = &c.keyframes[i];
    u = easeValue(k1->ease, (t - k0->time) / (k1->time - k0->time));
}

// Order, matching the reference: interpolate the body; add body overlays and collect each foot
// overlay; apply the BODY_* multipliers (this is the output body); then resolve legs. A foot leg is
// lerped, gets its overlay and FOOT_LIFT on z. A joint leg is lerped raw. A mixed leg takes the foot
// endpoint with its overlay and lift, runs IK against the OUTPUT body, and lerps in joint space with
// the raw joint endpoint, so the servo command is continuous at the switching keyframe.
inline void evaluate(const Clip &c, const float params[PARAM_COUNT], float t, Kinematics &kin,
                     const float stance[6][4], Pose &out) {
    const Keyframe *k0;
    const Keyframe *k1;
    float u;
    segment(c, t, k0, k1, u);
    t = CLIP(t, 0.0f, c.duration());
    float body[6];
    for (int a = 0; a < 6; ++a) body[a] = k0->body[a] + (k1->body[a] - k0->body[a]) * u;
    float footOverlay[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (int i = 0; i < c.overlayCount; ++i) {
        const Overlay &o = c.overlays[i];
        if (!(o.start <= t && t <= o.end)) continue;
        const float v = o.amplitude * params[OVERLAY_AMPLITUDE] * sinf(2.0f * (float)M_PI * o.frequency * t + o.phase);
        if (o.kind == CHANNEL_BODY) body[o.channel] += v;
        else footOverlay[o.channel / 3][o.channel % 3] += v;
    }
    for (int a = 0; a < 6; ++a) body[a] *= params[BODY_PARAM_FOR_AXIS[a]];
    for (int a = 0; a < 6; ++a) out.body[a] = body[a];

    for (int i = 0; i < 6; ++i) {
        const LegTarget a = legTarget(*k0, i);
        const LegTarget b = legTarget(*k1, i);
        LegTarget &leg = out.legs[i];
        auto lifted = [&](const float foot[3], float dst[3]) {
            for (int k = 0; k < 3; ++k) dst[k] = foot[k] + footOverlay[i][k];
            dst[2] *= params[FOOT_LIFT];
        };
        if (!a.joints && !b.joints) {
            float lerp[3];
            for (int k = 0; k < 3; ++k) lerp[k] = a.v[k] + (b.v[k] - a.v[k]) * u;
            leg.joints = false;
            lifted(lerp, leg.v);
        } else if (a.joints && b.joints) {
            leg.joints = true;
            for (int k = 0; k < 3; ++k) leg.v[k] = a.v[k] + (b.v[k] - a.v[k]) * u;
        } else {
            float ja[3], jb[3], foot[3];
            if (a.joints) {
                for (int k = 0; k < 3; ++k) ja[k] = a.v[k];
            } else {
                lifted(a.v, foot);
                legJointsDeg(kin, body, foot, i, stance, ja);
            }
            if (b.joints) {
                for (int k = 0; k < 3; ++k) jb[k] = b.v[k];
            } else {
                lifted(b.v, foot);
                legJointsDeg(kin, body, foot, i, stance, jb);
            }
            leg.joints = true;
            for (int k = 0; k < 3; ++k) leg.v[k] = ja[k] + (jb[k] - ja[k]) * u;
        }
    }
}

// 18 servo angles (deg, IK order) and an 18-bit mask: bit leg*3+joint for a joint pinned at its
// limit, and the femur and tibia bits of a foot leg the IK cannot reach.
inline uint32_t poseToAngles(const Pose &p, Kinematics &kin, const float stance[6][4], float angles[18]) {
    BodyStateMsg b;
    bodyState(p.body, stance, b);
    for (int i = 0; i < 6; ++i)
        if (!p.legs[i].joints)
            for (int k = 0; k < 3; ++k) b.feet[i][k] += p.legs[i].v[k];
    kin.inverseKinematics(b, angles);
    uint32_t mask = 0;
    for (int i = 0; i < 6; ++i) {
        if (p.legs[i].joints) {
            for (int k = 0; k < 3; ++k) angles[i * 3 + k] = p.legs[i].v[k];
        } else if (!kin.footReachable(b, i)) {
            mask |= 0x6u << (i * 3);
        }
    }
    for (int j = 0; j < 18; ++j) {
        const float limit = JOINT_LIMIT_DEG[j % 3];
        const float clamped = CLIP(angles[j], -limit, limit);
        if (clamped != angles[j]) {
            mask |= 1u << j;
            angles[j] = clamped;
        }
    }
    return mask;
}

inline void capturePose(const BodyStateMsg &b, const float stance[6][4], Pose &out) {
    out.body[ROLL] = b.omega;
    out.body[PITCH] = b.phi;
    out.body[YAW] = b.psi;
    out.body[X] = b.xm;
    out.body[Y] = b.ym;
    out.body[Z] = b.zm;
    for (int i = 0; i < 6; ++i) {
        out.legs[i].joints = false;
        for (int k = 0; k < 3; ++k) out.legs[i].v[k] = b.feet[i][k] - stance[i][k];
    }
}

// Status is pushed on every state change, and otherwise at most once per period while not idle.
inline bool statusDue(bool changed, bool idle, unsigned long nowMs, unsigned long lastMs, unsigned long periodMs) {
    if (changed) return true;
    if (idle) return false;
    return nowMs - lastMs >= periodMs;
}

enum class State : int { IDLE = 0, ENTRY = 1, PLAYING = 2, HOLD = 3, EXIT = 4 };

// Entry -> Playing -> Hold | Exit -> Idle around evaluate(). Mirrors the reference Player: REPEAT is
// max(1, floor(x + 0.5)); SPEED scales only the playing clock; Entry and Exit run on wall time; a
// play during any state starts Entry from the current pose; Exit after a finished play starts from
// the final keyframe; play() records the live pose so an immediate stop blends from it.
class Player {
  public:
    explicit Player(Kinematics &kin) : kin_(kin) {}

    // The standing feet every offset is relative to; the caller keeps them alive and current.
    void setStance(const float (*stance)[4]) { stance_ = stance; }

    void play(const Clip *clip, const ParamValue *values, int count, const Pose *live) {
        clip_ = clip;
        resolveParams(*clip, values, count, params_);
        t_ = 0.0f;
        playsDone_ = 0;
        const Pose start = live ? *live : lastPose_;
        lastPose_ = start;
        Pose first;
        evaluate(*clip, params_, 0.0f, kin_, stance_, first);
        startBlend(start, first, clip->entrySeconds(), State::ENTRY);
    }

    void stop() {
        if (state_ == State::IDLE) return;
        startBlend(lastPose_, Pose{}, clip_->exitSeconds(), State::EXIT);
    }

    const Pose &update(float dt) {
        if (state_ == State::IDLE) return lastPose_;
        if (state_ == State::ENTRY || state_ == State::EXIT) advanceBlend(dt);
        else if (state_ == State::HOLD) evaluate(*clip_, params_, clip_->duration(), kin_, stance_, lastPose_);
        else advancePlaying(dt);
        return lastPose_;
    }

    State state() const { return state_; }
    float t() const { return t_; }
    const Pose &lastPose() const { return lastPose_; }
    const Clip *clip() const { return clip_; }
    const float *params() const { return params_; }

  private:
    Kinematics &kin_;
    const float (*stance_)[4] = nullptr;
    State state_ = State::IDLE;
    const Clip *clip_ = nullptr;
    float params_[PARAM_COUNT] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    float t_ = 0.0f;
    Pose lastPose_;
    int playsDone_ = 0;
    float blendT_ = 0.0f;
    float blendSeconds_ = 1.0f;
    Pose blendFrom_;
    Pose blendTo_;

    // Legs where either side is a joint target are converted to joints on both sides once, so the
    // blend itself is a plain lerp; foot legs keep their offsets and get the step arc.
    void startBlend(const Pose &src, const Pose &dst, float seconds, State state) {
        blendFrom_ = src;
        blendTo_ = dst;
        for (int i = 0; i < 6; ++i) {
            if (!blendFrom_.legs[i].joints && !blendTo_.legs[i].joints) continue;
            if (!blendFrom_.legs[i].joints) {
                float j[3];
                legJointsDeg(kin_, blendFrom_.body, blendFrom_.legs[i].v, i, stance_, j);
                blendFrom_.legs[i] = {true, {j[0], j[1], j[2]}};
            }
            if (!blendTo_.legs[i].joints) {
                float j[3];
                legJointsDeg(kin_, blendTo_.body, blendTo_.legs[i].v, i, stance_, j);
                blendTo_.legs[i] = {true, {j[0], j[1], j[2]}};
            }
        }
        blendSeconds_ = seconds;
        blendT_ = 0.0f;
        state_ = state;
    }

    void advanceBlend(float dt) {
        blendT_ += dt;
        const float u = fminf(1.0f, blendT_ / blendSeconds_);
        const float e = easeValue(EASE_IN_OUT, u);
        for (int a = 0; a < 6; ++a) lastPose_.body[a] = blendFrom_.body[a] + (blendTo_.body[a] - blendFrom_.body[a]) * e;
        for (int i = 0; i < 6; ++i) {
            const LegTarget &la = blendFrom_.legs[i];
            const LegTarget &lb = blendTo_.legs[i];
            LegTarget &out = lastPose_.legs[i];
            out.joints = la.joints;
            for (int k = 0; k < 3; ++k) out.v[k] = la.v[k] + (lb.v[k] - la.v[k]) * e;
            if (la.joints) continue;
            const float travel = hypotf(lb.v[0] - la.v[0], lb.v[1] - la.v[1]);
            if (travel > STEP_ARC_MIN_TRAVEL_MM)
                out.v[2] += STEP_ARC_MM * fminf(1.0f, travel / STEP_ARC_FULL_TRAVEL_MM) * sinf((float)M_PI * u);
        }
        if (u >= 1.0f) {
            if (state_ == State::ENTRY) {
                state_ = State::PLAYING;
                t_ = 0.0f;
            } else {
                state_ = State::IDLE;
            }
        }
    }

    void advancePlaying(float dt) {
        const float duration = clip_->duration();
        t_ += dt * params_[SPEED];
        if (clip_->loop) {
            t_ = duration > 0.0f ? fmodf(t_, duration) : 0.0f;
            evaluate(*clip_, params_, t_, kin_, stance_, lastPose_);
            return;
        }
        if (t_ < duration) {
            evaluate(*clip_, params_, t_, kin_, stance_, lastPose_);
            return;
        }
        ++playsDone_;
        const int repeat = (int)fmaxf(1.0f, floorf(params_[REPEAT] + 0.5f));
        if (playsDone_ < repeat) {
            t_ = duration > 0.0f ? t_ - duration : 0.0f;
            evaluate(*clip_, params_, t_, kin_, stance_, lastPose_);
            return;
        }
        evaluate(*clip_, params_, duration, kin_, stance_, lastPose_);
        if (clip_->holdEnd) state_ = State::HOLD;
        else stop();
    }
};

}  // namespace anim
