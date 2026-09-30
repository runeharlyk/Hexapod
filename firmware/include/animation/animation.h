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
constexpr int NAME_MAX = 32;
constexpr int DESCRIPTION_MAX = 96;
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
    bool onBody = true;  // channel is a BodyAxis, else leg*3 + axis
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
    char name[NAME_MAX + 1] = {0};
    char description[DESCRIPTION_MAX + 1] = {0};
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
    if (n < 1 || n > NAME_MAX) return false;
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
    if (!validName(c.name)) return "name must be 1-32 characters of [a-z0-9_-]";
    if (strlen(c.description) > (size_t)DESCRIPTION_MAX) return "description longer than 96 bytes";
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
        if (o.onBody && (o.channel < 0 || o.channel > 5)) return "overlay body_axis out of range";
        if (!o.onBody && (o.channel < 0 || o.channel > 17)) return "overlay foot_channel out of range";
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

}  // namespace anim
