#pragma once

#include <cstdint>
#include <utils/math_utils.h>

#ifndef NUM_SERVO
#define NUM_SERVO 18
#endif

// Internal EventBus message structs (in-RAM only; the wire protocol is protobuf).

enum class MOTION_STATE { DEACTIVATED, IDLE, POSE, STAND, WALK, WALK_NN, ANIMATE };

// REQUEST: a mode asked for by a client, the controller or the firmware itself. BORROW: an animation
// play asking for ANIMATE. HANDBACK: the control task asking for the borrowed mode back. BORROW and
// HANDBACK are decided against the mode they find. APPLIED: MotionService reporting the mode it
// switched to, or kept for a refused BORROW, once it is in force; it is not a request. Mode observers
// outside MotionService see only APPLIED.
enum class ModeMsgKind { REQUEST, BORROW, HANDBACK, APPLIED };

struct ModeMsg {
    MOTION_STATE mode;
    ModeMsgKind kind = ModeMsgKind::REQUEST;
};

// TUNED is the CMA-ES-searched gait from simulation/src/resources/gait_library.json, emitted into
// gait_tuned.h by simulation/export_gait.py. Appended last so the existing wire values are stable.
enum class GaitType { TRI_GATE, BI_GATE, WAVE, RIPPLE, TUNED, AUTO };

struct GaitMsg {
    GaitType gait;
};

struct ServoAnglesMsg {
    float angles[NUM_SERVO];
};

struct ServoSignalMsg {
    int8_t id;
    uint16_t pwm;
};

struct ServoStateMsg {
    bool active;
};

struct IMUAnglesMsg {
    float rpy[3]{0, 0, 0};
    float temperature{-1};
    float heading{0};  // magnetometer compass heading in degrees; 0 when no magnetometer
    bool success{false};
};

// Who sent a command: the ESP-NOW handheld controller with a stick off centre, the handheld at rest, or any other
// client (the app, a gamepad through it, the Python facade, the host).
enum class CommandOrigin : uint8_t { CLIENT, HANDHELD, HANDHELD_MOVING };

struct CommandMsg {
    // stride last, so an initializer that omits it gets 0, the speed-optimal stride.
    float lx, ly, rx, ry, h, s, s1, fd, stride;
    // After the fields, so an initializer that omits it is a client's command.
    CommandOrigin origin = CommandOrigin::CLIENT;

    // Every field is a normalized stick or slider in [-1, 1]. Input arrives from the network, so a
    // command carrying NaN or infinity is rejected (returns false) and the rest are clamped, since
    // either would otherwise flow straight into the gait and IK.
    bool sanitize() {
        float *fields[] = {&lx, &ly, &rx, &ry, &h, &s, &s1, &fd, &stride};
        for (float *f : fields) {
            if (!std::isfinite(*f)) return false;
        }
        for (float *f : fields) *f = std::fmax(-1.0f, std::fmin(1.0f, *f));
        return true;
    }
};

struct BodyStateMsg {
    float omega, phi, psi, xm, ym, zm;
    float feet[6][4];

    void updateFeet(const float newFeet[6][4]) { COPY_2D_ARRAY_6x4(feet, newFeet); }
};

// Puppeteer target from the editor: body offsets and six leg targets, foot offsets or joint angles.
struct PoseMsg {
    float body[6];
    bool joints[6];
    float legs[6][3];
};

struct AnimationCommandMsg {
    char name[33];
    int paramCount;
    struct {
        int id;
        float value;
    } params[10];
};
