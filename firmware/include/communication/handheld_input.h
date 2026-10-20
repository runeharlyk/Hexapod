#pragma once

// The ESP-NOW handheld controller's sticks as a command, and the mode its mode button asks for, as the adapter
// (src/communication/espnow_adapter.cpp) publishes them and the virtual robot's tests drive them.

#include <cstddef>
#include <communication/controller_packet.h>
#include <message_types.h>

// The modes the mode button steps through, each press to the next, the last wrapping to the first.
inline constexpr MOTION_STATE HANDHELD_MODE_CYCLE[] = {MOTION_STATE::DEACTIVATED, MOTION_STATE::IDLE,
                                                       MOTION_STATE::STAND, MOTION_STATE::WALK};

// The mode a press of the mode button asks for, from the mode the robot reports. ANIMATE has no place in the cycle: the
// robot is in it for a play or for the ease out of IDLE, both on their way to a stand or a walk, so a press there asks
// for STAND, an explicit mode that waits for every leg to ease home (decideMode), and never cuts the servos or stops
// the legs half-eased in IDLE.
inline MOTION_STATE nextHandheldMode(MOTION_STATE reported) {
    if (reported == MOTION_STATE::ANIMATE) return MOTION_STATE::STAND;
    constexpr size_t count = sizeof(HANDHELD_MODE_CYCLE) / sizeof(HANDHELD_MODE_CYCLE[0]);
    size_t index = 0;
    for (size_t i = 0; i < count; ++i)
        if (HANDHELD_MODE_CYCLE[i] == reported) index = i;
    return HANDHELD_MODE_CYCLE[(index + 1) % count];
}

// The controller rescales from its own deadzone (80 raw counts by default, `deadzone <n>` on its CLI), so a thumbstick
// that drifts past it sends small axes from 1 up, at 50 Hz: an axis under 3 % of full scale is centred here, or the
// drift would walk, cancel goals and hold the handheld's window open.
constexpr int16_t HANDHELD_AXIS_THRESHOLD = AXIS_FULL_SCALE * 3 / 100;

inline int16_t handheldAxis(int16_t raw) {
    return raw > -HANDHELD_AXIS_THRESHOLD && raw < HANDHELD_AXIS_THRESHOLD ? 0 : raw;
}

// In STAND, the right stick with its button held sets the body height `height`, which every later command carries,
// and is then no tilt. The frame moves the robot when any of its axes is off centre, the trim's included though it
// leaves the command's sticks centred; its buttons do not, since the mode and the gait they press travel as messages
// of their own.
inline CommandMsg handheldCommand(const controller_packet_t &pkt, bool inStand, float &height) {
    constexpr float k = 1.0f / AXIS_FULL_SCALE;
    const int16_t lx = handheldAxis(pkt.left_x), ly = handheldAxis(pkt.left_y);
    const int16_t rx = handheldAxis(pkt.right_x), ry = handheldAxis(pkt.right_y);
    CommandMsg cmd {};
    cmd.lx = lx * k;
    cmd.ly = ly * k;
    cmd.rx = rx * k;
    cmd.ry = ry * k;
    if (inStand && (pkt.buttons & BTN_RIGHT)) {
        height = cmd.ry;
        cmd.ry = 0.0f;
    }
    cmd.h = height;
    const bool moving = lx != 0 || ly != 0 || rx != 0 || ry != 0;
    cmd.origin = moving ? CommandOrigin::HANDHELD_MOVING : CommandOrigin::HANDHELD;
    return cmd;
}
