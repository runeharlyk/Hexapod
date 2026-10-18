#pragma once

// The ESP-NOW handheld controller's sticks as a command, as the adapter (src/communication/espnow_adapter.cpp)
// publishes them and the virtual robot's tests drive them.

#include <communication/controller_packet.h>
#include <message_types.h>

// In STAND, the right stick with its button held sets the body height `height`, which every later command carries,
// and is then no tilt. The frame moves the robot when any of its axes is off centre, the trim's included though it
// leaves the command's sticks centred; its buttons do not, since the mode and the gait they press travel as messages
// of their own.
inline CommandMsg handheldCommand(const controller_packet_t &pkt, bool inStand, float &height) {
    constexpr float k = 1.0f / AXIS_FULL_SCALE;
    CommandMsg cmd {};
    cmd.lx = pkt.left_x * k;
    cmd.ly = pkt.left_y * k;
    cmd.rx = pkt.right_x * k;
    cmd.ry = pkt.right_y * k;
    if (inStand && (pkt.buttons & BTN_RIGHT)) {
        height = cmd.ry;
        cmd.ry = 0.0f;
    }
    cmd.h = height;
    const bool moving = pkt.left_x != 0 || pkt.left_y != 0 || pkt.right_x != 0 || pkt.right_y != 0;
    cmd.origin = moving ? CommandOrigin::HANDHELD_MOVING : CommandOrigin::HANDHELD;
    return cmd;
}
