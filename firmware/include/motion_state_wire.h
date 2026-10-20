#pragma once

// MOTION_STATE as it crosses the wire: ModeData, the telemetry's mode and the virtual robot's interface carry it as its
// ModesEnum value. The proto reserves POSE's 2, and a value the robot has no mode for is refused, never cast.

#include <optional>
#include <message_types.h>
#include <platform_shared/message.pb.h>

static_assert(static_cast<int>(MOTION_STATE::DEACTIVATED) == socket_message_ModesEnum_DEACTIVATED &&
                  static_cast<int>(MOTION_STATE::IDLE) == socket_message_ModesEnum_IDLE &&
                  static_cast<int>(MOTION_STATE::STAND) == socket_message_ModesEnum_STAND &&
                  static_cast<int>(MOTION_STATE::WALK) == socket_message_ModesEnum_WALK &&
                  static_cast<int>(MOTION_STATE::WALK_NN) == socket_message_ModesEnum_WALK_NN &&
                  static_cast<int>(MOTION_STATE::ANIMATE) == socket_message_ModesEnum_ANIMATE,
              "MOTION_STATE carries ModesEnum by value");

inline std::optional<MOTION_STATE> motionStateFromWire(int value) {
    switch (value) {
        case socket_message_ModesEnum_DEACTIVATED: return MOTION_STATE::DEACTIVATED;
        case socket_message_ModesEnum_IDLE: return MOTION_STATE::IDLE;
        case socket_message_ModesEnum_STAND: return MOTION_STATE::STAND;
        case socket_message_ModesEnum_WALK: return MOTION_STATE::WALK;
        case socket_message_ModesEnum_WALK_NN: return MOTION_STATE::WALK_NN;
        case socket_message_ModesEnum_ANIMATE: return MOTION_STATE::ANIMATE;
        default: return std::nullopt;
    }
}
