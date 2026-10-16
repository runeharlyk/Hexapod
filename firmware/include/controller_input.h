#pragma once

#include <message_types.h>
#include <platform_shared/message.pb.h>

// The motion's command from a client's ControllerInputData. A field the client never sends decodes to 0, so an older
// client walks the speed-optimal stride.
inline CommandMsg commandFromInput(const socket_message_ControllerInputData &in) {
    return {in.left.x, in.left.y, in.right.x, in.right.y, in.height, in.speed, in.s1, in.feet_distance, in.stride};
}
