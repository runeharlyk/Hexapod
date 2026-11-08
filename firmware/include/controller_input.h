#pragma once

#include <message_types.h>
#include <platform_shared/message.pb.h>

// The motion's command from a client's ControllerInputData. A field the client never sends decodes to 0, so an older
// client walks the speed-optimal stride and holds no heading.
inline CommandMsg commandFromInput(const socket_message_ControllerInputData &in) {
    const auto &left = in.left, &right = in.right;
    CommandMsg c {left.x, left.y, right.x, right.y, in.height, in.speed, in.s1, in.feet_distance, in.stride};
    c.holdHeading = in.hold_heading;
    c.climb = in.climb;
    return c;
}
