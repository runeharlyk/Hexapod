#pragma once

// The angles the motion publishes (ServoAnglesMsg) as they cross the wire (AnglesData): whole degrees in servo order,
// with the servos no pulse drives marked limp and given no angle. While every servo is limp no angle is known at all,
// and the list is empty, as a robot's that has driven no pose yet.

#include <cmath>
#include <message_types.h>
#include <platform_shared/message.pb.h>

constexpr uint32_t ALL_SERVOS_LIMP = (1u << NUM_SERVO) - 1;

inline socket_message_AnglesData anglesData(const ServoAnglesMsg &a) {
    socket_message_AnglesData out = socket_message_AnglesData_init_zero;
    if ((a.limp & ALL_SERVOS_LIMP) == ALL_SERVOS_LIMP) return out;
    out.limp = a.limp & ALL_SERVOS_LIMP;
    out.angles_count = NUM_SERVO;
    for (int i = 0; i < NUM_SERVO; i++)
        out.angles[i] = out.limp & (1u << i) ? 0 : static_cast<int32_t>(std::lround(a.angles[i]));
    return out;
}
