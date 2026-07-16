#pragma once

#include <stdint.h>

/*
 * Wire format broadcast by the ESP-NOW handheld controller.
 *
 * v2: axes are calibrated + normalized to signed -1000..1000 (0 = centered);
 *     deadzone and per-axis inversion are applied on the controller.
 */

#define CONTROLLER_PACKET_VERSION 2

#define AXIS_FULL_SCALE 1000

// Bit positions within controller_packet_t.buttons.
#define BTN_LEFT (1u << 0)  // left joystick push-switch
#define BTN_RIGHT (1u << 1) // right joystick push-switch

typedef struct __attribute__((packed)) {
    uint8_t version;  // == CONTROLLER_PACKET_VERSION
    uint8_t buttons;  // bitmask of BTN_* (1 = pressed)
    int16_t left_x;   // normalized -AXIS_FULL_SCALE..+AXIS_FULL_SCALE
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    uint32_t seq;     // monotonically increasing send counter
} controller_packet_t;
