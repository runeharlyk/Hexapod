#pragma once

// What the on-board status LED shows, apart from the driver that shows it: a colour per motion mode,
// steady while a client is connected and blinking while none is. Red is kept out of the mode colours
// for the battery warning (roadmap 2.1), which will be one more input here.

#include <cstdint>
#include <message_types.h>

namespace status_led {

struct Rgb {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    bool operator==(const Rgb&) const = default;
};

// The WS2812 is bright enough at an eighth of full scale; these are the values sent.
constexpr Rgb modeColour(MOTION_STATE mode) {
    switch (mode) {
        case MOTION_STATE::DEACTIVATED: return {6, 6, 6}; // dim white: servos limp, safe to handle
        case MOTION_STATE::IDLE: return {0, 0, 32};       // blue
        case MOTION_STATE::STAND: return {0, 32, 0};      // green
        case MOTION_STATE::WALK: return {0, 24, 24};      // cyan
        case MOTION_STATE::WALK_NN: return {24, 20, 0};   // yellow
        case MOTION_STATE::ANIMATE: return {32, 10, 0};   // orange
    }
    return {};
}

struct Inputs {
    MOTION_STATE mode;
    bool clientConnected;
};

constexpr int64_t BLINK_PERIOD_US = 1000000;

inline Rgb colourAt(const Inputs& in, int64_t nowUs) {
    if (!in.clientConnected && nowUs % BLINK_PERIOD_US >= BLINK_PERIOD_US / 2) return {};
    return modeColour(in.mode);
}

} // namespace status_led
