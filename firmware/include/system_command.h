#pragma once

#include <cstdint>

#include <platform_shared/message.pb.h>

// What the robot does once a reply has gone out (after_reply.h): nothing, a restart, or a deep sleep.
enum class AfterReply : uint8_t { NOTHING, RESTART, SLEEP };

struct SystemCommandOutcome {
    uint32_t status;
    const char *reason; // why it was refused or failed, for the app to show; empty on success
    AfterReply after;
};

// Answers a system command before acting on it: the reply goes out, and the restart or sleep it asks for follows. The
// servos go down first for every accepted command, as the PCA9685 keeps its outputs across a restart and a deep sleep.
// A reset erases the settings and answers how that went, then restarts either way: erasing NVS leaves it
// uninitialised, so a robot kept running after a failed erase would run on a store it can no longer use.
// `robot` provides `deactivateServos()` and `eraseSettings()`, the latter true when every setting is gone.
template <typename Robot>
SystemCommandOutcome runSystemCommand(socket_message_SystemCommand command, Robot &robot) {
    switch (command) {
        case socket_message_SystemCommand_SYS_RESTART: robot.deactivateServos(); return {200, "", AfterReply::RESTART};
        case socket_message_SystemCommand_SYS_SLEEP: robot.deactivateServos(); return {200, "", AfterReply::SLEEP};
        case socket_message_SystemCommand_SYS_RESET:
            robot.deactivateServos();
            if (robot.eraseSettings()) return {200, "", AfterReply::RESTART};
            return {500, "Some settings could not be erased; restarting anyway", AfterReply::RESTART};
        default: return {400, "Unknown system command", AfterReply::NOTHING};
    }
}
