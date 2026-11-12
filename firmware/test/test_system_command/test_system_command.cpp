// Host tests of the system commands: what the reply says, and what the robot does before it and after it. Nothing that
// ends the link may happen before the reply has gone out, so a restart or a sleep is only ever returned, never done.

#include <unity.h>

#include <string>
#include <system_command.h>

namespace {

// Records each call in order, so a test can tell an erase made with the servos still up from one made after.
struct FakeRobot {
    bool eraseSucceeds = true;
    std::string calls;

    void deactivateServos() { calls += "deactivate;"; }
    bool eraseSettings() {
        calls += "erase;";
        return eraseSucceeds;
    }
};

} // namespace

void setUp() {}
void tearDown() {}

void test_a_restart_is_answered_and_left_for_after_the_reply() {
    FakeRobot robot;
    const SystemCommandOutcome outcome = runSystemCommand(socket_message_SystemCommand_SYS_RESTART, robot);
    TEST_ASSERT_EQUAL_UINT32(200, outcome.status);
    TEST_ASSERT_EQUAL_STRING("", outcome.reason);
    TEST_ASSERT_TRUE(outcome.after == AfterReply::RESTART);
    TEST_ASSERT_EQUAL_STRING("deactivate;", robot.calls.c_str());
}

void test_a_sleep_is_answered_and_left_for_after_the_reply() {
    FakeRobot robot;
    const SystemCommandOutcome outcome = runSystemCommand(socket_message_SystemCommand_SYS_SLEEP, robot);
    TEST_ASSERT_EQUAL_UINT32(200, outcome.status);
    TEST_ASSERT_TRUE(outcome.after == AfterReply::SLEEP);
    TEST_ASSERT_EQUAL_STRING("deactivate;", robot.calls.c_str());
}

void test_a_reset_erases_with_the_servos_down_and_then_restarts() {
    FakeRobot robot;
    const SystemCommandOutcome outcome = runSystemCommand(socket_message_SystemCommand_SYS_RESET, robot);
    TEST_ASSERT_EQUAL_UINT32(200, outcome.status);
    TEST_ASSERT_TRUE(outcome.after == AfterReply::RESTART);
    TEST_ASSERT_EQUAL_STRING("deactivate;erase;", robot.calls.c_str());
}

void test_a_failed_erase_is_reported_and_still_restarts() {
    FakeRobot robot;
    robot.eraseSucceeds = false;
    const SystemCommandOutcome outcome = runSystemCommand(socket_message_SystemCommand_SYS_RESET, robot);
    TEST_ASSERT_EQUAL_UINT32(500, outcome.status);
    TEST_ASSERT_EQUAL_STRING("Some settings could not be erased; restarting anyway", outcome.reason);
    TEST_ASSERT_TRUE(outcome.after == AfterReply::RESTART);
}

// An empty request decodes as SYS_NONE; a value from a newer app decodes as itself. Neither may touch the robot.
void test_an_empty_or_unknown_command_is_refused_and_touches_nothing() {
    for (const auto command : {socket_message_SystemCommand_SYS_NONE, static_cast<socket_message_SystemCommand>(99)}) {
        FakeRobot robot;
        const SystemCommandOutcome outcome = runSystemCommand(command, robot);
        TEST_ASSERT_EQUAL_UINT32(400, outcome.status);
        TEST_ASSERT_EQUAL_STRING("Unknown system command", outcome.reason);
        TEST_ASSERT_TRUE(outcome.after == AfterReply::NOTHING);
        TEST_ASSERT_EQUAL_STRING("", robot.calls.c_str());
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_a_restart_is_answered_and_left_for_after_the_reply);
    RUN_TEST(test_a_sleep_is_answered_and_left_for_after_the_reply);
    RUN_TEST(test_a_reset_erases_with_the_servos_down_and_then_restarts);
    RUN_TEST(test_a_failed_erase_is_reported_and_still_restarts);
    RUN_TEST(test_an_empty_or_unknown_command_is_refused_and_touches_nothing);
    return UNITY_END();
}
