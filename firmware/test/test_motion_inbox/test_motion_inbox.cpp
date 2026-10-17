// The motion inbox (firmware/include/motion_inbox.h) on its own: what the other tasks post, the decisions taken as it
// arrives, and what the control task takes once a tick. Time is the argument the inbox is given.

#include <unity.h>
#include <motion_inbox.h>

namespace {

const Goal FORWARD_GOAL {GoalKind::FORWARD, 1.0f, 0, 0, 0.5f};
const CommandMsg STYLE {0, 0, 0, 0, 0, 1, 0, 0, 0};
int owners[2];
const GoalOwner OWNER {&owners[0], 7};
const GoalOwner OTHER_ID {&owners[0], 8};
const GoalOwner OTHER_TRANSPORT {&owners[1], 7};

int centred = 0;

MotionInbox::Mail take(MotionInbox &inbox, unsigned long nowMs = 0) {
    return inbox.take(nowMs, [] { ++centred; });
}

} // namespace

void setUp() { centred = 0; }
void tearDown() {}

void test_a_goal_is_taken_once_and_reported_settling_until_then() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::BI_GATE, {}, 0);
    const socket_message_GoalStatus pending = inbox.goalStatus();
    TEST_ASSERT_EQUAL_UINT32(1, pending.id);
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_SETTLING, pending.state);
    const MotionInbox::Mail mail = take(inbox);
    TEST_ASSERT_TRUE(mail.goal.has_value());
    TEST_ASSERT_EQUAL_UINT32(1, mail.goal->id);
    TEST_ASSERT_EQUAL(GaitType::BI_GATE, mail.goal->gait);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, mail.goal->goal.distance);
    TEST_ASSERT_FALSE(mail.goalCancelled);
    TEST_ASSERT_FALSE(take(inbox).goal.has_value());
    // Taken, the goal's status is the one the control task last pushed.
    socket_message_GoalStatus running = goalStatusOf(1, GoalState::RUNNING, FORWARD_GOAL, 0.5f, 1);
    inbox.report(running);
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_RUNNING, inbox.goalStatus().state);
}

void test_a_newer_goal_replaces_the_one_not_yet_taken() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    inbox.postGoal({GoalKind::TURN, 0, 1.0f, 0, 0.5f}, STYLE, GaitType::TRI_GATE, {}, 0);
    const MotionInbox::Mail mail = take(inbox);
    TEST_ASSERT_EQUAL_UINT32(2, mail.goal->id);
    TEST_ASSERT_EQUAL(GoalKind::TURN, mail.goal->goal.kind);
}

// A cancel ends the goal that drives the walk when it arrives, and no goal accepted after it.
void test_a_cancel_ends_only_the_goal_driving_when_it_arrives() {
    MotionInbox inbox;
    inbox.cancelGoal();
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    TEST_ASSERT_FALSE(take(inbox).goalCancelled);
    inbox.cancelGoal();
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    TEST_ASSERT_FALSE(take(inbox).goalCancelled);
    TEST_ASSERT_EQUAL_INT(0, centred);
    inbox.cancelGoal();
    const MotionInbox::Mail mail = take(inbox);
    TEST_ASSERT_TRUE(mail.goalCancelled);
    TEST_ASSERT_EQUAL_INT(1, centred);
    // The goal no longer drives, so a second cancel has nothing to end.
    inbox.cancelGoal();
    TEST_ASSERT_FALSE(take(inbox).goalCancelled);
}

void test_an_odometry_reset_is_refused_while_a_goal_drives_and_accepted_once_it_ends() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    TEST_ASSERT_FALSE(inbox.requestOdometryReset());
    TEST_ASSERT_FALSE(take(inbox).odometryReset);
    TEST_ASSERT_FALSE(inbox.requestOdometryReset());
    inbox.endGoalDrive([] { ++centred; });
    TEST_ASSERT_EQUAL_INT(1, centred);
    TEST_ASSERT_TRUE(inbox.requestOdometryReset());
    TEST_ASSERT_TRUE(take(inbox).odometryReset);
    TEST_ASSERT_FALSE(take(inbox).odometryReset);
}

// An owner heard within OWNER_SILENCE_MS keeps its goal; silent for longer, it loses it at the take. Another client's
// messages do not keep it.
void test_an_owner_silent_past_the_limit_loses_its_goal_at_the_take() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, OWNER, 1000);
    take(inbox, 1000);
    inbox.ownerHeard(OWNER, 2500);
    inbox.ownerHeard(OTHER_ID, 4000);
    inbox.ownerHeard(OTHER_TRANSPORT, 4000);
    TEST_ASSERT_FALSE(take(inbox, 2500 + MotionInbox::OWNER_SILENCE_MS).goalCancelled);
    TEST_ASSERT_TRUE(take(inbox, 2501 + MotionInbox::OWNER_SILENCE_MS).goalCancelled);
    TEST_ASSERT_EQUAL_INT(1, centred);
}

// A goal with no transport has no owner to fall silent.
void test_a_goal_without_an_owner_never_falls_silent() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    TEST_ASSERT_FALSE(take(inbox, 100 * MotionInbox::OWNER_SILENCE_MS).goalCancelled);
}

void test_the_owner_leaving_ends_its_goal_and_another_client_leaving_does_not() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, OWNER, 0);
    inbox.clientGone(OTHER_ID);
    inbox.clientGone(OTHER_TRANSPORT);
    TEST_ASSERT_FALSE(take(inbox).goalCancelled);
    inbox.clientGone(OWNER);
    TEST_ASSERT_TRUE(take(inbox).goalCancelled);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_goal_is_taken_once_and_reported_settling_until_then);
    RUN_TEST(test_a_newer_goal_replaces_the_one_not_yet_taken);
    RUN_TEST(test_a_cancel_ends_only_the_goal_driving_when_it_arrives);
    RUN_TEST(test_an_odometry_reset_is_refused_while_a_goal_drives_and_accepted_once_it_ends);
    RUN_TEST(test_an_owner_silent_past_the_limit_loses_its_goal_at_the_take);
    RUN_TEST(test_a_goal_without_an_owner_never_falls_silent);
    RUN_TEST(test_the_owner_leaving_ends_its_goal_and_another_client_leaving_does_not);
    return UNITY_END();
}
