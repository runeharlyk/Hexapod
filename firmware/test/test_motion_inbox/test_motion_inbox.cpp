// The motion inbox (firmware/include/motion_inbox.h) on its own: what the other tasks post, the decisions taken as it
// arrives, and what the control task takes once a tick. Time is the argument the inbox is given.

#include <unity.h>
#include <motion_inbox.h>

namespace {

const Goal FORWARD_GOAL {GoalKind::FORWARD, 1.0f, 0, 0, 0.5f};
const CommandMsg STYLE {0, 0, 0, 0, 0, 1, 0, 0, 0};
const CommandMsg CENTRED {0, 0, 0, 0.5f, -0.5f, 1, 1, 0.5f, -1};
const CommandMsg FORWARD_STICK {0, 1, 0, 0, 0, 0, 0, 0, 0};
int owners[2];
const GoalOwner OWNER {&owners[0], 7};
const GoalOwner OTHER_ID {&owners[0], 8};
const GoalOwner OTHER_TRANSPORT {&owners[1], 7};

// A goal posted and taken: it drives the walk.
void driving(MotionInbox &inbox) {
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    inbox.take(0);
}

} // namespace

void setUp() {}
void tearDown() {}

void test_a_goal_is_taken_once_and_reported_settling_until_then() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::BI_GATE, {}, 0);
    const socket_message_GoalStatus pending = inbox.goalStatus();
    TEST_ASSERT_EQUAL_UINT32(1, pending.id);
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_SETTLING, pending.state);
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_TRUE(mail.goal.has_value());
    TEST_ASSERT_EQUAL_UINT32(1, mail.goal->id);
    TEST_ASSERT_EQUAL(GaitType::BI_GATE, mail.goal->gait);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, mail.goal->goal.distance);
    TEST_ASSERT_FALSE(mail.goalCancelled);
    TEST_ASSERT_FALSE(inbox.take(0).goal.has_value());
    // Taken, the goal's status is the one the control task last pushed.
    socket_message_GoalStatus running = goalStatusOf(1, GoalState::RUNNING, FORWARD_GOAL, 0.5f, 1);
    inbox.report(running);
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_RUNNING, inbox.goalStatus().state);
}

void test_a_newer_goal_replaces_the_one_not_yet_taken() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    inbox.postGoal({GoalKind::TURN, 0, 1.0f, 0, 0.5f}, STYLE, GaitType::TRI_GATE, {}, 0);
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_EQUAL_UINT32(2, mail.goal->id);
    TEST_ASSERT_EQUAL(GoalKind::TURN, mail.goal->goal.kind);
}

// A cancel ends the goal that drives the walk when it arrives, and no goal accepted after it.
void test_a_cancel_ends_only_the_goal_driving_when_it_arrives() {
    MotionInbox inbox;
    inbox.cancelGoal();
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
    inbox.cancelGoal();
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
    inbox.cancelGoal();
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_TRUE(mail.goalCancelled);
    TEST_ASSERT_TRUE(mail.goalDriveEnded);
    // The goal no longer drives, so a second cancel has nothing to end.
    inbox.cancelGoal();
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
}

void test_an_odometry_reset_is_refused_while_a_goal_drives_and_accepted_once_it_ends() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    TEST_ASSERT_FALSE(inbox.requestOdometryReset());
    TEST_ASSERT_FALSE(inbox.take(0).odometryReset);
    TEST_ASSERT_FALSE(inbox.requestOdometryReset());
    TEST_ASSERT_TRUE(inbox.endGoalDrive());
    TEST_ASSERT_FALSE(inbox.endGoalDrive());
    TEST_ASSERT_TRUE(inbox.requestOdometryReset());
    TEST_ASSERT_TRUE(inbox.take(0).odometryReset);
    TEST_ASSERT_FALSE(inbox.take(0).odometryReset);
}

// An owner heard within OWNER_SILENCE_MS keeps its goal; silent for longer, it loses it at the take. Another client's
// messages do not keep it.
void test_an_owner_silent_past_the_limit_loses_its_goal_at_the_take() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, OWNER, 1000);
    inbox.take(1000);
    inbox.ownerHeard(OWNER, 2500);
    inbox.ownerHeard(OTHER_ID, 4000);
    inbox.ownerHeard(OTHER_TRANSPORT, 4000);
    TEST_ASSERT_FALSE(inbox.take(2500 + MotionInbox::OWNER_SILENCE_MS).goalCancelled);
    const MotionInbox::Mail mail = inbox.take(2501 + MotionInbox::OWNER_SILENCE_MS);
    TEST_ASSERT_TRUE(mail.goalCancelled);
    TEST_ASSERT_TRUE(mail.goalDriveEnded);
}

// A goal with no transport has no owner to fall silent.
void test_a_goal_without_an_owner_never_falls_silent() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    TEST_ASSERT_FALSE(inbox.take(100 * MotionInbox::OWNER_SILENCE_MS).goalCancelled);
}

void test_the_owner_leaving_ends_its_goal_and_another_client_leaving_does_not() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, OWNER, 0);
    inbox.clientGone(OTHER_ID);
    inbox.clientGone(OTHER_TRANSPORT);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
    inbox.clientGone(OWNER);
    TEST_ASSERT_TRUE(inbox.take(0).goalCancelled);
}

// Each command is a full snapshot of the sticks and sliders, so only the newest is walked; its arrival starts the
// dead-man's window.
void test_a_command_is_taken_once_with_its_arrival_and_the_newest_wins() {
    MotionInbox inbox;
    TEST_ASSERT_FALSE(inbox.take(0).command.has_value());
    inbox.postCommand(CENTRED, 10);
    inbox.postCommand(FORWARD_STICK, 12);
    const MotionInbox::Mail mail = inbox.take(15);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_EQUAL_FLOAT(1, mail.command->ly);
    TEST_ASSERT_EQUAL_UINT32(12, mail.commandMs);
    TEST_ASSERT_FALSE(inbox.take(20).command.has_value());
}

// While a goal drives, the heartbeats' centred commands are no command at all, and a stick off centre takes the walk
// back: it ends the goal, which no longer drives, and is walked.
void test_while_a_goal_drives_a_centred_command_is_swallowed_and_a_stick_takes_the_walk_back() {
    MotionInbox inbox;
    driving(inbox);
    inbox.postCommand(CENTRED, 10);
    MotionInbox::Mail mail = inbox.take(15);
    TEST_ASSERT_FALSE(mail.command.has_value());
    TEST_ASSERT_FALSE(mail.goalCancelled);
    TEST_ASSERT_TRUE(inbox.goalDrives());
    for (const CommandMsg stick :
         {CommandMsg {0.1f, 0, 0, 0, 0, 0, 0, 0, 0}, CommandMsg {0, -0.1f, 0, 0, 0, 0, 0, 0, 0},
          CommandMsg {0, 0, 0.1f, 0, 0, 0, 0, 0, 0}}) {
        driving(inbox);
        inbox.postCommand(stick, 20);
        TEST_ASSERT_FALSE(inbox.goalDrives());
        // The operator has the sticks: the walk is not centred under them.
        mail = inbox.take(25);
        TEST_ASSERT_TRUE(mail.command.has_value());
        TEST_ASSERT_TRUE(mail.goalCancelled);
        TEST_ASSERT_FALSE(mail.goalDriveEnded);
    }
}

// A command posted before the goal was accepted is walked first and ends nothing.
void test_a_command_posted_before_the_goal_leaves_it_alone() {
    MotionInbox inbox;
    inbox.postCommand(FORWARD_STICK, 10);
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 11);
    const MotionInbox::Mail mail = inbox.take(15);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_TRUE(mail.goal.has_value());
    TEST_ASSERT_FALSE(mail.goalCancelled);
    TEST_ASSERT_TRUE(inbox.goalDrives());
}

void test_a_gait_is_taken_once_and_the_newest_wins() {
    MotionInbox inbox;
    TEST_ASSERT_FALSE(inbox.take(0).gait.has_value());
    inbox.postGait(GaitType::WAVE);
    inbox.postGait(GaitType::BI_GATE);
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_TRUE(mail.gait.has_value());
    TEST_ASSERT_EQUAL(GaitType::BI_GATE, *mail.gait);
    TEST_ASSERT_FALSE(inbox.take(0).gait.has_value());
}

// A goal's timeout was reckoned at its gait's pace, so a gait other than the one selected ends it when it arrives,
// though a newer gait replaces it before the control task takes it; the gait already selected is no change.
void test_a_gait_other_than_the_selection_ends_the_goal() {
    MotionInbox inbox;
    driving(inbox);
    inbox.postGait(GaitType::TRI_GATE);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
    inbox.postGait(GaitType::BI_GATE);
    inbox.postGait(GaitType::TRI_GATE);
    TEST_ASSERT_TRUE(inbox.take(0).goalCancelled);
    // BI was selected and replaced, so TRI is the selection again.
    driving(inbox);
    inbox.postGait(GaitType::TRI_GATE);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
}

// The gait a goal walks becomes the selection without ending it, so a client sending that gait next changes nothing.
void test_a_goals_own_gait_is_the_selection() {
    MotionInbox inbox;
    driving(inbox);
    inbox.selectGait(GaitType::BI_GATE);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
    inbox.postGait(GaitType::BI_GATE);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
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
    RUN_TEST(test_a_command_is_taken_once_with_its_arrival_and_the_newest_wins);
    RUN_TEST(test_while_a_goal_drives_a_centred_command_is_swallowed_and_a_stick_takes_the_walk_back);
    RUN_TEST(test_a_command_posted_before_the_goal_leaves_it_alone);
    RUN_TEST(test_a_gait_is_taken_once_and_the_newest_wins);
    RUN_TEST(test_a_gait_other_than_the_selection_ends_the_goal);
    RUN_TEST(test_a_goals_own_gait_is_the_selection);
    return UNITY_END();
}
