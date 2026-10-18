// The motion inbox (firmware/include/motion_inbox.h) on its own: what the other tasks post, the decisions taken as it
// arrives, and what the control task takes once a tick. Time is the argument the inbox is given.

#include <unity.h>
#include <motion_inbox.h>

namespace {

const Goal FORWARD_GOAL {GoalKind::FORWARD, 1.0f, 0, 0, 0.5f};
const CommandMsg STYLE {0, 0, 0, 0, 0, 1, 0, 0, 0};
const CommandMsg CENTRED {0, 0, 0, 0.5f, -0.5f, 1, 1, 0.5f, -1};
const CommandMsg FORWARD_STICK {0, 1, 0, 0, 0, 0, 0, 0, 0};
const CommandMsg HANDHELD_FORWARD {0, 0.5f, 0, 0, 0, 0, 0, 0, 0, CommandOrigin::HANDHELD_MOVING};
const CommandMsg HANDHELD_CENTRED {0, 0, 0, 0, 0.2f, 0, 0, 0, 0, CommandOrigin::HANDHELD};
constexpr unsigned long WINDOW_MS = MotionInbox::HANDHELD_PRIORITY_MS;
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

// The control task reads the clock before the take locks the inbox, so an owner heard, or a goal posted, after that
// read and before the lock carries a time later than the take's. That is no silence.
void test_an_owner_heard_after_the_takes_clock_read_keeps_its_goal() {
    MotionInbox inbox;
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, OWNER, 1000);
    inbox.take(1000);
    inbox.ownerHeard(OWNER, 5001);
    TEST_ASSERT_FALSE(inbox.take(5000).goalCancelled);
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, OWNER, 7001);
    const MotionInbox::Mail mail = inbox.take(7000);
    TEST_ASSERT_TRUE(mail.goal.has_value());
    TEST_ASSERT_FALSE(mail.goalCancelled);
    TEST_ASSERT_TRUE(inbox.goalDrives());
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
    inbox.postGait(GaitType::BI_GATE);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
}

// A gait posted between a take and the goal style's selection has ended that goal and is the gait the next take puts
// in force, so it stays the selection: a client re-sending it ends nothing, and a change back to the goal's gait does.
void test_a_gait_posted_before_the_goals_selection_stays_the_selection() {
    MotionInbox inbox;
    driving(inbox);
    inbox.postGait(GaitType::BI_GATE);
    inbox.selectGait(GaitType::WAVE);
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_EQUAL(GaitType::BI_GATE, *mail.gait);
    TEST_ASSERT_TRUE(mail.goalCancelled);
    driving(inbox);
    inbox.postGait(GaitType::BI_GATE);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
    inbox.postGait(GaitType::WAVE);
    TEST_ASSERT_TRUE(inbox.take(0).goalCancelled);
}

// A gait posted equal to the selection ended nothing, but the goal's own gait then differs from the one the next take
// puts in force, so the goal would wait for a gait it is not in: it ends, rather than timing out unstarted.
void test_a_pending_gait_other_than_the_goals_own_ends_the_goal() {
    MotionInbox inbox;
    driving(inbox);
    inbox.postGait(GaitType::TRI_GATE);
    inbox.selectGait(GaitType::BI_GATE);
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_EQUAL(GaitType::TRI_GATE, *mail.gait);
    TEST_ASSERT_TRUE(mail.goalCancelled);
    // The goal's own gait pending is no change.
    driving(inbox);
    inbox.postGait(GaitType::TRI_GATE);
    inbox.selectGait(GaitType::TRI_GATE);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
}

// An animation call is honoured from a play's borrow queued, not only once it is applied: the play it follows is then
// already loaded, and the control task takes the borrow at its next tick.
void test_the_robot_animates_from_a_borrow_queued_until_the_mode_is_handed_back() {
    MotionInbox inbox;
    TEST_ASSERT_FALSE(inbox.animating());
    inbox.postMode({MOTION_STATE::WALK}, false);
    inbox.postMode({MOTION_STATE::ANIMATE, ModeMsgKind::HANDBACK}, false);
    TEST_ASSERT_FALSE(inbox.animating());
    inbox.postMode({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW}, false);
    TEST_ASSERT_TRUE(inbox.animating());
    inbox.take(0);
    TEST_ASSERT_FALSE(inbox.animating());
    inbox.noteMode({MOTION_STATE::ANIMATE, true, MOTION_STATE::STAND});
    TEST_ASSERT_TRUE(inbox.animating());
    inbox.noteMode({MOTION_STATE::STAND, false, MOTION_STATE::STAND});
    TEST_ASSERT_FALSE(inbox.animating());
}

// A borrow and a hand-back are decided against the mode they find, so modes are taken in the order they arrived, none
// merged: a DEACTIVATED and a borrow posted in one tick are two, the deactivation first. An APPLIED is no request.
void test_modes_are_taken_in_arrival_order_none_merged() {
    MotionInbox inbox;
    inbox.postMode({MOTION_STATE::DEACTIVATED}, false);
    inbox.postMode({MOTION_STATE::STAND, ModeMsgKind::APPLIED}, false);
    inbox.postMode({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW}, false);
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_EQUAL_INT(2, mail.modes.count);
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, mail.modes.items[0].mode);
    TEST_ASSERT_EQUAL(ModeMsgKind::REQUEST, mail.modes.items[0].kind);
    TEST_ASSERT_EQUAL(MOTION_STATE::ANIMATE, mail.modes.items[1].mode);
    TEST_ASSERT_EQUAL(ModeMsgKind::BORROW, mail.modes.items[1].kind);
    TEST_ASSERT_EQUAL_UINT(0, mail.modes.dropped);
    TEST_ASSERT_EQUAL_INT(0, inbox.take(0).modes.count);
}

// Full, the queue keeps the newest intent: the oldest mode goes, counted, and a DEACTIVATED posted last is kept.
void test_a_full_mode_queue_drops_the_oldest_and_keeps_a_newest_deactivation() {
    MotionInbox inbox;
    for (int i = 0; i < MotionInbox::MODE_DEPTH; ++i)
        inbox.postMode({i % 2 ? MOTION_STATE::WALK : MOTION_STATE::STAND}, false);
    inbox.postMode({MOTION_STATE::DEACTIVATED}, false);
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_EQUAL_INT(MotionInbox::MODE_DEPTH, mail.modes.count);
    TEST_ASSERT_EQUAL_UINT(1, mail.modes.dropped);
    TEST_ASSERT_EQUAL(MOTION_STATE::WALK, mail.modes.items[0].mode);
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, mail.modes.items[MotionInbox::MODE_DEPTH - 1].mode);
    TEST_ASSERT_EQUAL_UINT(0, inbox.take(0).modes.dropped);
}

// The replies judge the mode the robot reports: the one the control task noted as it applied it, not one queued or
// taken and not yet applied.
void test_the_replies_judge_the_mode_the_control_task_noted_last() {
    MotionInbox inbox;
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, inbox.appliedMode());
    inbox.postMode({MOTION_STATE::STAND}, false);
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, inbox.appliedMode());
    inbox.take(0);
    TEST_ASSERT_EQUAL(MOTION_STATE::DEACTIVATED, inbox.appliedMode());
    inbox.noteMode({MOTION_STATE::STAND, false, MOTION_STATE::STAND});
    TEST_ASSERT_EQUAL(MOTION_STATE::STAND, inbox.appliedMode());
}

// The end a mode implies is decided by the arbiter from the mode applied last: a borrow from a stand or a walk enters
// ANIMATE and ends the goal, and one from a mode that drives no servo is refused and ends nothing.
void test_a_borrow_ends_a_goal_only_from_a_mode_it_would_animate() {
    MotionInbox inbox;
    inbox.noteMode({MOTION_STATE::WALK, false, MOTION_STATE::STAND});
    driving(inbox);
    inbox.postMode({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW}, false);
    TEST_ASSERT_TRUE(inbox.take(0).goalCancelled);
    inbox.noteMode({MOTION_STATE::IDLE, false, MOTION_STATE::STAND});
    driving(inbox);
    inbox.postMode({MOTION_STATE::ANIMATE, ModeMsgKind::BORROW}, false);
    TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
}

// A goal walks in WALK: a mode that leaves it ends a goal accepted before it, and not one accepted after it.
void test_a_mode_leaving_walk_ends_only_a_goal_posted_before_it() {
    {
        MotionInbox inbox;
        inbox.postMode({MOTION_STATE::STAND}, false);
        inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
        inbox.postMode({MOTION_STATE::WALK}, false);
        TEST_ASSERT_FALSE(inbox.take(0).goalCancelled);
        inbox.postMode({MOTION_STATE::STAND}, false);
        TEST_ASSERT_TRUE(inbox.take(0).goalCancelled);
    }
    MotionInbox inbox;
    inbox.postMode({MOTION_STATE::IDLE}, false);
    inbox.postMode({MOTION_STATE::STAND}, false);
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    const MotionInbox::Mail mail = inbox.take(0);
    TEST_ASSERT_TRUE(mail.goal.has_value());
    TEST_ASSERT_FALSE(mail.goalCancelled);
}

// A reply off the control task carries one whole tick's pose: the one the control task noted last.
void test_the_replies_carry_the_pose_the_control_task_noted_last() {
    MotionInbox inbox;
    inbox.noteOdometry({.x = 0.1f, .y = 0.2f, .yaw = 0.3f, .kinematicYaw = 0.4f, .distance = 0.5f});
    const OdometryEstimate pose = inbox.odometry();
    TEST_ASSERT_EQUAL_FLOAT(0.1f, pose.x);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, pose.y);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, pose.yaw);
    TEST_ASSERT_EQUAL_FLOAT(0.4f, pose.kinematicYaw);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, pose.distance);
    inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 0);
    socket_message_GoalStatus status = inbox.goalStatus();
    TEST_ASSERT_EQUAL_FLOAT(0.1f, status.x);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, status.y);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, status.yaw);
    inbox.take(0);
    inbox.report(goalStatusOf(1, GoalState::RUNNING, FORWARD_GOAL, 0.5f, 1));
    inbox.noteOdometry({.x = 1.0f, .y = 2.0f, .yaw = 3.0f, .kinematicYaw = 0, .distance = 0});
    status = inbox.goalStatus();
    TEST_ASSERT_EQUAL(socket_message_GoalState_GOAL_RUNNING, status.state);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, status.y);
}

// The handheld's priority: while it has moved a stick within HANDHELD_PRIORITY_MS, it drives, and a client's command,
// the app's heartbeat above all, is dropped, centred or not, posture and all.
void test_a_clients_command_within_the_handheld_window_is_dropped() {
    MotionInbox inbox;
    inbox.postCommand(HANDHELD_FORWARD, 1000);
    inbox.postCommand(CENTRED, 1010);
    MotionInbox::Mail mail = inbox.take(1015);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, mail.command->ly);
    TEST_ASSERT_EQUAL(CommandOrigin::HANDHELD_MOVING, mail.command->origin);
    // Off centre too, and its arrival is no command's: the dead-man counts from the handheld's.
    inbox.postCommand(FORWARD_STICK, 1100);
    mail = inbox.take(1105);
    TEST_ASSERT_FALSE(mail.command.has_value());
    TEST_ASSERT_EQUAL_UINT32(1000, mail.commandMs);
}

// The window ends HANDHELD_PRIORITY_MS after the handheld's last frame with a stick off centre, to the millisecond.
void test_the_handheld_window_ends_600_ms_after_its_last_moving_frame() {
    MotionInbox inbox;
    inbox.postCommand(HANDHELD_FORWARD, 1000);
    inbox.take(1000);
    inbox.postCommand(CENTRED, 1000 + WINDOW_MS - 1);
    TEST_ASSERT_FALSE(inbox.take(1000 + WINDOW_MS - 1).command.has_value());
    inbox.postCommand(CENTRED, 1000 + WINDOW_MS);
    const MotionInbox::Mail mail = inbox.take(1000 + WINDOW_MS);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_EQUAL_UINT32(1000 + WINDOW_MS, mail.commandMs);
}

// The handheld's own centred frame is its stop, so it is walked at once, and it keeps no window open.
void test_the_handhelds_centred_frame_is_taken_and_opens_no_window() {
    MotionInbox inbox;
    inbox.postCommand(HANDHELD_FORWARD, 1000);
    inbox.postCommand(HANDHELD_CENTRED, 1100);
    MotionInbox::Mail mail = inbox.take(1100);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_EQUAL_FLOAT(0, mail.command->ly);
    TEST_ASSERT_EQUAL_UINT32(1100, mail.commandMs);
    inbox.postCommand(HANDHELD_CENTRED, 1500);
    inbox.postCommand(CENTRED, 1000 + WINDOW_MS);
    mail = inbox.take(1000 + WINDOW_MS);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_EQUAL(CommandOrigin::CLIENT, mail.command->origin);
}

// Before any handheld frame with a stick off centre there is no window, from boot on.
void test_a_robot_that_never_saw_the_handheld_move_takes_every_command() {
    MotionInbox inbox;
    inbox.postCommand(HANDHELD_CENTRED, 0);
    inbox.postCommand(FORWARD_STICK, 1);
    TEST_ASSERT_TRUE(inbox.take(1).command.has_value());
    TEST_ASSERT_TRUE(inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, {}, 2));
}

// A goal asked for while the handheld drives is refused, decided under the lock of the posts; once the window ends it
// is accepted.
void test_a_goal_within_the_handheld_window_is_refused() {
    MotionInbox inbox;
    inbox.postCommand(HANDHELD_FORWARD, 1000);
    TEST_ASSERT_FALSE(inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, OWNER, 1000 + WINDOW_MS - 1));
    TEST_ASSERT_FALSE(inbox.take(1000 + WINDOW_MS - 1).goal.has_value());
    TEST_ASSERT_FALSE(inbox.goalDrives());
    TEST_ASSERT_TRUE(inbox.postGoal(FORWARD_GOAL, STYLE, GaitType::TRI_GATE, OWNER, 1000 + WINDOW_MS));
    TEST_ASSERT_TRUE(inbox.take(1000 + WINDOW_MS).goal.has_value());
}

// The handheld moving any axis ends a goal, its tilt and its height trim included, which leave the command's walk
// sticks centred; a client's command so centred leaves the goal alone.
void test_a_handheld_moving_frame_ends_a_goal_though_its_walk_sticks_are_centred() {
    MotionInbox inbox;
    driving(inbox);
    inbox.postCommand(CENTRED, 10);
    TEST_ASSERT_FALSE(inbox.take(10).goalCancelled);
    CommandMsg tilt = CENTRED;
    tilt.origin = CommandOrigin::HANDHELD_MOVING;
    inbox.postCommand(tilt, 20);
    const MotionInbox::Mail mail = inbox.take(20);
    TEST_ASSERT_TRUE(mail.goalCancelled);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_FALSE(inbox.goalDrives());
}

// Outside its window an idle handheld's centred frame is no walk command: it carries its posture, and the walk, the
// sticks, speed and stride, stays the one of the command before it, a pending one's merged in, whose arrival the
// dead-man keeps counting from. Within the window the same frame is the handheld's stop, taken whole.
void test_an_idle_handhelds_frame_outside_its_window_keeps_the_walk() {
    MotionInbox inbox;
    const CommandMsg walk {0.1f, 1, 0.2f, 0, 0.6f, 0.5f, 0, 0, 0.3f};
    inbox.postCommand(walk, 1000);
    inbox.postCommand(HANDHELD_CENTRED, 1010);
    MotionInbox::Mail mail = inbox.take(1015);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_FALSE(mail.commandKeepsWalk);
    TEST_ASSERT_EQUAL_UINT32(1000, mail.commandMs);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, mail.command->lx);
    TEST_ASSERT_EQUAL_FLOAT(1, mail.command->ly);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, mail.command->rx);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, mail.command->s);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, mail.command->stride);
    TEST_ASSERT_EQUAL_FLOAT(HANDHELD_CENTRED.h, mail.command->h);
    inbox.postCommand(HANDHELD_CENTRED, 1100);
    mail = inbox.take(1105);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_TRUE(mail.commandKeepsWalk);
    TEST_ASSERT_EQUAL_UINT32(1000, mail.commandMs);
    inbox.postCommand(HANDHELD_FORWARD, 2000);
    inbox.take(2000);
    inbox.postCommand(HANDHELD_CENTRED, 2100);
    mail = inbox.take(2100);
    TEST_ASSERT_FALSE(mail.commandKeepsWalk);
    TEST_ASSERT_EQUAL_UINT32(2100, mail.commandMs);
    TEST_ASSERT_EQUAL_FLOAT(0, mail.command->ly);
}

// The clock wraps after 49 days; the window is a difference, so it spans the wrap. A poster reads its clock before the
// lock, so one read before the handheld's frame was posted is within the window too.
void test_the_handheld_window_spans_the_clocks_wrap() {
    MotionInbox inbox;
    const unsigned long moved = static_cast<unsigned long>(-1) - 100;
    inbox.postCommand(HANDHELD_FORWARD, moved);
    inbox.take(moved);
    inbox.postCommand(CENTRED, moved - 1);
    TEST_ASSERT_FALSE(inbox.take(moved).command.has_value());
    inbox.postCommand(CENTRED, moved + WINDOW_MS - 1);
    TEST_ASSERT_FALSE(inbox.take(moved + WINDOW_MS - 1).command.has_value());
    inbox.postCommand(CENTRED, moved + WINDOW_MS);
    TEST_ASSERT_TRUE(inbox.take(moved + WINDOW_MS).command.has_value());
}

// The robot stays stopped after the handheld lets go: from the window's end, a client's stick off centre, the stale
// stream of a script or of a stick the app held through the drive, is dropped until a client sends a centred command,
// which is taken. A stick after it is obeyed.
void test_after_the_handheld_window_a_clients_stick_waits_for_a_centred_command() {
    MotionInbox inbox;
    inbox.postCommand(HANDHELD_FORWARD, 1000);
    inbox.take(1000);
    inbox.postCommand(FORWARD_STICK, 1000 + WINDOW_MS);
    TEST_ASSERT_FALSE(inbox.take(1000 + WINDOW_MS).command.has_value());
    inbox.postCommand(FORWARD_STICK, 5000);
    TEST_ASSERT_FALSE(inbox.take(5000).command.has_value());
    inbox.postCommand(CENTRED, 5100);
    MotionInbox::Mail mail = inbox.take(5100);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_EQUAL_FLOAT(0, mail.command->ly);
    inbox.postCommand(FORWARD_STICK, 5200);
    mail = inbox.take(5200);
    TEST_ASSERT_TRUE(mail.command.has_value());
    TEST_ASSERT_EQUAL_FLOAT(1, mail.command->ly);
}

// Only a centred command after the window counts: one dropped within it, and the handheld's own centred frames, leave
// a client's stick waiting. The handheld moving again sets the wait again.
void test_only_a_clients_centred_command_after_the_window_ends_the_wait() {
    MotionInbox inbox;
    inbox.postCommand(HANDHELD_FORWARD, 1000);
    inbox.postCommand(CENTRED, 1100);
    inbox.postCommand(HANDHELD_CENTRED, 1000 + WINDOW_MS + 50);
    inbox.take(1000 + WINDOW_MS + 50);
    inbox.postCommand(FORWARD_STICK, 1000 + WINDOW_MS + 100);
    TEST_ASSERT_FALSE(inbox.take(1000 + WINDOW_MS + 100).command.has_value());
    inbox.postCommand(CENTRED, 2000);
    inbox.postCommand(HANDHELD_FORWARD, 2100);
    inbox.take(2100);
    inbox.postCommand(FORWARD_STICK, 2100 + WINDOW_MS);
    TEST_ASSERT_FALSE(inbox.take(2100 + WINDOW_MS).command.has_value());
}

// Each take says whether the handheld drives, from the take after its first moving frame to the first take past the
// window, and keeps it for the replies.
void test_the_take_says_whether_the_handheld_drives() {
    MotionInbox inbox;
    TEST_ASSERT_FALSE(inbox.take(1000).handheldDriving);
    inbox.postCommand(HANDHELD_FORWARD, 1000);
    TEST_ASSERT_FALSE(inbox.handheldDriving());
    TEST_ASSERT_TRUE(inbox.take(1000).handheldDriving);
    TEST_ASSERT_TRUE(inbox.handheldDriving());
    TEST_ASSERT_TRUE(inbox.take(1000 + WINDOW_MS - 1).handheldDriving);
    TEST_ASSERT_FALSE(inbox.take(1000 + WINDOW_MS).handheldDriving);
    TEST_ASSERT_FALSE(inbox.handheldDriving());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_goal_is_taken_once_and_reported_settling_until_then);
    RUN_TEST(test_a_newer_goal_replaces_the_one_not_yet_taken);
    RUN_TEST(test_a_cancel_ends_only_the_goal_driving_when_it_arrives);
    RUN_TEST(test_an_odometry_reset_is_refused_while_a_goal_drives_and_accepted_once_it_ends);
    RUN_TEST(test_an_owner_silent_past_the_limit_loses_its_goal_at_the_take);
    RUN_TEST(test_an_owner_heard_after_the_takes_clock_read_keeps_its_goal);
    RUN_TEST(test_a_goal_without_an_owner_never_falls_silent);
    RUN_TEST(test_the_owner_leaving_ends_its_goal_and_another_client_leaving_does_not);
    RUN_TEST(test_a_command_is_taken_once_with_its_arrival_and_the_newest_wins);
    RUN_TEST(test_while_a_goal_drives_a_centred_command_is_swallowed_and_a_stick_takes_the_walk_back);
    RUN_TEST(test_a_command_posted_before_the_goal_leaves_it_alone);
    RUN_TEST(test_a_gait_is_taken_once_and_the_newest_wins);
    RUN_TEST(test_a_gait_other_than_the_selection_ends_the_goal);
    RUN_TEST(test_a_goals_own_gait_is_the_selection);
    RUN_TEST(test_a_gait_posted_before_the_goals_selection_stays_the_selection);
    RUN_TEST(test_a_pending_gait_other_than_the_goals_own_ends_the_goal);
    RUN_TEST(test_the_robot_animates_from_a_borrow_queued_until_the_mode_is_handed_back);
    RUN_TEST(test_modes_are_taken_in_arrival_order_none_merged);
    RUN_TEST(test_a_full_mode_queue_drops_the_oldest_and_keeps_a_newest_deactivation);
    RUN_TEST(test_the_replies_judge_the_mode_the_control_task_noted_last);
    RUN_TEST(test_a_borrow_ends_a_goal_only_from_a_mode_it_would_animate);
    RUN_TEST(test_a_mode_leaving_walk_ends_only_a_goal_posted_before_it);
    RUN_TEST(test_the_replies_carry_the_pose_the_control_task_noted_last);
    RUN_TEST(test_a_clients_command_within_the_handheld_window_is_dropped);
    RUN_TEST(test_the_handheld_window_ends_600_ms_after_its_last_moving_frame);
    RUN_TEST(test_the_handhelds_centred_frame_is_taken_and_opens_no_window);
    RUN_TEST(test_a_robot_that_never_saw_the_handheld_move_takes_every_command);
    RUN_TEST(test_a_goal_within_the_handheld_window_is_refused);
    RUN_TEST(test_a_handheld_moving_frame_ends_a_goal_though_its_walk_sticks_are_centred);
    RUN_TEST(test_an_idle_handhelds_frame_outside_its_window_keeps_the_walk);
    RUN_TEST(test_the_handheld_window_spans_the_clocks_wrap);
    RUN_TEST(test_after_the_handheld_window_a_clients_stick_waits_for_a_centred_command);
    RUN_TEST(test_only_a_clients_centred_command_after_the_window_ends_the_wait);
    RUN_TEST(test_the_take_says_whether_the_handheld_drives);
    return UNITY_END();
}
