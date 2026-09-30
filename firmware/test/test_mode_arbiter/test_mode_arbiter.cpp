// Host tests for the mode decision MotionService runs on the ModeMsg worker: each test replays one
// interleaving of explicit modes, animation borrows and hand-backs as the worker would receive them.

#include <unity.h>

#include <initializer_list>
#include <animation/mode_arbiter.h>

namespace {

using D = ModeArbiterDecision;
using M = MOTION_STATE;
using K = ModeMsgKind;

// The worker's view of MotionService, updated exactly as handleInputMode executes a decision.
struct Robot {
    M mode;
    bool borrowed = false;
    M previous = M::STAND;
};

D deliver(Robot &r, M requested, K kind, bool playerBusy) {
    const D d = decideMode({r.mode, r.borrowed, r.previous, requested, kind, playerBusy});
    r.borrowed = d.borrowed;
    r.previous = d.previous;
    if (d.action == D::APPLY) r.mode = d.mode;
    return d;
}

D explicitMode(Robot &r, M mode, bool playerBusy) { return deliver(r, mode, K::REQUEST, playerBusy); }
D borrow(Robot &r, bool playerBusy) { return deliver(r, M::ANIMATE, K::BORROW, playerBusy); }
D handback(Robot &r) { return deliver(r, r.previous, K::HANDBACK, false); }
D applied(Robot &r, M mode) { return deliver(r, mode, K::APPLIED, false); }

void assertMode(M expected, M actual) { TEST_ASSERT_EQUAL_INT((int)expected, (int)actual); }

} // namespace

void setUp() {}
void tearDown() {}

void test_two_quick_plays_from_stand_keep_stand_as_the_hand_back() {
    Robot r {M::STAND};
    D d = borrow(r, true);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    assertMode(M::ANIMATE, r.mode);
    TEST_ASSERT_TRUE(r.borrowed);
    assertMode(M::STAND, r.previous);

    // The second play saw STAND on the adapter task before the first borrow landed.
    d = borrow(r, true);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    TEST_ASSERT_FALSE(d.cancelPending);
    assertMode(M::ANIMATE, r.mode);
    TEST_ASSERT_TRUE(r.borrowed);
    assertMode(M::STAND, r.previous);

    d = handback(r);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    assertMode(M::STAND, r.mode);
    TEST_ASSERT_FALSE(r.borrowed);
}

void test_an_explicit_walk_queued_ahead_of_the_borrow_becomes_the_hand_back() {
    Robot r {M::STAND};
    D d = explicitMode(r, M::WALK, true);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    TEST_ASSERT_TRUE(d.cancelPending);
    assertMode(M::WALK, r.mode);

    d = borrow(r, false);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    assertMode(M::ANIMATE, r.mode);
    assertMode(M::WALK, r.previous);

    handback(r);
    assertMode(M::WALK, r.mode);
    TEST_ASSERT_FALSE(r.borrowed);
}

void test_deactivated_during_the_load_refuses_the_borrow_and_restates() {
    Robot r {M::STAND};
    D d = explicitMode(r, M::DEACTIVATED, true);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    TEST_ASSERT_TRUE(d.cancelPending);

    d = borrow(r, true);
    TEST_ASSERT_EQUAL_INT(D::RESTATE, d.action);
    assertMode(M::DEACTIVATED, d.mode);
    TEST_ASSERT_TRUE(d.cancelPending);
    TEST_ASSERT_FALSE(d.requestStop);
    assertMode(M::DEACTIVATED, r.mode);
    TEST_ASSERT_FALSE(r.borrowed);
}

void test_a_hand_back_while_sticky_is_ignored() {
    Robot r {M::STAND};
    explicitMode(r, M::ANIMATE, false);
    TEST_ASSERT_FALSE(r.borrowed);

    const D d = handback(r);
    TEST_ASSERT_EQUAL_INT(D::IGNORE, d.action);
    assertMode(M::ANIMATE, r.mode);
    TEST_ASSERT_FALSE(d.cancelPending);
    TEST_ASSERT_FALSE(d.requestStop);
}

void test_a_hand_back_after_an_explicit_stand_left_animate_is_ignored() {
    Robot r {M::WALK};
    borrow(r, true);
    // The play has finished and the hand-back is queued when an explicit STAND arrives first.
    D d = explicitMode(r, M::STAND, false);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    assertMode(M::STAND, r.mode);
    TEST_ASSERT_FALSE(r.borrowed);

    d = handback(r);
    TEST_ASSERT_EQUAL_INT(D::IGNORE, d.action);
    assertMode(M::STAND, r.mode);
}

void test_an_explicit_stand_mid_play_leaves_through_exit() {
    Robot r {M::WALK};
    borrow(r, true);
    D d = explicitMode(r, M::STAND, true);
    TEST_ASSERT_EQUAL_INT(D::GRACEFUL_LEAVE, d.action);
    TEST_ASSERT_TRUE(d.requestStop);
    TEST_ASSERT_FALSE(d.cancelPending);
    assertMode(M::ANIMATE, r.mode);
    TEST_ASSERT_TRUE(r.borrowed);
    assertMode(M::STAND, r.previous);

    // Exit has eased the legs home; the hand-back delivers the mode that was asked for.
    d = handback(r);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    assertMode(M::STAND, r.mode);
    TEST_ASSERT_FALSE(r.borrowed);

    // A sticky ANIMATE mid-play leaves the same way.
    Robot sticky {M::ANIMATE};
    d = explicitMode(sticky, M::WALK_NN, true);
    TEST_ASSERT_EQUAL_INT(D::GRACEFUL_LEAVE, d.action);
    TEST_ASSERT_TRUE(sticky.borrowed);
    assertMode(M::WALK_NN, sticky.previous);
}

void test_cutting_modes_mid_play_apply_at_once() {
    for (M cut : {M::DEACTIVATED, M::IDLE, M::POSE}) {
        Robot r {M::STAND};
        borrow(r, true);
        const D d = explicitMode(r, cut, true);
        TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
        TEST_ASSERT_TRUE(d.cancelPending);
        TEST_ASSERT_FALSE(d.requestStop);
        assertMode(cut, r.mode);
        TEST_ASSERT_FALSE(r.borrowed);
        TEST_ASSERT_EQUAL_INT(D::IGNORE, handback(r).action);
        assertMode(cut, r.mode);
    }
}

void test_an_explicit_animate_while_borrowed_becomes_sticky() {
    Robot r {M::STAND};
    borrow(r, true);
    D d = explicitMode(r, M::ANIMATE, true);
    TEST_ASSERT_EQUAL_INT(D::APPLY, d.action);
    TEST_ASSERT_FALSE(d.cancelPending);
    TEST_ASSERT_FALSE(d.requestStop);
    assertMode(M::ANIMATE, r.mode);
    TEST_ASSERT_FALSE(r.borrowed);

    d = handback(r);
    TEST_ASSERT_EQUAL_INT(D::IGNORE, d.action);
    assertMode(M::ANIMATE, r.mode);
}

void test_an_applied_report_is_never_taken_as_a_request() {
    // MotionService reports the mode a borrow or hand-back produced as APPLIED on the same bus; were
    // it decided, an APPLIED STAND trailing a hand-back would end a later sticky ANIMATE.
    Robot r {M::STAND};
    borrow(r, true);
    D d = applied(r, M::ANIMATE);
    TEST_ASSERT_EQUAL_INT(D::IGNORE, d.action);
    TEST_ASSERT_FALSE(d.cancelPending);
    TEST_ASSERT_FALSE(d.requestStop);
    TEST_ASSERT_TRUE(r.borrowed);
    assertMode(M::STAND, r.previous);

    explicitMode(r, M::ANIMATE, false);
    d = applied(r, M::STAND);
    TEST_ASSERT_EQUAL_INT(D::IGNORE, d.action);
    assertMode(M::ANIMATE, r.mode);
    TEST_ASSERT_FALSE(r.borrowed);

    Robot off {M::DEACTIVATED};
    TEST_ASSERT_EQUAL_INT(D::IGNORE, applied(off, M::STAND).action);
    assertMode(M::DEACTIVATED, off.mode);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_two_quick_plays_from_stand_keep_stand_as_the_hand_back);
    RUN_TEST(test_an_explicit_walk_queued_ahead_of_the_borrow_becomes_the_hand_back);
    RUN_TEST(test_deactivated_during_the_load_refuses_the_borrow_and_restates);
    RUN_TEST(test_a_hand_back_while_sticky_is_ignored);
    RUN_TEST(test_a_hand_back_after_an_explicit_stand_left_animate_is_ignored);
    RUN_TEST(test_an_explicit_stand_mid_play_leaves_through_exit);
    RUN_TEST(test_cutting_modes_mid_play_apply_at_once);
    RUN_TEST(test_an_explicit_animate_while_borrowed_becomes_sticky);
    RUN_TEST(test_an_applied_report_is_never_taken_as_a_request);
    return UNITY_END();
}
