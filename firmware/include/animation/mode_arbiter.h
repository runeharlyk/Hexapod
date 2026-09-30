#pragma once

// The MotionService mode decision as a pure function, so every interleaving of explicit modes,
// animation borrows and hand-backs is host-tested (firmware/test/test_mode_arbiter). It only decides;
// MotionService executes the decision on the ModeMsg worker, the one place the mode changes, so a
// hand-back can never overtake an emergency stop that was handled first.
//
// Rules:
// - An APPLIED message is MotionService reporting its own decision and is never a request: ignored.
// - A borrow (a play asking for ANIMATE) from a mode that is not actuated is refused: the loaded play
//   is cancelled and the standing mode is restated as APPLIED, so the client that asked for the play
//   learns the answer. A DEACTIVATED handled while the clip loaded therefore wins over the play.
// - A borrow from an actuated mode other than ANIMATE enters ANIMATE and records that mode for the
//   hand-back; a borrow while already in ANIMATE (a chained play) changes nothing.
// - A hand-back returns to the recorded mode only while ANIMATE is still borrowed; an explicit mode
//   handled since has ended the borrow, and the hand-back is ignored.
// - An explicit STAND or walking mode while ANIMATE is playing or has a play pending is a graceful
//   leave: the mode stays ANIMATE, the player is asked to stop and the requested mode becomes the
//   hand-back target, so Exit eases every leg home before the new mode drives it. Leaving at once
//   would snap joint-angle legs to stance at full servo speed.
// - DEACTIVATED, IDLE and POSE apply at once, even mid-play: they cut or centre the servos, and that
//   immediacy is the safety property.
// - Any other explicit mode applies and ends a borrow, so an explicit ANIMATE is sticky; an explicit
//   mode other than ANIMATE also drops a play that was loaded but not started.

#include <message_types.h>

inline bool actuated(MOTION_STATE mode) {
    return mode == MOTION_STATE::STAND || mode == MOTION_STATE::WALK || mode == MOTION_STATE::WALK_NN ||
           mode == MOTION_STATE::ANIMATE;
}

struct ModeArbiterInput {
    MOTION_STATE current;
    bool borrowed;
    MOTION_STATE previous;
    MOTION_STATE requested;
    ModeMsgKind kind;
    bool playerBusy; // the player is not idle, or a play is loaded and waiting
};

struct ModeArbiterDecision {
    // APPLY switches to mode; IGNORE and GRACEFUL_LEAVE keep the current mode; RESTATE keeps it and
    // publishes it again.
    enum Action { APPLY, IGNORE, GRACEFUL_LEAVE, RESTATE } action;
    MOTION_STATE mode;
    bool borrowed;
    MOTION_STATE previous;
    bool cancelPending;
    bool requestStop;
};

inline ModeArbiterDecision decideMode(const ModeArbiterInput &in) {
    using D = ModeArbiterDecision;
    D d {D::IGNORE, in.current, in.borrowed, in.previous, false, false};
    if (in.kind == ModeMsgKind::APPLIED) return d;
    if (in.kind == ModeMsgKind::BORROW) {
        if (!actuated(in.current)) {
            d.action = D::RESTATE;
            d.cancelPending = true;
            return d;
        }
        d.action = D::APPLY;
        d.mode = MOTION_STATE::ANIMATE;
        if (in.current != MOTION_STATE::ANIMATE) {
            d.borrowed = true;
            d.previous = in.current;
        }
        return d;
    }
    if (in.kind == ModeMsgKind::HANDBACK) {
        if (in.current == MOTION_STATE::ANIMATE && in.borrowed) {
            d.action = D::APPLY;
            d.mode = in.previous;
            d.borrowed = false;
        }
        return d;
    }
    if (in.current == MOTION_STATE::ANIMATE && in.playerBusy && actuated(in.requested) &&
        in.requested != MOTION_STATE::ANIMATE) {
        d.action = D::GRACEFUL_LEAVE;
        d.borrowed = true;
        d.previous = in.requested;
        d.requestStop = true;
        return d;
    }
    d.action = D::APPLY;
    d.mode = in.requested;
    d.borrowed = false;
    d.cancelPending = in.requested != MOTION_STATE::ANIMATE;
    return d;
}
