import { MotionModes } from '$lib/motion'

// Well inside the firmware's 600 ms steering timeout (firmware/include/motion.h,
// STEERING_TIMEOUT_MS): a held stick is re-sent so the robot can tell it from a lost link.
export const COMMAND_HEARTBEAT_MS = 200

const HEARTBEAT_MODES = new Set([MotionModes.STAND, MotionModes.WALK, MotionModes.WALK_NN])

// ANIMATE is absent on purpose: the firmware keeps an animation's window open by itself.
export const needsHeartbeat = (mode: MotionModes) => HEARTBEAT_MODES.has(mode)
