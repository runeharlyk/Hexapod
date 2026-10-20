import { MotionModes } from '$lib/motion'

/**
 * The controls the controller page shows beside the mode buttons, which every mode shows:
 * - height: the body height;
 * - feetDistance: the feet distance;
 * - sticks: the two sticks and their keys, a body shift and tilt in STAND and the walk in the walking modes;
 * - gait: the gait choice;
 * - walk: speed, step height and stride;
 * - joints: driving the servos by dragging the 3D model's joints, and the servo page;
 * - animation: playing and stopping the robot's animations.
 */
export interface ModeControls {
  height: boolean
  feetDistance: boolean
  sticks: boolean
  gait: boolean
  walk: boolean
  joints: boolean
  animation: boolean
}

const NONE: ModeControls = {
  height: false,
  feetDistance: false,
  sticks: false,
  gait: false,
  walk: false,
  joints: false,
  animation: false
}

const STANDING: ModeControls = { ...NONE, height: true, feetDistance: true, sticks: true }

// Each mode shows only what it reads. WALK_NN's policy rides at its own height, lift, stride and cadence, chooses its
// own gait and walks the stance it was trained on: it reads the sticks alone.
const CONTROLS: Record<MotionModes, ModeControls> = {
  [MotionModes.DEACTIVATED]: NONE,
  [MotionModes.IDLE]: { ...NONE, joints: true },
  [MotionModes.STAND]: STANDING,
  [MotionModes.WALK]: { ...STANDING, gait: true, walk: true },
  [MotionModes.WALK_NN]: { ...NONE, sticks: true },
  [MotionModes.ANIMATE]: { ...NONE, animation: true }
}

/** A mode this app does not know (null) shows nothing beside the mode buttons, since what it reads is unknown. */
export const controlsFor = (mode: MotionModes | null): ModeControls =>
  mode ? CONTROLS[mode] : NONE
