import { MotionModes } from '$lib/motion'

/**
 * The controls the controller page shows beside the mode buttons, which every mode shows:
 * - posture: body height and feet distance;
 * - sticks: the two sticks and their keys, a body shift and tilt in STAND and the walk in the walking modes;
 * - gait: the gait choice;
 * - walk: speed, step height and stride;
 * - joints: driving the servos by dragging the 3D model's joints, and the servo page;
 * - animation: playing and stopping the robot's animations.
 */
export interface ModeControls {
  posture: boolean
  sticks: boolean
  gait: boolean
  walk: boolean
  joints: boolean
  animation: boolean
}

const NONE: ModeControls = {
  posture: false,
  sticks: false,
  gait: false,
  walk: false,
  joints: false,
  animation: false
}

const STANDING: ModeControls = { ...NONE, posture: true, sticks: true }

// Each mode shows only what it reads. WALK_NN's policy chooses its own gait.
const CONTROLS: Record<MotionModes, ModeControls> = {
  [MotionModes.DEACTIVATED]: NONE,
  [MotionModes.IDLE]: { ...NONE, joints: true },
  [MotionModes.STAND]: STANDING,
  [MotionModes.WALK]: { ...STANDING, gait: true, walk: true },
  [MotionModes.WALK_NN]: { ...STANDING, walk: true },
  [MotionModes.ANIMATE]: { ...NONE, animation: true }
}

export const controlsFor = (mode: MotionModes): ModeControls => CONTROLS[mode]
