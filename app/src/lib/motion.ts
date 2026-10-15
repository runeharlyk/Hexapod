export enum MotionModes {
  DEACTIVATED = 'deactivated',
  IDLE = 'idle',
  POSE = 'pose',
  STAND = 'stand',
  WALK = 'walk',
  WALK_NN = 'walk nn',
  ANIMATE = 'animate'
}

/** The firmware's 18 joint angles, three per leg in leg order, in the order of the URDF model's joint names. */
export const urdfJointOrder = (a: number[]) =>
  [3, 0, 4, 1, 5, 2].flatMap(i => a.slice(i * 3, i * 3 + 3))
