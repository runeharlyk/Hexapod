import { Euler, Quaternion, Vector3 } from 'three'
import type { Ground } from '$lib/kinematic'
import { stanceToScene } from '$lib/animation/handles'

// The URDF's orientation for a level, unturned body; the scene is y up (see handles.ts for the axes).
const LEVEL = new Quaternion().setFromEuler(new Euler(-Math.PI / 2, 0, Math.PI / 2))
const UP = new Vector3(0, 1, 0)

export interface Placement {
  orientation: Quaternion
  position: Vector3
}

/**
 * Where the robot model stands for the ground under its feet (Kinematics.groundUnderFeet): the plane its planted feet
 * span turned level and lowered onto the floor (y = 0) about their centre, so they stay where they stand whatever the
 * body does above them. `yaw` (radians about the scene's up) turns the whole robot. Without a ground the robot stands
 * level on its stance.
 */
export const groundPlacement = (ground: Ground | null, yaw: number): Placement => {
  const tilt = new Quaternion()
  const shift = new Vector3()
  if (ground) {
    const [nx, ny, nz] = ground.normal
    // stanceToScene's axes, for a direction rather than a point
    tilt.setFromUnitVectors(new Vector3(ny, nz, nx), UP)
    const centre = ground.planted
      .reduce((sum, foot) => sum.add(new Vector3(...stanceToScene(foot))), new Vector3())
      .divideScalar(ground.planted.length)
    const turned = centre.clone().applyQuaternion(tilt)
    shift.set(centre.x - turned.x, -turned.y, centre.z - turned.z)
  }
  const turn = new Quaternion().setFromAxisAngle(UP, yaw)
  return {
    orientation: turn.clone().multiply(tilt).multiply(LEVEL),
    position: shift.applyQuaternion(turn)
  }
}
