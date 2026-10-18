import { Quaternion, Vector3 } from 'three'
import type { Ground } from '$lib/kinematic'
import { stanceToScene } from '$lib/animation/handles'
import { LEVEL } from '$lib/model-frame'
import { DMP_ANGLE_SIGNS } from '$lib/simulation/virtual-robot'

const UP = new Vector3(0, 1, 0)

// stanceToScene's axes, for a direction rather than a point
const directionToScene = ([x, y, z]: readonly number[]) => new Vector3(y, z, x)

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
    tilt.setFromUnitVectors(directionToScene(ground.normal), UP)
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

/** IMUData: the MPU6050 DMP's roll, pitch and yaw in radians, with the DMP's signs (firmware/include/imu_frame.h). */
export interface DmpAngles {
  x: number
  y: number
  z: number
}

/**
 * World up in the body frame for the DMP's roll and pitch. The DMP takes roll as atan2(gy, gz) and pitch as
 * atan2(gx, hypot(gy, gz)) of this vector, so it is rebuilt exactly up to a 90 degree tilt; imu_frame.h's signs turn
 * the angles into the body frame's rotations about +x (nose up) and +y (right side down) first.
 */
const worldUp = ({ x, y }: DmpAngles): number[] => {
  const noseUp = DMP_ANGLE_SIGNS[0] * x
  const rightSideDown = DMP_ANGLE_SIGNS[1] * y
  return [
    -Math.sin(rightSideDown),
    Math.cos(rightSideDown) * Math.sin(noseUp),
    Math.cos(rightSideDown) * Math.cos(noseUp)
  ]
}

/**
 * Where the connected robot stands for its IMU: tilted as the DMP's roll and pitch measure it, with no yaw, so it keeps
 * facing the camera, and lowered until the lowest of `feet` (body frame, mm, as the reported joint angles place them)
 * touches the floor (y = 0), its body origin kept over the scene's origin. Without an IMU reading it stands level.
 */
export const imuPlacement = (
  imu: DmpAngles | null,
  feet: readonly (readonly number[])[]
): Placement => {
  const tilt = new Quaternion()
  if (imu) tilt.setFromUnitVectors(directionToScene(worldUp(imu)), UP)
  const origin = new Vector3(...stanceToScene([0, 0, 0]))
  const turned = origin.clone().applyQuaternion(tilt)
  const lowest = Math.min(
    ...feet.map(foot => new Vector3(...stanceToScene(foot)).applyQuaternion(tilt).y)
  )
  return {
    orientation: tilt.multiply(LEVEL),
    position: new Vector3(origin.x - turned.x, -lowest, origin.z - turned.z)
  }
}
