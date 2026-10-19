import { Matrix4, Quaternion, Vector3 } from 'three'

/**
 * The URDF model's frame (static/model.xacro) is the body frame of docs/conventions.md turned half a turn about z: it
 * puts its front right leg at (-x, -y), where the body frame has the right front leg at (+x, +y).
 */
export const URDF_TO_BODY = new Quaternion().setFromAxisAngle(new Vector3(0, 0, 1), Math.PI)

// The 3D view's scene is y up, and a body-frame point (x, y, z) is drawn at scene (y, z, x), as stanceToScene
// (animation/handles.ts) places it.
// prettier-ignore
const BODY_TO_SCENE = new Quaternion().setFromRotationMatrix(new Matrix4().set(
  0, 1, 0, 0,
  0, 0, 1, 0,
  1, 0, 0, 0,
  0, 0, 0, 1
))

/** The URDF model's orientation in the 3D view's scene for a level, unturned body. */
export const LEVEL = BODY_TO_SCENE.clone().multiply(URDF_TO_BODY)
