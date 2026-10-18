import type { Object3D } from 'three'
import type { URDFRobot } from 'urdf-loader'
import { URDF_TO_BODY } from '$lib/model-frame'

/** The simulation view's URDF model is drawn at 120 mm per URDF unit. */
export const METRES_PER_URDF_UNIT = 0.12

/**
 * Hangs the URDF model in `body`, a group that carries MuJoCo's base pose in metres, so the model is drawn in the
 * MuJoCo body frame, which is the body frame of docs/conventions.md.
 */
export function mountModel(urdf: URDFRobot, body: Object3D) {
  urdf.position.set(0, 0, 0)
  urdf.quaternion.copy(URDF_TO_BODY)
  urdf.scale.setScalar(METRES_PER_URDF_UNIT)
  body.add(urdf)
}
