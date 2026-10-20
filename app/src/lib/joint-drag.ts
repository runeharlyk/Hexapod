import { Plane, Raycaster, Vector2, Vector3, type Camera, type Object3D } from 'three'
import type { URDFJoint, URDFRobot } from 'urdf-loader'

const isMovingJoint = (o: Object3D): o is URDFJoint =>
  (o as URDFJoint).isURDFJoint === true && (o as URDFJoint).jointType === 'revolute'

/** The revolute joint that moves the part of the model `o` belongs to: the nearest one above it. */
const jointMoving = (o: Object3D | null): URDFJoint | null => {
  for (let at = o; at; at = at.parent) if (isMovingJoint(at)) return at
  return null
}

interface Grab {
  joint: URDFJoint
  axis: Vector3
  pivot: Vector3
  plane: Plane
  last: Vector3
}

/**
 * Turns the model's revolute joints with the pointer: a press on a leg takes the joint that moves the part pressed, and
 * the pointer turns it about its axis, as the angle swept around the joint's pivot in the plane of its rotation. The
 * press is taken before the orbit controls hear it, so the camera holds still while a joint turns. A part that a joint
 * can turn shows the grab cursor.
 */
export class JointDrag {
  private readonly raycaster = new Raycaster()
  private readonly pointer = new Vector2()
  private readonly point = new Vector3()
  private grab: Grab | undefined

  constructor(
    private readonly model: URDFRobot,
    private readonly camera: Camera,
    private readonly element: HTMLElement,
    private readonly onTurn: (joint: URDFJoint, angle: number) => void
  ) {
    element.addEventListener('pointerdown', this.press, { capture: true })
    element.addEventListener('pointermove', this.move)
    element.addEventListener('pointerup', this.release)
    element.addEventListener('pointercancel', this.release)
  }

  dispose() {
    this.element.removeEventListener('pointerdown', this.press, { capture: true })
    this.element.removeEventListener('pointermove', this.move)
    this.element.removeEventListener('pointerup', this.release)
    this.element.removeEventListener('pointercancel', this.release)
    this.grab = undefined
    this.element.style.cursor = ''
  }

  private aim(event: PointerEvent) {
    const rect = this.element.getBoundingClientRect()
    this.pointer.set(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1
    )
    this.raycaster.setFromCamera(this.pointer, this.camera)
  }

  private jointUnder(event: PointerEvent): URDFJoint | null {
    this.aim(event)
    const [hit] = this.raycaster.intersectObject(this.model, true)
    return jointMoving(hit?.object ?? null)
  }

  private press = (event: PointerEvent) => {
    if (event.button !== 0) return
    const joint = this.jointUnder(event)
    if (!joint) return
    event.stopImmediatePropagation()
    const axis = joint.axis.clone().transformDirection(joint.matrixWorld).normalize()
    const pivot = new Vector3().setFromMatrixPosition(joint.matrixWorld)
    const plane = new Plane().setFromNormalAndCoplanarPoint(axis, pivot)
    const last = this.raycaster.ray.intersectPlane(plane, new Vector3())
    if (!last) return
    this.grab = { joint, axis, pivot, plane, last }
    this.element.setPointerCapture(event.pointerId)
    this.element.style.cursor = 'grabbing'
  }

  private move = (event: PointerEvent) => {
    const grab = this.grab
    if (!grab) {
      this.element.style.cursor = this.jointUnder(event) ? 'grab' : ''
      return
    }
    this.aim(event)
    const point = this.raycaster.ray.intersectPlane(grab.plane, this.point)
    if (!point) return
    const from = grab.last.clone().sub(grab.pivot)
    const to = point.clone().sub(grab.pivot)
    const swept = Math.atan2(from.clone().cross(to).dot(grab.axis), from.dot(to))
    grab.last.copy(point)
    this.onTurn(grab.joint, grab.joint.angle + swept)
  }

  private release = (event: PointerEvent) => {
    if (!this.grab) return
    this.grab = undefined
    if (this.element.hasPointerCapture(event.pointerId))
      this.element.releasePointerCapture(event.pointerId)
    this.element.style.cursor = ''
  }
}
