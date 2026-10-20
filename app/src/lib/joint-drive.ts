import { derived, type Readable } from 'svelte/store'
import { MotionModes, urdfJointOrder } from '$lib/motion'
import { JointAngleData } from '$lib/platform_shared/message'
import { mode } from '$lib/robot/state'
import { drivesSimulation } from '$lib/stores/drive'
import { isLinked } from '$lib/stores/link'
import { dataBroker } from '$lib/transport/databroker'

const URDF_SERVOS = urdfJointOrder(Array.from({ length: 18 }, (_, servo) => servo))

/** The firmware servo whose angle the 3D view draws on the URDF model's joint `index`: urdfJointOrder's inverse. */
export const servoOfUrdfJoint = (index: number) => URDF_SERVOS[index]

/**
 * Each joint's travel in degrees, coxa, femur and tibia: a copy of the firmware's kinematics.h JOINT_LIMIT_DEG, which
 * tests/unit/joint-drive.spec.ts keeps equal to it. The URDF model's own limits are wider, the femur's to -152.8.
 */
export const JOINT_LIMIT_DEG = [31.5, 90, 149] as const

/** The URDF joint `index` at `radians`, held within the travel of the servo it drives, as the firmware holds it. */
export const withinTravel = (index: number, radians: number) => {
  const limit = (JOINT_LIMIT_DEG[servoOfUrdfJoint(index) % 3] * Math.PI) / 180
  return Math.min(limit, Math.max(-limit, radians))
}

/**
 * Whether dragging a 3D view's joints drives the robot's servos: from the controller page's view alone, in IDLE, over
 * a link, and not while the controls drive the simulation view's robot. Every other 3D view poses the drawn model only.
 */
export const drivesJoints = ({
  controllerView,
  mode,
  linked,
  simulation
}: {
  controllerView: boolean
  mode: MotionModes | null
  linked: boolean
  simulation: boolean
}) => controllerView && mode === MotionModes.IDLE && linked && !simulation

/** Whether the controller page drives the robot's joints now, which its badge shows and its 3D views follow. */
export const drivingJoints: Readable<boolean> = derived(
  [mode, isLinked, drivesSimulation],
  ([current, linked, simulation]) =>
    drivesJoints({ controllerView: true, mode: current, linked, simulation })
)

/**
 * A joint as the robot takes it: the newest angle of each servo goes out, and while a write is under way only that
 * waits (DataBroker.emitVolatile), so a drag sends no backlog and its last angle always arrives while the link is up.
 */
export const sendJoint = (joint: JointAngleData) =>
  dataBroker.emitVolatile(JointAngleData, joint, joint.servo)

/** Sends the joints a 3D view's drag moves, for as long as `driving` holds and no longer. */
export class JointDrive {
  private driving = false
  private readonly unsubscribe: () => void

  constructor(
    driving: Readable<boolean>,
    private readonly send: (joint: JointAngleData) => boolean = sendJoint
  ) {
    this.unsubscribe = driving.subscribe(value => (this.driving = value))
  }

  get active() {
    return this.driving
  }

  /** The URDF joint `index` dragged to `radians`; false, and nothing sent, while not driving. */
  drag(index: number, radians: number): boolean {
    if (!this.driving) return false
    return this.send({ servo: servoOfUrdfJoint(index), degrees: (radians * 180) / Math.PI })
  }

  dispose() {
    this.unsubscribe()
    this.driving = false
  }
}
