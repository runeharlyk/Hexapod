// The hexapod in MuJoCo, driven by the firmware's own motion code (virtual-robot.ts): every 5 ms control tick
// the robot reads the simulated IMU and sets its joint targets, and the servo model (servo.ts) turns those into
// torque at the physics rate. The scene is simulation/src/resources/model.xml, the model the training uses,
// and the stepping, IMU and servo handling mirror HexapodSim and FirmwareLoop (simulation/src/sim/).
import type { MainModule, MjData, MjModel } from '@mujoco/mujoco'
import { GEAR_FRICTION, servoTorque } from './servo'
import { TICK_SECONDS, type SimImu, type VirtualRobot } from './virtual-robot'

/** MuJoCo's joint and actuator names, in the firmware's angle order. */
export const JOINTS = [0, 1, 2, 3, 4, 5].flatMap(leg =>
  ['coxa', 'femur', 'tibia'].map(j => `${j}_${leg}`)
)

/** The most simulated time one call may advance, so a tab returning from the background resumes instead of replaying. */
export const MAX_FRAME_SECONDS = 0.05
const FALLEN_TILT_COS = Math.cos((60 * Math.PI) / 180)
const DEG = Math.PI / 180

export class HexapodSim {
  private readonly model: MjModel
  private readonly data: MjData
  private readonly actuators: number[]
  private readonly qposAdr: number[]
  private readonly dofAdr: number[]
  private readonly baseFriction: number[]
  private readonly gyroAdr: number
  private readonly targets = new Float64Array(18)
  private ticks = 0
  private steps = 0
  private pending = 0

  constructor(
    private readonly mujoco: MainModule,
    sceneXml: string,
    private readonly robot: VirtualRobot
  ) {
    const { mjtObj } = mujoco
    mujoco.FS.writeFile('/hexapod.xml', sceneXml)
    this.model = mujoco.MjModel.from_xml_path('/hexapod.xml')
    this.data = new mujoco.MjData(this.model)
    const id = (kind: number, name: string) => {
      const found = mujoco.mj_name2id(this.model, kind, name)
      if (found < 0) throw new Error(`The hexapod model has no ${name}`)
      return found
    }
    this.actuators = JOINTS.map(name => id(mjtObj.mjOBJ_ACTUATOR.value, name))
    const joints = JOINTS.map(name => id(mjtObj.mjOBJ_JOINT.value, name))
    this.qposAdr = joints.map(j => this.model.jnt_qposadr[j])
    this.dofAdr = joints.map(j => this.model.jnt_dofadr[j])
    this.baseFriction = this.dofAdr.map(dof => this.model.dof_frictionloss[dof])
    this.gyroAdr = this.model.sensor_adr[id(mjtObj.mjOBJ_SENSOR.value, 'imu_gyro')]
    // As HexapodSim: the gear friction's constraint as stiff as MuJoCo allows, so a held joint neither creeps
    // nor overshoots.
    for (const dof of this.dofAdr) {
      this.model.dof_solref[dof * 2] = 2 * this.model.opt.timestep
      this.model.dof_solimp[dof * 5] = 0.999
      this.model.dof_solimp[dof * 5 + 1] = 0.9999
    }
    this.reset()
  }

  /** Spawns the robot unpowered in the firmware's stand pose, its lowest point on the floor. */
  reset() {
    const { mujoco, model, data } = this
    mujoco.mj_resetData(model, data)
    const stand = this.robot.standPose()
    this.qposAdr.forEach((adr, i) => {
      data.qpos[adr] = stand[i] * DEG
      this.targets[i] = stand[i] * DEG
    })
    mujoco.mj_forward(model, data)
    data.qpos[2] -= this.lowestFoot()
    mujoco.mj_forward(model, data)
    this.ticks = this.steps = this.pending = 0
  }

  /** Advances by whole control ticks, carrying the remainder to the next call. */
  step(seconds: number) {
    this.pending += Math.min(seconds, MAX_FRAME_SECONDS)
    for (; this.pending >= TICK_SECONDS; this.pending -= TICK_SECONDS) this.tick()
  }

  /** One firmware tick, then the physics steps up to its end: 2 ms steps do not divide 5 ms ticks. */
  tick() {
    this.robot.setImu(this.imu())
    this.robot.tick()
    const powered = this.robot.servosDriven()
    const angles = this.robot.servoAngles()
    for (let i = 0; i < 18; i++) this.targets[i] = angles[i] * DEG
    this.ticks++
    const due = Math.round((this.ticks * TICK_SECONDS) / this.model.opt.timestep)
    for (; this.steps < due; this.steps++) {
      this.applyServos(powered)
      this.mujoco.mj_step(this.model, this.data)
    }
  }

  /** World position of the base in metres, z up. */
  basePosition(): [number, number, number] {
    return [this.data.qpos[0], this.data.qpos[1], this.data.qpos[2]]
  }

  baseQuaternion(): [w: number, x: number, y: number, z: number] {
    return [this.data.qpos[3], this.data.qpos[4], this.data.qpos[5], this.data.qpos[6]]
  }

  /** The joints' angles in radians, in the firmware's angle order. */
  jointAngles(): number[] {
    return this.qposAdr.map(adr => this.data.qpos[adr])
  }

  /** Tilted beyond 60 degrees; a robot lying flat after its servos went limp has not fallen. */
  hasFallen(): boolean {
    const [, x, y] = this.baseQuaternion()
    return 1 - 2 * (x * x + y * y) < FALLEN_TILT_COS
  }

  dispose() {
    this.data.delete()
    this.model.delete()
  }

  // An unpowered servo leaves only its gear's base friction.
  private applyServos(powered: boolean) {
    const { model, data } = this
    for (let i = 0; i < 18; i++) {
      const dof = this.dofAdr[i]
      model.dof_frictionloss[dof] = this.baseFriction[i] + (powered ? GEAR_FRICTION : 0)
      data.ctrl[this.actuators[i]] =
        powered ? servoTorque(this.targets[i], data.qpos[this.qposAdr[i]], data.qvel[dof]) : 0
    }
  }

  // The IMU as the training environment derives it from the base: roll, pitch and yaw of the base quaternion,
  // world up in the body frame, and the gyro sensor at the IMU site.
  private imu(): SimImu {
    const [w, x, y, z] = this.baseQuaternion()
    const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))))
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    const gyro = this.data.sensordata
    return {
      rpyDeg: [roll / DEG, pitch / DEG, yaw / DEG],
      gravityUp: [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
      gyroRad: [gyro[this.gyroAdr], gyro[this.gyroAdr + 1], gyro[this.gyroAdr + 2]]
    }
  }

  // The foot spheres are the robot's lowest points in the stand pose.
  private lowestFoot(): number {
    let lowest = Infinity
    for (let leg = 0; leg < 6; leg++) {
      const geom = this.mujoco.mj_name2id(
        this.model,
        this.mujoco.mjtObj.mjOBJ_GEOM.value,
        `foot_${leg}`
      )
      lowest = Math.min(lowest, this.data.geom_xpos[geom * 3 + 2] - this.model.geom_size[geom * 3])
    }
    return lowest
  }
}
