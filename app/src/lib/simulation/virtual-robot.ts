// The firmware's Hexapod compiled to WebAssembly (firmware/host/virtual_robot.h, built by
// scripts/build_virtual_robot.js): the robot's own motion code, run on the simulation's clock.

/** One pass of the firmware's control loop. */
export const TICK_SECONDS = 0.005

/** firmware/include/imu_frame.h: the MPU6050 DMP's roll, pitch and yaw against the body frame. */
export const DMP_ANGLE_SIGNS = [1, -1, -1] as const

/** The IMU as Peripherals reports it: roll, pitch and yaw in radians, gravity up and rates in the body frame. */
export interface SimImu {
  rpyRad: [number, number, number]
  gravityUp: [number, number, number]
  gyroRad: [number, number, number]
}

/** The fields of a ControllerInputData message, in CommandMsg order: lx, ly, rx, ry, h, s, s1, fd, stride. */
export type Command = [number, number, number, number, number, number, number, number, number]

export interface VirtualRobot {
  setMode(mode: number): void
  setGait(gait: number): void
  command(command: Command): void
  setImu(imu: SimImu): void
  tick(): void
  /** Whether the servos are driven: the boards are awake and a pose has been sent since boot. */
  servosDriven(): boolean
  /** The joint angles the servos are driven to in degrees, coxa, femur and tibia per leg in leg order. */
  servoAngles(): Float32Array
  /** The joint angles of the firmware's stand, for spawning; leaves the robot as new. */
  standPose(): Float32Array
}

const STAND = 3

export async function loadVirtualRobot(): Promise<VirtualRobot> {
  const { default: create } = await import('./virtual-robot/virtual_robot.mjs')
  const m = await create()
  m._reset()
  const servoAngles = () => m.HEAPF32.slice(m._servoAngles() / 4, m._servoAngles() / 4 + 18)
  return {
    setMode: mode => m._setMode(mode),
    setGait: gait => m._setGait(gait),
    command: c => m._command(...c),
    setImu: ({ rpyRad, gravityUp, gyroRad }) => m._setImu(...rpyRad, ...gravityUp, ...gyroRad),
    tick: () => m._tick(),
    servosDriven: () => m._servosDriven() !== 0,
    servoAngles,
    standPose: () => {
      m._reset()
      m._setMode(STAND)
      m._tick()
      const pose = servoAngles()
      m._reset()
      return pose
    }
  }
}
