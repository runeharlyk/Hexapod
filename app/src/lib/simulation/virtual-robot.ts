// The firmware's Hexapod compiled to WebAssembly (firmware/host/virtual_robot.h, built by
// scripts/build_virtual_robot.js): the robot's own motion code, run on the simulation's clock.

/** One pass of the firmware's control loop. */
export const TICK_SECONDS = 0.005

/** The IMU as Peripherals reports it: DMP roll, pitch and yaw in degrees, gravity up and rates in the body frame. */
export interface SimImu {
  rpyDeg: [number, number, number]
  gravityUp: [number, number, number]
  gyroRad: [number, number, number]
}

/** The stick fields of a ControllerInputData message, in CommandMsg order: lx, ly, rx, ry, h, s, s1, fd. */
export type Command = [number, number, number, number, number, number, number, number]

export interface VirtualRobot {
  setMode(mode: number): void
  setGait(gait: number): void
  command(command: Command): void
  setImu(imu: SimImu): void
  tick(): void
  /** The last tick's joint angles in degrees, coxa, femur and tibia per leg in leg order. */
  angles(): Float32Array
  mode(): number
  /** The joint angles of the firmware's stand, for spawning; leaves the robot as new. */
  standPose(): Float32Array
}

const STAND = 3

export async function loadVirtualRobot(): Promise<VirtualRobot> {
  const { default: create } = await import('./virtual-robot/virtual_robot.mjs')
  const m = await create()
  m._reset()
  const angles = () => m.HEAPF32.slice(m._angles() / 4, m._angles() / 4 + 18)
  return {
    setMode: mode => m._setMode(mode),
    setGait: gait => m._setGait(gait),
    command: c => m._command(...c),
    setImu: ({ rpyDeg, gravityUp, gyroRad }) => m._setImu(...rpyDeg, ...gravityUp, ...gyroRad),
    tick: () => m._tick(),
    angles,
    mode: () => m._mode(),
    standPose: () => {
      m._reset()
      m._setMode(STAND)
      m._tick()
      const pose = angles()
      m._reset()
      return pose
    }
  }
}
