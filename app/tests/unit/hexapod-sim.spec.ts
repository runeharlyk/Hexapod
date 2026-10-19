// @vitest-environment node
// The simulation view's physics with the firmware's own motion code: the WebAssembly virtual robot driving
// simulation/src/resources/model.xml in MuJoCo. Needs the virtual robot built (scripts/build_virtual_robot.js).
import { readFileSync } from 'node:fs'
import path from 'node:path'
import { beforeAll, describe, expect, it } from 'vitest'
import loadMujoco, { type MainModule } from '@mujoco/mujoco'
import { HexapodSim } from '../../src/lib/simulation/hexapod-sim'
import {
  DMP_ANGLE_SIGNS,
  loadVirtualRobot,
  type Command,
  type VirtualRobot
} from '../../src/lib/simulation/virtual-robot'

const sceneXml = readFileSync(
  path.resolve(__dirname, '../../../simulation/src/resources/model.xml'),
  'utf8'
)
const STAND = 3
const WALK = 4
const FORWARD: Command = [0, 1, 0, 0, 0, 0, 0, 0, 0]
const STRAFE_RIGHT: Command = [1, 0, 0, 0, 0, 0, 0, 0, 0]
const TURN_RIGHT: Command = [0, 0, 1, 0, 0, 0, 0, 0, 0]

let mujoco: MainModule
let robot: VirtualRobot

// Runs for `seconds` in 20 ms frames, as the view does, sending `command` each frame when given.
function run(sim: HexapodSim, seconds: number, command?: Command) {
  for (let t = 0; t < seconds; t += 0.02) {
    if (command) robot.command(command)
    sim.step(0.02)
  }
}

// The base's heading about world +z, counter-clockwise from above.
function yaw(sim: HexapodSim) {
  const [w, x, y, z] = sim.baseQuaternion()
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
}

// Stands, walks `command` from a stand for `seconds`, and returns how far the base moved and turned.
function walkFromStand(command: Command, seconds: number) {
  const sim = new HexapodSim(mujoco, sceneXml, robot)
  robot.setMode(STAND)
  run(sim, 1)
  robot.setMode(WALK)
  const [x0, y0] = sim.basePosition()
  const yaw0 = yaw(sim)
  run(sim, seconds, command)
  const [x1, y1] = sim.basePosition()
  const moved = { x: x1 - x0, y: y1 - y0, yaw: yaw(sim) - yaw0, fallen: sim.hasFallen() }
  sim.dispose()
  return moved
}

describe("the simulated IMU's angle signs", () => {
  it("are the firmware's, from imu_frame.h", () => {
    const header = readFileSync(
      path.resolve(__dirname, '../../../firmware/include/imu_frame.h'),
      'utf8'
    )
    const signs = ['ROLL', 'PITCH', 'YAW'].map(name =>
      Number(new RegExp(`${name}_SIGN = ([+-][0-9.]+)f;`).exec(header)![1])
    )
    expect(signs).toEqual([...DMP_ANGLE_SIGNS])
  })
})

describe('the simulated hexapod', () => {
  beforeAll(async () => {
    mujoco = await loadMujoco()
    robot = await loadVirtualRobot()
  })

  it('spawns with its feet on the floor and sags while unpowered', () => {
    const sim = new HexapodSim(mujoco, sceneXml, robot)
    const spawn = sim.basePosition()[2]
    run(sim, 1)
    expect(sim.basePosition()[2]).toBeLessThan(spawn - 0.01)
    sim.dispose()
  })

  it('stands up and stays level when switched to STAND', () => {
    const sim = new HexapodSim(mujoco, sceneXml, robot)
    robot.setMode(STAND)
    run(sim, 2)
    const [, x, y] = sim.baseQuaternion()
    expect(sim.hasFallen()).toBe(false)
    expect(sim.basePosition()[2]).toBeGreaterThan(0.05)
    expect(1 - 2 * (x * x + y * y)).toBeGreaterThan(Math.cos((5 * Math.PI) / 180))
    sim.dispose()
  })

  it('walks forward, along +y, for a stick pushed forward', () => {
    const sim = new HexapodSim(mujoco, sceneXml, robot)
    robot.setMode(STAND)
    run(sim, 1)
    robot.setMode(WALK)
    const [x0, y0] = sim.basePosition()
    run(sim, 3, FORWARD)
    const [x1, y1] = sim.basePosition()
    expect(sim.hasFallen()).toBe(false)
    expect(y1 - y0).toBeGreaterThan(0.1)
    expect(Math.abs(x1 - x0)).toBeLessThan((y1 - y0) / 4)
    sim.dispose()
  })

  // docs/conventions.md: for an operator behind the robot, a stick pushed right walks it to its right, +x.
  it('strafes right, along +x, for the left stick pushed right', () => {
    const moved = walkFromStand(STRAFE_RIGHT, 3)
    expect(moved.fallen).toBe(false)
    expect(moved.x).toBeGreaterThan(0.1)
    expect(Math.abs(moved.y)).toBeLessThan(moved.x / 4)
  })

  it('turns clockwise for the right stick pushed right', () => {
    const moved = walkFromStand(TURN_RIGHT, 1)
    expect(moved.fallen).toBe(false)
    expect(moved.yaw).toBeLessThan(-0.3)
    expect(Math.hypot(moved.x, moved.y)).toBeLessThan(0.05)
  })
})
