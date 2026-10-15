// @vitest-environment node
// The simulation view's physics with the firmware's own motion code: the WebAssembly virtual robot driving
// simulation/src/resources/model.xml in MuJoCo. Needs the virtual robot built (scripts/build_virtual_robot.js).
import { readFileSync } from 'node:fs'
import path from 'node:path'
import { beforeAll, describe, expect, it } from 'vitest'
import loadMujoco, { type MainModule } from '@mujoco/mujoco'
import { HexapodSim } from '../../src/lib/simulation/hexapod-sim'
import {
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
const FORWARD: Command = [0, 1, 0, 0, 0, 0, 0, 0]

let mujoco: MainModule
let robot: VirtualRobot

// Runs for `seconds` in 20 ms frames, as the view does, sending `command` each frame when given.
function run(sim: HexapodSim, seconds: number, command?: Command) {
  for (let t = 0; t < seconds; t += 0.02) {
    if (command) robot.command(command)
    sim.step(0.02)
  }
}

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
})
