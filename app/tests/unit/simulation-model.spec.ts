// The simulation view draws the URDF model (static/model.xacro) on MuJoCo's base pose with MuJoCo's joint angles, so
// each leg the firmware drives must be drawn where MuJoCo simulates it and where the sign convention puts it.
// Needs the virtual robot built (scripts/build_virtual_robot.js).
import { readFileSync } from 'node:fs'
import path from 'node:path'
import { beforeAll, describe, expect, it } from 'vitest'
import { Object3D, Vector3 } from 'three'
import URDFLoader, { type URDFRobot } from 'urdf-loader'
import { XacroParser } from 'xacro-parser'
import loadMujoco, { type MainModule } from '@mujoco/mujoco'
import { HexapodSim } from '../../src/lib/simulation/hexapod-sim'
import { mountModel } from '../../src/lib/simulation/model-mount'
import { loadVirtualRobot, type VirtualRobot } from '../../src/lib/simulation/virtual-robot'
import { urdfJointOrder } from '../../src/lib/motion'

const repo = path.resolve(__dirname, '../../..')
const sceneXml = readFileSync(path.join(repo, 'simulation/src/resources/model.xml'), 'utf8')
const fixture: { legs: string[]; stance_mm: number[][] } = JSON.parse(
  readFileSync(path.join(repo, 'platform_shared/sign_convention.json'), 'utf8')
)
const STAND = 3
// A middle leg sits at y = 0 in the convention, so only its side is compared.
const sign = (v: number) => Math.sign(Math.round(v * 1e3))

let mujoco: MainModule
let robot: VirtualRobot

// The URDF model as the app loads it, its meshes left out.
async function loadUrdf(): Promise<URDFRobot> {
  const parser = new XacroParser()
  parser.getFileContents = async file => readFileSync(file, 'utf8')
  const xml = await parser.parse(readFileSync(path.join(repo, 'app/static/model.xacro'), 'utf8'))
  const loader = new URDFLoader()
  loader.loadMeshCb = (_path, _manager, done) => done(new Object3D())
  return loader.parse(xml)
}

// Which firmware leg each of the model's legs draws: the leg whose angles urdfJointOrder hands its shoulder joint.
function shoulderOfLeg(jointNames: string[]): string[] {
  const legOfSlot = urdfJointOrder([0, 1, 2, 3, 4, 5].flatMap(leg => [leg, leg, leg]))
  const shoulders: string[] = []
  jointNames.forEach((name, slot) => {
    if (name.endsWith('_shoulder')) shoulders[legOfSlot[slot]] = name
  })
  return shoulders
}

describe('the simulation view', () => {
  beforeAll(async () => {
    mujoco = await loadMujoco()
    robot = await loadVirtualRobot()
  })

  it("draws each leg where MuJoCo simulates it and on the convention's side", async () => {
    const sim = new HexapodSim(mujoco, sceneXml, robot)
    robot.setMode(STAND)
    for (let t = 0; t < 1.5; t += 0.02) sim.step(0.02)

    const urdf = await loadUrdf()
    const jointNames = Object.entries(urdf.joints)
      .filter(([, joint]) => joint.jointType !== 'fixed')
      .map(([name]) => name)
    const body = new Object3D()
    mountModel(urdf, body)
    const [x, y, z] = sim.basePosition()
    const [w, qx, qy, qz] = sim.baseQuaternion()
    body.position.set(x, y, z)
    body.quaternion.set(qx, qy, qz, w)
    const angles = urdfJointOrder(sim.jointAngles())
    jointNames.forEach((name, i) => urdf.joints[name].setJointValue(angles[i]))
    body.updateMatrixWorld(true)

    const base = new Vector3(x, y, z)
    const shoulders = shoulderOfLeg(jointNames)
    fixture.legs.forEach((leg, i) => {
      const link = shoulders[i].replace(/_shoulder$/, '_foot_link')
      const drawn = urdf.links[link].getWorldPosition(new Vector3()).sub(base)
      const simulated = [0, 1].map(a => sim.footPositions()[i][a] - base.getComponent(a))
      const expected = fixture.stance_mm[i].slice(0, 2).map(Math.sign)
      expect(sign(simulated[0]), `${leg}: MuJoCo side`).toBe(expected[0])
      expect(sign(drawn.x), `${leg}: drawn side`).toBe(expected[0])
      if (expected[1] === 0) return
      expect(sign(simulated[1]), `${leg}: MuJoCo front or rear`).toBe(expected[1])
      expect(sign(drawn.y), `${leg}: drawn front or rear`).toBe(expected[1])
    })
    sim.dispose()
  })
})
