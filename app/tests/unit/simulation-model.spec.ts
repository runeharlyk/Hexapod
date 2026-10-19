// The simulation view draws the URDF model (static/model.xacro) on MuJoCo's base pose with MuJoCo's joint angles, so
// each leg the firmware drives must be drawn where MuJoCo simulates it and where the sign convention puts it.
// Needs the virtual robot built (scripts/build_virtual_robot.js).
import { readFileSync } from 'node:fs'
import path from 'node:path'
import { beforeAll, describe, expect, it } from 'vitest'
import { Mesh, Object3D, Vector3 } from 'three'
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js'
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
// The URDF's meshes are not the MuJoCo model's geometry: a drawn toe lies within this of the simulated foot.
const TOE_TO_FOOT_M = 0.03

let mujoco: MainModule
let robot: VirtualRobot

// The URDF model as the app loads it, with its meshes. Its link frames sit off the physical joints, so only the
// meshes show where a foot is drawn.
async function loadUrdf(): Promise<URDFRobot> {
  const parser = new XacroParser()
  parser.getFileContents = async file => readFileSync(file, 'utf8')
  const xml = await parser.parse(readFileSync(path.join(repo, 'app/static/model.xacro'), 'utf8'))
  const loader = new URDFLoader()
  loader.packages = { hex: path.join(repo, 'app/static/hex') }
  const stl = new STLLoader()
  loader.loadMeshCb = (file, _manager, done) =>
    done(new Mesh(stl.parse(readFileSync(file).buffer as ArrayBuffer)))
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

// The lowest point of a link's meshes in the frame of `body`: the drawn toe of a foot link.
function toe(link: Object3D, body: Object3D): Vector3 {
  let lowest: Vector3 | undefined
  link.traverse(part => {
    if (!(part instanceof Mesh)) return
    const vertices = part.geometry.attributes.position
    for (let i = 0; i < vertices.count; i++) {
      const v = body.worldToLocal(part.localToWorld(new Vector3().fromBufferAttribute(vertices, i)))
      if (!lowest || v.z < lowest.z) lowest = v
    }
  })
  return lowest!
}

describe('the simulation view', () => {
  beforeAll(async () => {
    mujoco = await loadMujoco()
    robot = await loadVirtualRobot()
  })

  it("draws each foot where MuJoCo simulates it and on the convention's side", async () => {
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

    const shoulders = shoulderOfLeg(jointNames)
    fixture.legs.forEach((leg, i) => {
      const drawn = toe(urdf.links[shoulders[i].replace(/_shoulder$/, '_foot_link')], body)
      const simulated = body.worldToLocal(new Vector3(...sim.footPositions()[i]))
      const side = fixture.stance_mm[i].slice(0, 2).map(Math.sign)
      expect(Math.sign(simulated.x), `${leg}: MuJoCo side`).toBe(side[0])
      expect(Math.sign(drawn.x), `${leg}: drawn side`).toBe(side[0])
      if (side[1] !== 0) {
        expect(Math.sign(simulated.y), `${leg}: MuJoCo front or rear`).toBe(side[1])
        expect(Math.sign(drawn.y), `${leg}: drawn front or rear`).toBe(side[1])
      }
      const apart = Math.hypot(drawn.x - simulated.x, drawn.y - simulated.y)
      expect(apart, `${leg}: drawn toe to MuJoCo foot (m)`).toBeLessThan(TOE_TO_FOOT_M)
    })
    sim.dispose()
  })
})
