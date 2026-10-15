import { multiplyVector, get_transformation_matrix } from './math'

export interface body_state_t {
  omega: number
  phi: number
  psi: number
  xm: number
  ym: number
  zm: number
  feet: number[][]
  cumulative_x: number
  cumulative_y: number
  cumulative_z: number
  cumulative_roll: number
  cumulative_pitch: number
  cumulative_yaw: number
}

export interface HexapodConfig {
  legMountX: number[]
  legMountY: number[]
  legRootToJoint1: number
  legJoint1ToJoint2: number
  legJoint2ToJoint3: number
  legJoint3ToTip: number
  legMountAngle: number[]
}

/** The body level and centred over the stance, with no feet. */
export const levelBody = (): body_state_t => ({
  omega: 0,
  phi: 0,
  psi: 0,
  xm: 0,
  ym: 0,
  zm: 0,
  feet: [],
  cumulative_x: 0,
  cumulative_y: 0,
  cumulative_z: 0,
  cumulative_roll: 0,
  cumulative_pitch: 0,
  cumulative_yaw: 0
})

/**
 * The legs' forward kinematics, which the firmware has no counterpart of: the app draws the robot from joint angles,
 * and the robot only solves feet into angles (Kinematics::inverseKinematics in firmware/include/kinematics.h).
 */
export default class Kinematics {
  private readonly rootJ1: number
  private readonly j1J2: number
  private readonly j2J3: number
  private readonly j3Tip: number
  private readonly mountPosition: [number, number, number][]
  private readonly ca: number[]
  private readonly sa: number[]

  constructor(config: HexapodConfig) {
    this.rootJ1 = config.legRootToJoint1
    this.j1J2 = config.legJoint1ToJoint2
    this.j2J3 = config.legJoint2ToJoint3
    this.j3Tip = config.legJoint3ToTip
    this.mountPosition = config.legMountX.map((x, i) => [x, config.legMountY[i], 0])
    const mountAngles = config.legMountAngle.map(a => (a * Math.PI) / 180)
    this.ca = mountAngles.map(Math.cos)
    this.sa = mountAngles.map(Math.sin)
  }

  private invertTransform(T: number[][]): number[][] {
    const R = [
      [T[0][0], T[0][1], T[0][2]],
      [T[1][0], T[1][1], T[1][2]],
      [T[2][0], T[2][1], T[2][2]]
    ]
    const t = [T[0][3], T[1][3], T[2][3]]
    const Rt = [
      [R[0][0], R[1][0], R[2][0]],
      [R[0][1], R[1][1], R[2][1]],
      [R[0][2], R[1][2], R[2][2]]
    ]
    const tinv = [
      -(Rt[0][0] * t[0] + Rt[0][1] * t[1] + Rt[0][2] * t[2]),
      -(Rt[1][0] * t[0] + Rt[1][1] * t[1] + Rt[1][2] * t[2]),
      -(Rt[2][0] * t[0] + Rt[2][1] * t[1] + Rt[2][2] * t[2])
    ]
    return [
      [Rt[0][0], Rt[0][1], Rt[0][2], tinv[0]],
      [Rt[1][0], Rt[1][1], Rt[1][2], tinv[1]],
      [Rt[2][0], Rt[2][1], Rt[2][2], tinv[2]],
      [0, 0, 0, 1]
    ]
  }

  forwardKinematics(
    bodyState: body_state_t,
    angles: [number, number, number][]
  ): [number, number, number, number][] {
    const T = get_transformation_matrix(bodyState)
    const Ti = this.invertTransform(T)
    const feet: [number, number, number, number][] = []

    for (let i = 0; i < angles.length; i++) {
      const [a0, a1, a2] = angles[i]
      const phi = -a0
      const ux = Math.cos(phi)
      const uy = Math.sin(phi)
      const r = this.j2J3 * Math.cos(a1) + this.j3Tip * Math.cos(a1 + a2)
      const z = this.j2J3 * Math.sin(a1) + this.j3Tip * Math.sin(a1 + a2)
      const s = this.j1J2 + r
      const lx = this.rootJ1 + s * ux
      const ly = s * uy
      const lz = z

      const bx = this.mountPosition[i][0] + this.ca[i] * lx + this.sa[i] * ly
      const by = this.mountPosition[i][1] + this.sa[i] * lx - this.ca[i] * ly
      const bz = lz

      const foot = multiplyVector(Ti, [bx, by, bz, 1]).slice(0, 4) as [
        number,
        number,
        number,
        number
      ]
      feet.push(foot)
    }

    return feet
  }

  /**
   * The ground the feet stand on, in the body frame (mm), for joint angles (radians, per leg, IK order): the plane under
   * all six feet, its unit normal pointing up out of the ground (the body's world up) and its offset (normal . p on it),
   * and the feet resting on it. A foot lifted in its swing is above the plane and not among them.
   */
  groundUnderFeet(angles: [number, number, number][]): Ground {
    const feet = this.forwardKinematics(levelBody(), angles).map(([x, y, z]) => [x, y, z])
    const { normal, offset } = supportingPlane(feet)
    const planted = feet.filter(f => dot(normal, f) <= offset + SUPPORT_TOLERANCE_MM)
    return { normal, offset, planted }
  }
}

export interface Ground {
  normal: number[]
  offset: number
  planted: number[][]
}

const SUPPORT_TOLERANCE_MM = 0.5

const dot = (a: readonly number[], b: readonly number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

// The plane through three feet that has none of the six below it, normal pointing up out of the ground.
const supportingPlane = (feet: number[][]): { normal: number[]; offset: number } => {
  let best: { normal: number[]; offset: number } | undefined
  for (let a = 0; a < feet.length; a++)
    for (let b = a + 1; b < feet.length; b++)
      for (let c = b + 1; c < feet.length; c++) {
        const u = feet[b].map((v, i) => v - feet[a][i])
        const w = feet[c].map((v, i) => v - feet[a][i])
        let n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
        const length = Math.hypot(n[0], n[1], n[2])
        if (length < 1e-9) continue
        n = n.map(v => (v / length) * Math.sign(n[2] || 1))
        const offset = dot(n, feet[a])
        const supports = feet.every(f => dot(n, f) >= offset - SUPPORT_TOLERANCE_MM)
        if (supports && (!best || n[2] > best.normal[2])) best = { normal: n, offset }
      }
  if (!best) throw new Error('No three feet support the body')
  return best
}
