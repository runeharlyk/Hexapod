import type { body_state_t } from './kinematic'
import type { Matrix } from './math'

export enum GaitType {
  TRI_GATE = 'Tri Gate',
  BI_GATE = 'Bi Gate',
  WAVE = 'Wave',
  RIPPLE = 'Ripple'
}

export const GaitLabels: Record<string, GaitType> = {
  'Tri Gate': GaitType.TRI_GATE,
  'Bi Gate': GaitType.BI_GATE,
  Wave: GaitType.WAVE,
  Ripple: GaitType.RIPPLE
}

export const default_offset: Record<GaitType, number[]> = {
  [GaitType.TRI_GATE]: [0, 0.52, 0.08, 0.58, 0.16, 0.66],
  [GaitType.BI_GATE]: [0, 1 / 3, 2 / 3, 2 / 3, 1 / 3, 0],
  [GaitType.WAVE]: [0, 1 / 6, 2 / 6, 5 / 6, 4 / 6, 3 / 6],
  [GaitType.RIPPLE]: [0, 4 / 6, 2 / 6, 1 / 6, 5 / 6, 3 / 6]
}

export const default_stand_frac: Record<GaitType, number> = {
  [GaitType.TRI_GATE]: 3.1 / 6,
  [GaitType.BI_GATE]: 2.1 / 6,
  [GaitType.WAVE]: 5 / 6,
  [GaitType.RIPPLE]: 5 / 6
}

export interface gait_state_t {
  step_height: number
  step_x: number
  step_z: number
  step_angle: number
  step_speed: number
  step_depth: number
  stand_frac: number
  offset: number[]
  gait_type: GaitType
}

export interface ControllerCommand {
  stop: number
  lx: number
  ly: number
  rx: number
  ry: number
  h: number
  s: number
  s1: number
}

export class GaitController {
  defaultPosition: Matrix
  targetDefaultPosition: Matrix
  swingStartPosition: Matrix
  footWasSwinging: boolean[]
  phase = 0
  last_body_state: body_state_t | null = null
  protected cumulative_position = { x: 0, y: 0, z: 0 }
  protected cumulative_orientation = { roll: 0, pitch: 0, yaw: 0 }
  protected walkTranslationScale = 2.5

  constructor(defaultPosition: Matrix) {
    this.defaultPosition = defaultPosition
    this.targetDefaultPosition = defaultPosition.map(foot => [...foot])
    this.swingStartPosition = defaultPosition.map(foot => [...foot])
    this.footWasSwinging = new Array(defaultPosition.length).fill(false)
  }

  setDefaultFootTarget(targetDefaultPosition: Matrix) {
    this.targetDefaultPosition = targetDefaultPosition.map(foot => [...foot])
  }

  hasPendingStanceChange() {
    return this.defaultPosition.some((foot, i) =>
      foot.some((value, j) => Math.abs(value - this.targetDefaultPosition[i][j]) > 0.5)
    )
  }

  step(gait: gait_state_t, body: body_state_t, dt: number) {
    const { step_x, step_z, step_angle: angle } = gait
    const isMoving = Math.abs(step_x) >= 2 || Math.abs(step_z) >= 2 || angle !== 0
    const isRepositioning = !isMoving && this.hasPendingStanceChange()

    if (!isMoving && !isRepositioning) {
      body.feet = body.feet.map((f, i) =>
        f.map((v, j) => v + (this.defaultPosition[i][j] - v) * dt * 10)
      )
      this.phase = 0
      return
    }

    const lengthRaw = Math.hypot(step_x, step_z)
    const length = step_x < 0 ? -lengthRaw : lengthRaw
    const speed = isRepositioning
      ? gait.step_speed
      : gait.step_speed * Math.min(1.5, Math.max(0.75, Math.abs(length) / 25, Math.abs(angle * 1.5)))
    const turnAmplitude = Math.atan2(step_z, length) * 2

    this.advancePhase(dt, speed)

    this.defaultPosition = this.defaultPosition.map((foot, i) => {
      const phase = (this.phase + gait.offset[i]) % 1
      const isSwinging = phase >= gait.stand_frac

      if (isSwinging && !this.footWasSwinging[i]) {
        this.swingStartPosition[i] = [...foot]
      }
      this.footWasSwinging[i] = isSwinging

      if (!isSwinging) return foot

      const swingProgress = (phase - gait.stand_frac) / (1 - gait.stand_frac)
      return foot.map((_, j) =>
        this.swingStartPosition[i][j] +
        (this.targetDefaultPosition[i][j] - this.swingStartPosition[i][j]) * swingProgress
      )
    })

    const newFeet = this.defaultPosition.map(fp => fp.map(() => 0))

    for (let i = 0; i < this.defaultPosition.length; i++) {
      const defaultFoot = this.defaultPosition[i]
      const currentFoot = body.feet[i]
      const phase = (this.phase + gait.offset[i]) % 1
      const [phNorm, curveFn, amp] = this.phaseParams(
        phase,
        gait.stand_frac,
        gait.step_depth,
        gait.step_height
      )
      const deltaPos = curveFn(length / 2, turnAmplitude, amp, phNorm)
      const deltaRot = curveFn(
        (angle * 180) / Math.PI,
        yawArc(defaultFoot, currentFoot),
        amp,
        phNorm
      )
      newFeet[i] = defaultFoot.map((v, j) => v + deltaPos[j] + deltaRot[j])
      newFeet[i][3] = 1
    }

    body.feet = newFeet
    this.update_cumulative_position(body, gait, dt)
  }

  private advancePhase(dt: number, velocity: number) {
    this.phase = (this.phase + dt * velocity) % 1
  }

  private phaseParams(
    phase: number,
    standFrac: number,
    depth: number,
    height: number
  ): [number, typeof sine_curve | typeof bezier_curve, number] {
    if (phase < standFrac) {
      return [phase / standFrac, sine_curve, -depth]
    }
    return [(phase - standFrac) / (1 - standFrac), bezier_curve, height]
  }

  update_cumulative_position(body_state: body_state_t, gait_state: gait_state_t, dt: number) {
    if (this.last_body_state === null) {
      this.last_body_state = { ...body_state }
      body_state.cumulative_x = 0
      body_state.cumulative_y = 0
      body_state.cumulative_z = 0
      body_state.cumulative_roll = 0
      body_state.cumulative_pitch = 0
      body_state.cumulative_yaw = 0
      return
    }

    const m = gait_state
    const moving = m.step_x !== 0 || m.step_z !== 0 || m.step_angle !== 0

    if (moving) {
      const step_displacement_x_local = m.step_x * this.walkTranslationScale * m.step_speed * dt
      const step_displacement_z_local = m.step_z * this.walkTranslationScale * m.step_speed * dt
      const step_displacement_yaw = m.step_angle * m.step_speed * dt

      const cos_yaw = Math.cos(this.cumulative_orientation.yaw)
      const sin_yaw = Math.sin(this.cumulative_orientation.yaw)
      const step_displacement_x =
        step_displacement_x_local * cos_yaw - step_displacement_z_local * sin_yaw
      const step_displacement_z =
        step_displacement_x_local * sin_yaw + step_displacement_z_local * cos_yaw

      this.cumulative_position.x += step_displacement_x
      this.cumulative_position.z += step_displacement_z
      this.cumulative_orientation.yaw += step_displacement_yaw
    }

    body_state.cumulative_x = this.cumulative_position.x
    body_state.cumulative_y = this.cumulative_position.y
    body_state.cumulative_z = this.cumulative_position.z
    body_state.cumulative_roll = this.cumulative_orientation.roll
    body_state.cumulative_pitch = this.cumulative_orientation.pitch
    body_state.cumulative_yaw = this.cumulative_orientation.yaw

    this.last_body_state = { ...body_state }
  }
}

export function sine_curve(length: number, angle: number, height: number, phase: number): number[] {
  const step = length * (1 - 2 * phase)
  const x = step * Math.cos(angle)
  const z = step * Math.sin(angle)
  const y = length ? height * Math.cos((Math.PI * (x + z)) / (2 * length)) : 0
  return [x, z, y]
}

const yawArc = (default_foot_pos: number[], current_foot_pos: number[]): number => {
  const foot_mag = Math.hypot(default_foot_pos[0] + default_foot_pos[1])
  const foot_dir = Math.atan2(default_foot_pos[1], default_foot_pos[0])
  const offsets = [
    current_foot_pos[0] - default_foot_pos[0],
    current_foot_pos[2] - default_foot_pos[2],
    current_foot_pos[1] - default_foot_pos[1]
  ]
  const offset_mag = Math.hypot(offsets[0] + offsets[1])
  const offset_mod = Math.atan2(offset_mag, foot_mag)

  return Math.PI / 2.0 + foot_dir + offset_mod
}

const bezier_curve = (length: number, angle: number, height: number, phase: number): number[] => {
  const control_points = get_control_points(length, angle, height)
  const n = control_points.length - 1

  const point = [0, 0, 0]
  for (let i = 0; i <= n; i++) {
    const bernstein_poly = comb(n, i) * Math.pow(phase, i) * Math.pow(1 - phase, n - i)
    point[0] += bernstein_poly * control_points[i][0]
    point[1] += bernstein_poly * control_points[i][1]
    point[2] += bernstein_poly * control_points[i][2]
  }
  return point
}
const get_control_points = (length: number, angle: number, height: number): number[][] => {
  const X_POLAR = Math.cos(angle)
  const Z_POLAR = Math.sin(angle)

  const STEP = [
    -length,
    -length * 1.4,
    -length * 1.5,
    -length * 1.5,
    -length * 1.5,
    0.0,
    0.0,
    0.0,
    length * 1.5,
    length * 1.5,
    length * 1.4,
    length
  ]

  const Y = [
    0.0,
    0.0,
    height * 0.9,
    height * 0.9,
    height * 0.9,
    height * 0.9,
    height * 0.9,
    height * 1.1,
    height * 1.1,
    height * 1.1,
    0.0,
    0.0
  ]

  const control_points: number[][] = []

  for (let i = 0; i < STEP.length; i++) {
    const X = STEP[i] * X_POLAR
    const Z = STEP[i] * Z_POLAR
    control_points.push([X, Z, Y[i]])
  }

  return control_points
}

const comb = (n: number, k: number): number => {
  if (k < 0 || k > n) return 0
  if (k === 0 || k === n) return 1
  k = Math.min(k, n - k)
  let c = 1
  for (let i = 0; i < k; i++) {
    c = (c * (n - i)) / (i + 1)
  }
  return c
}
