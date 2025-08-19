import { degToRad } from 'three/src/math/MathUtils'
import { config } from './components/config'
import Kinematics, { type body_state_t } from './kinematic'
import {
  default_offset,
  default_stand_frac,
  GaitController,
  GaitType,
  type gait_state_t
} from './gait'

export enum MotionModes {
  DEACTIVATED = 'deactivated',
  IDLE = 'idle',
  POSE = 'pose',
  STAND = 'stand',
  WALK = 'walk',
  CONSTRAINED_RANDOM = 'constrained_random'
}

export default class Motion {
  mode: MotionModes
  kinematics: Kinematics
  gait: GaitController
  body_state: body_state_t
  gait_state: gait_state_t
  defaultPosition: [number, number, number, number][]
  angles = new Array(18).fill(0)
  targetAngles = new Array(18).fill(0)

  poses = [new Array(18).fill(0)]
  current_pose_idx = 0

  lastTick: number = 0
  lastPoseGeneration: number = 0
  constrainedPoses: number[][] = []
  constrainedPoseIndex = 0
  targetPose: [number, number, number, number][] = []
  isMovingToTarget = false

  constructor() {
    this.mode = MotionModes.STAND
    this.kinematics = new Kinematics(config)
    this.defaultPosition = this.kinematics.genPosture(degToRad(60), degToRad(75))
    this.gait = new GaitController(this.defaultPosition)
    this.body_state = {
      omega: 0,
      phi: 0,
      psi: 0,
      xm: 0,
      ym: 0,
      zm: 0,
      feet: this.defaultPosition
    }
    this.gait_state = {
      step_height: 15,
      step_x: 0,
      step_z: 0,
      step_angle: 0,
      step_speed: 1,
      step_depth: 0.002,
      stand_frac: default_stand_frac[GaitType.TRI_GATE],
      offset: default_offset[GaitType.TRI_GATE],
      gait_type: GaitType.TRI_GATE
    }

    this.initializeConstrainedPoses()
  }

  setMode(mode: MotionModes) {
    this.mode = mode
    if (mode === MotionModes.CONSTRAINED_RANDOM) {
      this.lastPoseGeneration = 0
      this.constrainedPoseIndex = 0
      this.isMovingToTarget = false
      this.gait_state.gait_type = GaitType.WAVE
      this.gait_state.offset = default_offset[GaitType.WAVE]
      this.gait_state.stand_frac = default_stand_frac[GaitType.WAVE]
      this.gait_state.step_height = 8
      this.gait_state.step_speed = 0.5
      this.body_state.feet = this.defaultPosition
      this.generateConstrainedRandomPose()
    }
  }

  setGait(gait: GaitType) {
    this.gait_state.gait_type = gait
    this.gait_state.offset = default_offset[gait]
    this.gait_state.stand_frac = default_stand_frac[gait]
  }

  handleCommand(command: number[]) {
    this.body_state.zm = command[4] * 50
    this.body_state.omega = command[3] * 0.254
    switch (this.mode) {
      case MotionModes.STAND:
        this.body_state.xm = command[0] * 50
        this.body_state.ym = -command[1] * 50
        this.body_state.phi = command[2] * 0.254
        break
      case MotionModes.WALK:
        this.gait_state.step_x = -command[0] * 100
        this.gait_state.step_z = command[1] * 100
        this.gait_state.step_angle = command[2] * 0.8
        this.gait_state.step_speed = command[5] + 1
        this.gait_state.step_height = (command[6] + 1) * 20
        this.gait_state.step_depth = 0.002
        break
    }
  }

  step() {
    switch (this.mode) {
      case MotionModes.DEACTIVATED:
      case MotionModes.IDLE:
        return false
      case MotionModes.POSE:
        this.targetAngles = this.poses[this.current_pose_idx]
        break
      case MotionModes.STAND: {
        this.targetAngles = this.order(this.kinematics.inverseKinematics(this.body_state).flat())
        break
      }
      case MotionModes.WALK: {
        const delta = performance.now() - this.lastTick
        this.lastTick = performance.now()
        this.gait.step(this.gait_state, this.body_state, delta / 1000)
        this.targetAngles = this.order(this.kinematics.inverseKinematics(this.body_state).flat())
        break
      }
      case MotionModes.CONSTRAINED_RANDOM: {
        const now = performance.now()
        if (now - this.lastPoseGeneration > 2000) {
          this.generateConstrainedRandomPose()
          this.lastPoseGeneration = now
        }

        if (this.isMovingToTarget && this.targetPose.length > 0) {
          const delta = performance.now() - this.lastTick
          this.lastTick = performance.now()

          this.body_state.feet = this.moveTowardTarget(
            this.body_state.feet as [number, number, number, number][],
            this.targetPose,
            delta / 1000
          )
        }

        this.targetAngles = this.order(this.kinematics.inverseKinematics(this.body_state).flat())
        break
      }
    }
    return true
  }

  initializeConstrainedPoses() {
    this.constrainedPoses = [this.generateStandingPose(), new Array(18).fill(0)]
  }

  generateStandingPose(): number[] {
    const tempBodyState = { ...this.body_state }
    tempBodyState.feet = this.defaultPosition

    const poseAngles = this.kinematics.inverseKinematics(tempBodyState)
    return this.order(poseAngles.flat())
  }

  generateConstrainedRandomPose() {
    if (this.constrainedPoseIndex < 1) {
      this.constrainedPoseIndex++
      this.isMovingToTarget = false
      return
    }

    const constrainedFeet = this.defaultPosition.map(defaultFoot => {
      const randomX = defaultFoot[0] + (Math.random() - 0.5) * 100
      const randomY = defaultFoot[1] + (Math.random() - 0.5) * 80

      return [randomX, randomY, defaultFoot[2], 1] as [number, number, number, number]
    })

    this.targetPose = constrainedFeet
    this.isMovingToTarget = true
    this.gait.phase = 0

    const tempBodyState = {
      omega: 0,
      phi: 0,
      psi: 0,
      xm: 0,
      ym: 0,
      zm: 0,
      feet: constrainedFeet
    }

    const poseAngles = this.kinematics.inverseKinematics(tempBodyState)
    this.constrainedPoses[1] = this.order(poseAngles.flat())

    this.constrainedPoseIndex = (this.constrainedPoseIndex + 1) % 2
  }

  moveTowardTarget(
    currentFeet: [number, number, number, number][],
    targetFeet: [number, number, number, number][],
    dt: number
  ): [number, number, number, number][] {
    const distances = currentFeet.map((currentFoot, i) => {
      const dx = targetFeet[i][0] - currentFoot[0]
      const dy = targetFeet[i][1] - currentFoot[1]
      const dz = targetFeet[i][2] - currentFoot[2]
      return Math.hypot(dx, dy, dz)
    })

    const maxDistance = Math.max(...distances)
    const threshold = 5

    if (maxDistance < threshold) {
      this.isMovingToTarget = false
      return targetFeet
    }

    const newFeet = currentFeet.map((currentFoot, i) => {
      const targetFoot = targetFeet[i]
      const phase = (this.gait.phase + this.gait_state.offset[i]) % 1

      const dx = targetFoot[0] - currentFoot[0]
      const dy = targetFoot[1] - currentFoot[1]
      const dz = targetFoot[2] - currentFoot[2]
      const distance = Math.hypot(dx, dy, dz)

      if (distance < threshold) {
        return targetFoot
      }

      let stepProgress = 0
      if (phase < this.gait_state.stand_frac) {
        stepProgress = 0
      } else {
        const liftPhase = (phase - this.gait_state.stand_frac) / (1 - this.gait_state.stand_frac)
        stepProgress = liftPhase
      }

      const stepSize = Math.min(distance, 15)
      const heightCurve = Math.sin(stepProgress * Math.PI) * this.gait_state.step_height

      const newX = currentFoot[0] + (dx / distance) * stepSize * stepProgress
      const newY = currentFoot[1] + (dy / distance) * stepSize * stepProgress
      const newZ = currentFoot[2] + (dz / distance) * stepSize * stepProgress + heightCurve

      return [newX, newY, newZ, 1] as [number, number, number, number]
    })

    this.gait.phase = (this.gait.phase + dt * this.gait_state.step_speed) % 1

    return newFeet
  }

  order(a: number[]) {
    return [3, 0, 4, 1, 5, 2].flatMap(i => a.slice(i * 3, i * 3 + 3))
  }
}
