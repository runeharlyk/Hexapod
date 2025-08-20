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
  CONSTRAINED_RANDOM = 'constrained_random',
  LAYING_TRANSITION = 'laying_transition',
  STANDING_UP = 'standing_up'
}

export default class Motion {
  mode: MotionModes
  kinematics: Kinematics
  gait: GaitController
  body_state: body_state_t
  gait_state: gait_state_t
  defaultPosition: [number, number, number, number][]
  baseDefaultPosition: [number, number, number, number][]
  feetDistanceScale: number = 1.0
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
  standTargetPose: [number, number, number, number][] = []
  layingDownPose: [number, number, number, number][] = []
  standingUpPhase = 0
  layingDownPhase = 0

  constructor() {
    this.mode = MotionModes.STAND
    this.kinematics = new Kinematics(config)
    this.baseDefaultPosition = this.kinematics.genPosture(degToRad(60), degToRad(75))
    this.defaultPosition = [...this.baseDefaultPosition]
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
    this.generateLayingDownPose()
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
    } else if (mode === MotionModes.LAYING_TRANSITION) {
      this.isMovingToTarget = true
      this.layingDownPhase = 0
      this.targetPose = this.calculateNextLayingStep()
      this.lastPoseGeneration = performance.now()
    } else if (mode === MotionModes.STANDING_UP) {
      this.isMovingToTarget = true
      this.standingUpPhase = 0
      this.targetPose = this.calculateNextStandingStep()
      this.lastPoseGeneration = performance.now()
    }
  }

  setGait(gait: GaitType) {
    this.gait_state.gait_type = gait
    this.gait_state.offset = default_offset[gait]
    this.gait_state.stand_frac = default_stand_frac[gait]
  }

  updateFeetDistance(distanceValue: number) {
    this.feetDistanceScale = 0.5 + (distanceValue + 1) * 0.5

    const newDefaultPosition = this.baseDefaultPosition.map(foot => {
      const [x, y, z, w] = foot
      const distanceFromCenter = Math.hypot(x, y)
      const angle = Math.atan2(y, x)
      const newDistance = distanceFromCenter * this.feetDistanceScale

      return [Math.cos(angle) * newDistance, Math.sin(angle) * newDistance, z, w] as [
        number,
        number,
        number,
        number
      ]
    })

    this.defaultPosition = newDefaultPosition
    this.gait.defaultPosition = this.defaultPosition

    if (this.mode === MotionModes.STAND) {
      this.standTargetPose = this.defaultPosition
      this.isMovingToTarget = true
      this.gait.phase = 0
      this.gait_state.gait_type = GaitType.WAVE
      this.gait_state.offset = default_offset[GaitType.WAVE]
      this.gait_state.stand_frac = default_stand_frac[GaitType.WAVE]
      this.gait_state.step_height = 8
      this.gait_state.step_speed = 0.5
    } else {
      this.body_state.feet = this.defaultPosition
    }
  }

  handleCommand(command: number[]) {
    this.body_state.zm = command[4] * 50
    this.body_state.omega = command[3] * 0.254

    if (command[7] !== undefined) {
      this.updateFeetDistance(command[7])
    }

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
        if (this.isMovingToTarget && this.standTargetPose.length > 0) {
          if (this.lastTick === 0) this.lastTick = performance.now()
          const delta = performance.now() - this.lastTick
          this.lastTick = performance.now()

          this.body_state.feet = this.moveTowardTarget(
            this.body_state.feet as [number, number, number, number][],
            this.standTargetPose,
            delta / 1000
          )

          if (!this.isMovingToTarget) {
            this.standTargetPose = []
            this.gait_state.gait_type = GaitType.TRI_GATE
            this.gait_state.offset = default_offset[GaitType.TRI_GATE]
            this.gait_state.stand_frac = default_stand_frac[GaitType.TRI_GATE]
            this.gait_state.step_height = 15
            this.gait_state.step_speed = 1
          }
        }

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
      case MotionModes.LAYING_TRANSITION: {
        const now = performance.now()

        if (!this.isMovingToTarget && now - this.lastPoseGeneration > 800) {
          const nextStep = this.calculateNextLayingStep()
          if (nextStep.length > 0) {
            this.targetPose = nextStep
            this.isMovingToTarget = true
          }
          this.lastPoseGeneration = now
        }

        if (this.isMovingToTarget && this.targetPose.length > 0) {
          this.body_state.feet = this.moveDirectlyToTarget(
            this.body_state.feet as [number, number, number, number][],
            this.targetPose
          )
        }

        this.targetAngles = this.order(this.kinematics.inverseKinematics(this.body_state).flat())
        break
      }
      case MotionModes.STANDING_UP: {
        const now = performance.now()

        if (!this.isMovingToTarget && now - this.lastPoseGeneration > 800) {
          const nextStep = this.calculateNextStandingStep()
          if (nextStep.length > 0) {
            this.targetPose = nextStep
            this.isMovingToTarget = true
          }
          this.lastPoseGeneration = now
        }

        if (this.isMovingToTarget && this.targetPose.length > 0) {
          this.body_state.feet = this.moveDirectlyToTarget(
            this.body_state.feet as [number, number, number, number][],
            this.targetPose
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

  generateLayingDownPose() {
    const zeroAngles: [number, number, number][] = Array(6).fill([0, 0, 0])

    const neutralBodyState = {
      omega: 0,
      phi: 0,
      psi: 0,
      xm: 0,
      ym: 0,
      zm: 0,
      feet: this.defaultPosition
    }

    this.layingDownPose = this.kinematics.forwardKinematics(neutralBodyState, zeroAngles)
  }

  calculateNextLayingStep(): [number, number, number, number][] {
    const currentFeet = this.body_state.feet as [number, number, number, number][]
    const targetFeet = this.layingDownPose

    if (this.layingDownPhase === 0) {
      return this.calculateZLowering(currentFeet, targetFeet)
    } else {
      return this.calculateLayingXYPositioning(currentFeet, targetFeet)
    }
  }

  calculateLayingXYPositioning(
    currentFeet: [number, number, number, number][],
    targetFeet: [number, number, number, number][]
  ): [number, number, number, number][] {
    const currentHeight = currentFeet[0][2]

    const xyTargetFeet = targetFeet.map(
      foot => [foot[0], foot[1], currentHeight, 1] as [number, number, number, number]
    )

    const distances = currentFeet.map((currentFoot, i) => {
      const dx = xyTargetFeet[i][0] - currentFoot[0]
      const dy = xyTargetFeet[i][1] - currentFoot[1]
      return Math.hypot(dx, dy)
    })

    const maxDistance = Math.max(...distances)

    if (maxDistance < 5) {
      return []
    }

    return xyTargetFeet
  }

  calculateZLowering(
    currentFeet: [number, number, number, number][],
    targetFeet: [number, number, number, number][]
  ): [number, number, number, number][] {
    const currentHeight = currentFeet[0][2]
    const targetHeight = Math.min(...targetFeet.map(f => f[2]))
    const heightDiff = targetHeight - currentHeight

    if (Math.abs(heightDiff) < 2) {
      this.layingDownPhase = 1
      return this.calculateLayingXYPositioning(currentFeet, targetFeet)
    }

    const maxHeightStep = 15
    const heightStep = Math.max(heightDiff, -maxHeightStep)
    const newHeight = currentHeight + heightStep

    const nextFeet = currentFeet.map(
      foot => [foot[0], foot[1], newHeight, 1] as [number, number, number, number]
    )

    return nextFeet
  }

  calculateNextStandingStep(): [number, number, number, number][] {
    const currentFeet = this.body_state.feet as [number, number, number, number][]
    const targetFeet = this.defaultPosition

    if (this.standingUpPhase === 0) {
      return this.calculateXYPositioning(currentFeet, targetFeet)
    } else {
      return this.calculateZLifting(currentFeet, targetFeet)
    }
  }

  calculateXYPositioning(
    currentFeet: [number, number, number, number][],
    targetFeet: [number, number, number, number][]
  ): [number, number, number, number][] {
    const currentHeight = currentFeet[0][2]

    const xyTargetFeet = targetFeet.map(
      foot => [foot[0], foot[1], currentHeight, 1] as [number, number, number, number]
    )

    const distances = currentFeet.map((currentFoot, i) => {
      const dx = xyTargetFeet[i][0] - currentFoot[0]
      const dy = xyTargetFeet[i][1] - currentFoot[1]
      return Math.hypot(dx, dy)
    })

    const maxDistance = Math.max(...distances)

    if (maxDistance < 5) {
      this.standingUpPhase = 1
      return this.calculateZLifting(currentFeet, targetFeet)
    }

    return xyTargetFeet
  }

  calculateZLifting(
    currentFeet: [number, number, number, number][],
    targetFeet: [number, number, number, number][]
  ): [number, number, number, number][] {
    const currentHeight = currentFeet[0][2]
    const targetHeight = targetFeet[0][2]
    const heightDiff = targetHeight - currentHeight

    if (Math.abs(heightDiff) < 2) {
      return []
    }

    const maxHeightStep = 15
    const heightStep = Math.min(heightDiff, maxHeightStep)
    const newHeight = currentHeight + heightStep

    const nextFeet = currentFeet.map(
      foot => [foot[0], foot[1], newHeight, 1] as [number, number, number, number]
    )

    return nextFeet
  }

  moveDirectlyToTarget(
    currentFeet: [number, number, number, number][],
    targetFeet: [number, number, number, number][]
  ): [number, number, number, number][] {
    const threshold = 2

    const distances = currentFeet.map((currentFoot, i) => {
      const dx = targetFeet[i][0] - currentFoot[0]
      const dy = targetFeet[i][1] - currentFoot[1]
      const dz = targetFeet[i][2] - currentFoot[2]
      return Math.hypot(dx, dy, dz)
    })

    const maxDistance = Math.max(...distances)

    if (maxDistance < threshold) {
      this.isMovingToTarget = false
      return targetFeet
    }

    const transitionSpeed = 0.15

    const newFeet = currentFeet.map((currentFoot, i) => {
      const target = targetFeet[i]
      const dx = target[0] - currentFoot[0]
      const dy = target[1] - currentFoot[1]
      const dz = target[2] - currentFoot[2]

      return [
        currentFoot[0] + dx * transitionSpeed,
        currentFoot[1] + dy * transitionSpeed,
        currentFoot[2] + dz * transitionSpeed,
        1
      ] as [number, number, number, number]
    })

    return newFeet
  }

  order(a: number[]) {
    return [3, 0, 4, 1, 5, 2].flatMap(i => a.slice(i * 3, i * 3 + 3))
  }
}
