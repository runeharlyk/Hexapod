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
  RANDOM_POSE = 'random_pose',
  CONSTRAINED_RANDOM = 'constrained_random',
  LAYING_DOWN = 'laying_down'
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
  layingDownPose: number[] = []

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
    this.layingDownPose = this.generateLayingDownPose()
  }

  setMode(mode: MotionModes) {
    this.mode = mode
    if (mode === MotionModes.RANDOM_POSE) {
      this.lastPoseGeneration = 0
      this.generateRandomPose()
    } else if (mode === MotionModes.CONSTRAINED_RANDOM) {
      this.lastPoseGeneration = 0
      this.constrainedPoseIndex = 0
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
      case MotionModes.RANDOM_POSE: {
        const now = performance.now()
        if (now - this.lastPoseGeneration > 1000) {
          this.generateRandomPose()
          this.lastPoseGeneration = now
        }
        this.targetAngles = this.poses[this.current_pose_idx]
        break
      }
      case MotionModes.CONSTRAINED_RANDOM: {
        const now = performance.now()
        if (now - this.lastPoseGeneration > 5000) {
          this.generateConstrainedRandomPose()
          this.lastPoseGeneration = now
        }
        this.targetAngles = this.constrainedPoses[this.constrainedPoseIndex]
        break
      }
    }
    return true
  }

  generateRandomPose() {
    const randomAngles: [number, number, number][] = []

    for (let i = 0; i < 6; i++) {
      const a0 = (Math.random() - 0.5) * (Math.PI / 3)
      const a1 = Math.random() * (Math.PI / 2) + Math.PI / 6
      const a2 = -(Math.random() * (Math.PI / 3) + Math.PI / 6)
      randomAngles.push([a0, a1, a2])
    }

    const neutralBodyState = {
      omega: 0,
      phi: 0,
      psi: 0,
      xm: 0,
      ym: 0,
      zm: 0,
      feet: this.defaultPosition
    }

    const feet = this.kinematics.forwardKinematics(neutralBodyState, randomAngles)

    this.body_state.feet = feet

    const poseAngles = this.kinematics.inverseKinematics(this.body_state)
    this.poses[this.current_pose_idx] = this.order(poseAngles.flat())
  }

  initializeConstrainedPoses() {
    this.constrainedPoses = [
      this.generateLayingDownPose(),
      this.generateStandingPose(),
      new Array(18).fill(0)
    ]
  }

  generateLayingDownPose(): number[] {
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

    const feet = this.kinematics.forwardKinematics(neutralBodyState, zeroAngles)

    const tempBodyState = { ...this.body_state }
    tempBodyState.feet = feet

    const poseAngles = this.kinematics.inverseKinematics(tempBodyState)
    return this.order(poseAngles.flat())
  }

  generateStandingPose(): number[] {
    const tempBodyState = { ...this.body_state }
    tempBodyState.feet = this.defaultPosition

    const poseAngles = this.kinematics.inverseKinematics(tempBodyState)
    return this.order(poseAngles.flat())
  }

  generateConstrainedRandomPose() {
    if (this.constrainedPoseIndex < 2) {
      this.constrainedPoseIndex++
      return
    }

    const constrainedFeet = this.defaultPosition.map((defaultFoot, i) => {
      const randomX = defaultFoot[0] + (Math.random() - 0.5) * 100
      const randomY = defaultFoot[1] + (Math.random() - 0.5) * 80

      return [randomX, randomY, defaultFoot[2], 1]
    })

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
    this.constrainedPoses[2] = this.order(poseAngles.flat())

    this.constrainedPoseIndex = (this.constrainedPoseIndex + 1) % 3
  }

  order(a: number[]) {
    return [3, 0, 4, 1, 5, 2].flatMap(i => a.slice(i * 3, i * 3 + 3))
  }
}
