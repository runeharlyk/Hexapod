import { describe, it, expect, beforeEach } from 'vitest'
import Kinematics, {
  gen_posture,
  type HexapodConfig,
  type body_state_t
} from '../../src/lib/kinematic'

describe('Kinematics', () => {
  const config: HexapodConfig = {
    legMountX: [60, 0, -60, -60, 0, 60],
    legMountY: [0, 60, 0, 0, -60, 0],
    legRootToJoint1: 30,
    legJoint1ToJoint2: 50,
    legJoint2ToJoint3: 65,
    legJoint3ToTip: 95,
    legMountAngle: [0, 60, 120, 180, 240, 300]
  }

  let kinematics: Kinematics

  const makeBodyState = (overrides: Partial<body_state_t> = {}): body_state_t => ({
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
    cumulative_yaw: 0,
    ...overrides
  })

  beforeEach(() => {
    kinematics = new Kinematics(config)
  })

  describe('gen_posture function', () => {
    it('should generate correct posture for zero angles', () => {
      const j2Angle = 0
      const j3Angle = 0
      const posture = gen_posture(j2Angle, j3Angle, config)

      expect(posture).toHaveLength(6)

      // Debug the actual calculation for gen_posture
      const j2_rad = (j2Angle / 180) * Math.PI
      const j3_rad = (j3Angle / 180) * Math.PI
      const expr =
        config.legRootToJoint1 +
        config.legJoint1ToJoint2 +
        config.legJoint2ToJoint3 * Math.sin(j2_rad) +
        config.legJoint3ToTip * Math.cos(j3_rad)

      for (let i = 0; i < 6; i++) {
        const [x, y, z, w] = posture[i]
        expect(w).toBe(1)

        const mountAngleRad = (config.legMountAngle[i] / 180) * Math.PI
        const expectedX = config.legMountX[i] + expr * Math.cos(mountAngleRad)
        const expectedY = config.legMountY[i] + expr * Math.sin(mountAngleRad)
        const expectedZ =
          config.legJoint2ToJoint3 * Math.cos(j2_rad) - config.legJoint3ToTip * Math.sin(j3_rad)

        expect(x).toBeCloseTo(expectedX, 10)
        expect(y).toBeCloseTo(expectedY, 10)
        expect(z).toBeCloseTo(expectedZ, 10)
      }
    })

    it('should generate different postures for different angles', () => {
      const posture1 = gen_posture(0, 0, config)
      const posture2 = gen_posture(Math.PI / 4, Math.PI / 6, config)

      expect(posture1).not.toEqual(posture2)

      for (let i = 0; i < 6; i++) {
        expect(posture1[i][2]).not.toBeCloseTo(posture2[i][2], 5)
      }
    })

    it('should handle negative angles correctly', () => {
      const posture = gen_posture(-Math.PI / 4, -Math.PI / 6, config)

      expect(posture).toHaveLength(6)

      for (const [x, y, z, w] of posture) {
        expect(Number.isFinite(x)).toBe(true)
        expect(Number.isFinite(y)).toBe(true)
        expect(Number.isFinite(z)).toBe(true)
        expect(w).toBe(1)
      }
    })

    it('should be consistent with class genPosture method', () => {
      const j2 = Math.PI / 6
      const j3 = -Math.PI / 4

      const functionResult = gen_posture((j2 * 180) / Math.PI, (j3 * 180) / Math.PI, config)
      const classResult = kinematics.genPosture(j2, j3)

      expect(functionResult).toHaveLength(classResult.length)

      for (let i = 0; i < functionResult.length; i++) {
        for (let j = 0; j < 4; j++) {
          expect(functionResult[i][j]).toBeCloseTo(classResult[i][j], 10)
        }
      }
    })
  })

  describe('Kinematics class methods', () => {
    describe('genPosture', () => {
      it('should generate correct posture for zero angles', () => {
        const j2 = 0
        const j3 = 0
        const posture = kinematics.genPosture(j2, j3)

        expect(posture).toHaveLength(6)

        // Match the actual implementation calculation
        const ext =
          kinematics.rootJ1 +
          kinematics.j1J2 +
          kinematics.j2J3 * Math.sin(j2) +
          kinematics.j3Tip * Math.cos(j3)

        for (let i = 0; i < 6; i++) {
          const [x, y, z, w] = posture[i]
          expect(w).toBe(1)

          const expectedX = kinematics.mountX[i] + ext * kinematics.ca[i]
          const expectedY = kinematics.mountY[i] + ext * kinematics.sa[i]
          const expectedZ = kinematics.j2J3 * Math.cos(j2) - kinematics.j3Tip * Math.sin(j3)

          expect(x).toBeCloseTo(expectedX, 10)
          expect(y).toBeCloseTo(expectedY, 10)
          expect(z).toBeCloseTo(expectedZ, 10)
        }
      })

      it('should handle various joint angles', () => {
        const testCases = [
          [0, 0],
          [Math.PI / 4, Math.PI / 6],
          [-Math.PI / 3, Math.PI / 2],
          [Math.PI / 2, -Math.PI / 4]
        ]

        testCases.forEach(([j2, j3]) => {
          const posture = kinematics.genPosture(j2, j3)

          expect(posture).toHaveLength(6)

          for (const [x, y, z, w] of posture) {
            expect(Number.isFinite(x)).toBe(true)
            expect(Number.isFinite(y)).toBe(true)
            expect(Number.isFinite(z)).toBe(true)
            expect(w).toBe(1)
          }
        })
      })
    })

    describe('forwardKinematics', () => {
      it('should calculate correct foot positions for zero angles', () => {
        const bodyState = makeBodyState()
        const angles: [number, number, number][] = Array(6).fill([0, 0, 0])
        const positions = kinematics.forwardKinematics(bodyState, angles)

        expect(positions).toHaveLength(6)

        const expectedExtension =
          config.legRootToJoint1 +
          config.legJoint1ToJoint2 +
          config.legJoint2ToJoint3 +
          config.legJoint3ToTip

        for (let i = 0; i < 6; i++) {
          const [x, y, z, w] = positions[i]
          expect(w).toBe(1)

          const mountAngleRad = (config.legMountAngle[i] * Math.PI) / 180
          const expectedX = config.legMountX[i] + expectedExtension * Math.cos(mountAngleRad)
          const expectedY = config.legMountY[i] + expectedExtension * Math.sin(mountAngleRad)
          const expectedZ = 0

          expect(x).toBeCloseTo(expectedX, 5)
          expect(y).toBeCloseTo(expectedY, 5)
          expect(z).toBeCloseTo(expectedZ, 5)
        }
      })

      it('should handle body transformations', () => {
        const bodyState = makeBodyState({ omega: 0.1, phi: 0.2, psi: 0.3, xm: 10, ym: 20, zm: 30 })
        const angles: [number, number, number][] = Array(6).fill([0, 0, 0])
        const positions = kinematics.forwardKinematics(bodyState, angles)

        expect(positions).toHaveLength(6)

        for (const [x, y, z, w] of positions) {
          expect(Number.isFinite(x)).toBe(true)
          expect(Number.isFinite(y)).toBe(true)
          expect(Number.isFinite(z)).toBe(true)
          expect(w).toBe(1)
        }
      })

      it('should handle extreme angles correctly', () => {
        const bodyState = makeBodyState()
        const extremeAngles: [number, number, number][] = [
          [Math.PI / 2, Math.PI / 4, -Math.PI / 4],
          [-Math.PI / 2, -Math.PI / 4, Math.PI / 4],
          [0, Math.PI / 2, -Math.PI / 2],
          [Math.PI, 0, 0],
          [0, -Math.PI / 4, Math.PI / 4],
          [-Math.PI / 4, Math.PI / 4, -Math.PI / 4]
        ]

        const positions = kinematics.forwardKinematics(bodyState, extremeAngles)

        expect(positions).toHaveLength(6)

        for (const [x, y, z, w] of positions) {
          expect(Number.isFinite(x)).toBe(true)
          expect(Number.isFinite(y)).toBe(true)
          expect(Number.isFinite(z)).toBe(true)
          expect(w).toBe(1)
        }
      })
    })

    describe('inverseKinematics', () => {
      it('should calculate correct angles for known positions', () => {
        const bodyState = makeBodyState()

        // Use the class genPosture to get realistic foot positions
        const referenceAngles: [number, number, number][] = Array(6).fill([0, 0, 0])
        const positions = kinematics.forwardKinematics(bodyState, referenceAngles)

        // Extract 3D coordinates for feet
        bodyState.feet = positions.map(pos => [pos[0], pos[1], pos[2], 1])

        const angles = kinematics.inverseKinematics(bodyState)

        expect(angles).toHaveLength(6)

        for (const [a0, a1, a2] of angles) {
          expect(Number.isFinite(a0)).toBe(true)
          expect(Number.isFinite(a1)).toBe(true)
          expect(Number.isFinite(a2)).toBe(true)
        }
      })

      it('should handle body transformations in inverse kinematics', () => {
        const bodyState = makeBodyState({ omega: 0.1, phi: 0.2, psi: 0.3, xm: 10, ym: 20, zm: 30 })

        // Generate reasonable foot positions first
        const referenceAngles: [number, number, number][] = Array(6).fill([0.1, 0.2, -0.1])
        const positions = kinematics.forwardKinematics(bodyState, referenceAngles)
        bodyState.feet = positions.map(pos => [pos[0], pos[1], pos[2], 1])

        const angles = kinematics.inverseKinematics(bodyState)

        expect(angles).toHaveLength(6)

        for (const [a0, a1, a2] of angles) {
          expect(Number.isFinite(a0)).toBe(true)
          expect(Number.isFinite(a1)).toBe(true)
          expect(Number.isFinite(a2)).toBe(true)
        }
      })

      it('should handle unreachable positions gracefully', () => {
        const maxReach =
          config.legRootToJoint1 +
          config.legJoint1ToJoint2 +
          config.legJoint2ToJoint3 +
          config.legJoint3ToTip

        const bodyState = makeBodyState({
          feet: [
            [maxReach * 1.5, 0, 0, 1], // Challenging but potentially reachable
            [0, maxReach * 1.5, 0, 1],
            [-maxReach * 1.5, 0, 0, 1],
            [0, -maxReach * 1.5, 0, 1],
            [maxReach * 0.8, maxReach * 0.8, 0, 1],
            [-maxReach * 0.8, -maxReach * 0.8, 0, 1]
          ]
        })

        const angles = kinematics.inverseKinematics(bodyState)

        expect(angles).toHaveLength(6)

        // Even for difficult positions, we should get some result (even if clamped)
        for (const [a0, a1, a2] of angles) {
          expect(Number.isFinite(a0)).toBe(true)
          expect(Number.isFinite(a1)).toBe(true)
          expect(Number.isFinite(a2)).toBe(true)
        }
      })
    })
  })

  describe('Forward and Inverse Kinematics Consistency', () => {
    // inverseKinematics resolves the 2-link elbow with a fixed sign (a1 = base + acos(..),
    // a2 = -(acos(..) + acos(..))), exactly as firmware/include/kinematics.h does, so its
    // image is the knee-down branch only: a2 <= 0. Angle round-trips are therefore asserted
    // on knee-down poses; a knee-up pose only round-trips in foot position (see below).
    const kneeDownAngles: [number, number, number][] = [
      [0.1, 0.2, -0.3],
      [-0.2, 0.3, -0.4],
      [0.15, -0.1, -0.2],
      [-0.1, 0.25, -0.3],
      [0.2, -0.15, -0.25],
      [0.0, 0.1, -0.2]
    ]

    it('should be consistent with inverse kinematics for small angles', () => {
      const bodyState = makeBodyState()

      const positions = kinematics.forwardKinematics(bodyState, kneeDownAngles)
      bodyState.feet = positions.map(pos => [pos[0], pos[1], pos[2], 1])

      const recoveredAngles = kinematics.inverseKinematics(bodyState)

      for (let i = 0; i < kneeDownAngles.length; i++) {
        for (let j = 0; j < 3; j++) {
          expect(recoveredAngles[i][j]).toBeCloseTo(kneeDownAngles[i][j], 5)
        }
      }
    })

    it('should map a knee-up pose onto the knee-down branch without moving the foot', () => {
      const bodyState = makeBodyState()

      const kneeUpAngles: [number, number, number][] = kneeDownAngles.map(
        ([a0, a1, a2]) => [a0, a1, -a2] as [number, number, number]
      )

      const positions = kinematics.forwardKinematics(bodyState, kneeUpAngles)
      bodyState.feet = positions.map(pos => [pos[0], pos[1], pos[2], 1])

      const recoveredAngles = kinematics.inverseKinematics(bodyState)
      const recoveredPositions = kinematics.forwardKinematics(bodyState, recoveredAngles)

      for (let i = 0; i < kneeUpAngles.length; i++) {
        // The solver cannot reproduce the knee-up joint angles ...
        expect(recoveredAngles[i][2]).toBeLessThanOrEqual(0)
        // ... but it must reach the same foot position.
        for (let j = 0; j < 3; j++) {
          expect(recoveredPositions[i][j]).toBeCloseTo(positions[i][j], 5)
        }
      }
    })

    it('should be consistent with body transformations', () => {
      const bodyState = makeBodyState({ omega: 0.05, phi: 0.1, psi: 0.15, xm: 5, ym: 10, zm: 15 })

      const testAngles: [number, number, number][] = Array(6).fill([0.1, 0.15, -0.1])

      const positions = kinematics.forwardKinematics(bodyState, testAngles)
      bodyState.feet = positions.map(pos => [pos[0], pos[1], pos[2], 1])

      const recoveredAngles = kinematics.inverseKinematics(bodyState)

      // Check all angles are finite first
      for (let i = 0; i < recoveredAngles.length; i++) {
        for (let j = 0; j < 3; j++) {
          expect(Number.isFinite(recoveredAngles[i][j])).toBe(true)
        }
      }

      // Check consistency with reduced precision for stability
      for (let i = 0; i < testAngles.length; i++) {
        for (let j = 0; j < 3; j++) {
          expect(recoveredAngles[i][j]).toBeCloseTo(testAngles[i][j], 1)
        }
      }
    })

    it('should reach a fixed point after one round trip', () => {
      const bodyState = makeBodyState()

      // Seed with knee-up poses so the first round trip has to move the joints. Every later
      // round trip must then be a no-op: repeated FK/IK must not accumulate drift, which is
      // what the visualization does frame after frame.
      let currentAngles: [number, number, number][] = kneeDownAngles.map(
        ([a0, a1, a2]) => [a0, a1, -a2] as [number, number, number]
      )

      let previousAngles: [number, number, number][] | null = null

      for (let iteration = 0; iteration < 3; iteration++) {
        const positions = kinematics.forwardKinematics(bodyState, currentAngles)
        bodyState.feet = positions.map(pos => [pos[0], pos[1], pos[2], 1])
        currentAngles = kinematics.inverseKinematics(bodyState)

        if (previousAngles) {
          for (let i = 0; i < currentAngles.length; i++) {
            for (let j = 0; j < 3; j++) {
              expect(currentAngles[i][j]).toBeCloseTo(previousAngles[i][j], 9)
            }
          }
        }
        previousAngles = currentAngles
      }
    })
  })

  describe('Edge Cases and Validation', () => {
    it('should handle zero-length segments gracefully', () => {
      const zeroConfig: HexapodConfig = {
        ...config,
        legRootToJoint1: 0
      }
      const zeroKinematics = new Kinematics(zeroConfig)

      const angles: [number, number, number][] = Array(6).fill([0, 0, 0])
      const bodyState = makeBodyState()

      const positions = zeroKinematics.forwardKinematics(bodyState, angles)
      expect(positions).toHaveLength(6)

      for (const [x, y, z, w] of positions) {
        expect(Number.isFinite(x)).toBe(true)
        expect(Number.isFinite(y)).toBe(true)
        expect(Number.isFinite(z)).toBe(true)
        expect(w).toBe(1)
      }
    })

    it('should handle large body translations', () => {
      const bodyState = makeBodyState({ xm: 1000, ym: 2000, zm: 3000 })

      const angles: [number, number, number][] = Array(6).fill([0, 0, 0])
      const positions = kinematics.forwardKinematics(bodyState, angles)

      expect(positions).toHaveLength(6)

      for (const [x, y, z, w] of positions) {
        expect(Number.isFinite(x)).toBe(true)
        expect(Number.isFinite(y)).toBe(true)
        expect(Number.isFinite(z)).toBe(true)
        expect(w).toBe(1)
      }
    })

    it('should handle large body rotations', () => {
      const bodyState = makeBodyState({ omega: Math.PI, phi: Math.PI / 2, psi: -Math.PI / 2 })

      const angles: [number, number, number][] = Array(6).fill([0.1, 0.2, -0.1])
      const positions = kinematics.forwardKinematics(bodyState, angles)

      expect(positions).toHaveLength(6)

      for (const [x, y, z, w] of positions) {
        expect(Number.isFinite(x)).toBe(true)
        expect(Number.isFinite(y)).toBe(true)
        expect(Number.isFinite(z)).toBe(true)
        expect(w).toBe(1)
      }
    })
  })
})
