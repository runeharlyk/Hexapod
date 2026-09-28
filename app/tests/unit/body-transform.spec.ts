import { describe, expect, it } from 'vitest'
import { get_transformation_matrix, multiplyVector } from '../../src/lib/math'
import type { body_state_t } from '../../src/lib/kinematic'

const bodyState = (pose: Pick<body_state_t, 'omega' | 'phi' | 'psi' | 'xm' | 'ym' | 'zm'>) => ({
  ...pose,
  feet: [],
  cumulative_x: 0,
  cumulative_y: 0,
  cumulative_z: 0,
  cumulative_roll: 0,
  cumulative_pitch: 0,
  cumulative_yaw: 0
})

// Applies Rz(psi), then Ry(phi), then Rx(omega) one axis at a time and adds the translation
// afterwards, which is what the firmware's Rx * Ry * Rz matrix with t in the last column does.
const firmwareTransform = (
  [x, y, z]: number[],
  {
    omega,
    phi,
    psi,
    xm,
    ym,
    zm
  }: { omega: number; phi: number; psi: number; xm: number; ym: number; zm: number }
) => {
  const x1 = Math.cos(psi) * x - Math.sin(psi) * y
  const y1 = Math.sin(psi) * x + Math.cos(psi) * y
  const z1 = z

  const x2 = Math.cos(phi) * x1 + Math.sin(phi) * z1
  const y2 = y1
  const z2 = -Math.sin(phi) * x1 + Math.cos(phi) * z1

  const x3 = x2
  const y3 = Math.cos(omega) * y2 - Math.sin(omega) * z2
  const z3 = Math.sin(omega) * y2 + Math.cos(omega) * z2

  return [x3 + xm, y3 + ym, z3 + zm]
}

describe('get_transformation_matrix', () => {
  const pose = { omega: 0.3, phi: -0.2, psi: 0.15, xm: 12, ym: -7, zm: -40 }
  const foot = [120, 80, -95, 1]

  it('rotates then translates a foot exactly like firmware kinematics.h', () => {
    const actual = multiplyVector(get_transformation_matrix(bodyState(pose)), foot)
    const expected = firmwareTransform(foot, pose)

    for (let i = 0; i < 3; i++) expect(actual[i]).toBeCloseTo(expected[i], 9)
    expect(actual[3]).toBe(1)
  })

  it('does not rotate the body translation', () => {
    const translated = { omega: 0.4, phi: 0.3, psi: 0, xm: 25, ym: 0, zm: -60 }
    const origin = multiplyVector(get_transformation_matrix(bodyState(translated)), [0, 0, 0, 1])

    expect(origin.slice(0, 3)).toEqual([25, 0, -60])
  })
})
