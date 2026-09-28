import type { body_state_t } from './kinematic'

export type Matrix = number[][]

// Mirrors get_transformation_matrix in firmware/include/kinematics.h:
// R = Rx(omega) * Ry(phi) * Rz(psi) with the body translation in the last column, so T * p = R * p + t.
export const get_transformation_matrix = ({
  omega,
  phi,
  psi,
  xm,
  ym,
  zm
}: body_state_t): Matrix => {
  const co = Math.cos(omega),
    so = Math.sin(omega)
  const cp = Math.cos(phi),
    sp = Math.sin(phi)
  const cs = Math.cos(psi),
    ss = Math.sin(psi)
  return [
    [cp * cs, -cp * ss, sp, xm],
    [so * sp * cs + ss * co, -so * sp * ss + co * cs, -so * cp, ym],
    [so * ss - sp * co * cs, so * cs + sp * ss * co, co * cp, zm],
    [0, 0, 0, 1]
  ]
}

export const multiplyVector = (matrix: Matrix, vector: number[]): number[] =>
  matrix.map(row => row.reduce((sum, value, j) => sum + value * vector[j], 0))
