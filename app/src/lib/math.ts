import type { body_state_t } from './kinematic';

type Matrix = number[][];

export const rot_x = (theta: number): Matrix => {
    const c = Math.cos(theta),
        s = Math.sin(theta);
    return [
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ];
};

export const rot_y = (theta: number): Matrix => {
    const c = Math.cos(theta),
        s = Math.sin(theta);
    return [
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]
    ];
};

export const rot_z = (theta: number): Matrix => {
    const c = Math.cos(theta),
        s = Math.sin(theta);
    return [
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ];
};

export const rot = (omega: number, phi: number, psi: number): Matrix => {
    return matrixMultiply(rot_z(psi), matrixMultiply(rot_y(phi), rot_x(omega)));
};

export const translation = (x: number, y: number, z: number): Matrix => {
    return [
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ];
};

export const transformation = (
    omega: number,
    phi: number,
    psi: number,
    x: number,
    y: number,
    z: number
): Matrix => {
    const T: Matrix = translation(x, y, z);
    const R: Matrix = rot(omega, phi, psi);
    return matrixMultiply(R, T);
};

export const get_transformation_matrix = (body_state: body_state_t): Matrix => {
    const omega = body_state.omega;
    const phi = body_state.phi;
    const psi = body_state.psi;
    const xm = body_state.xm;
    const ym = body_state.ym;
    const zm = body_state.zm;
    return transformation(omega, phi, psi, xm, ym, zm);
};

export const matrixMultiply = (a: Matrix, b: Matrix): Matrix => {
    const result: Matrix = [];
    for (let i = 0; i < a.length; i++) {
        result[i] = [];
        for (let j = 0; j < b[0].length; j++) {
            let sum = 0;
            for (let k = 0; k < a[i].length; k++) {
                sum += a[i][k] * b[k][j];
            }
            result[i][j] = sum;
        }
    }
    return result;
};

export const multiplyVector = (matrix: Matrix, vector: number[]): number[] => {
    const result: number[] = [];
    for (let i = 0; i < matrix.length; i++) {
        let sum = 0;
        for (let j = 0; j < matrix[0].length; j++) {
            sum += matrix[i][j] * vector[j];
        }
        result[i] = sum;
    }
    return result;
};

export const transpose = (matrix: Matrix): Matrix => {
    const rows = matrix.length;
    const cols = matrix[0].length;
    const transposed: Matrix = [];
    for (let j = 0; j < cols; j++) {
        transposed[j] = [];
        for (let i = 0; i < rows; i++) {
            transposed[j][i] = matrix[i][j];
        }
    }
    return transposed;
};

export const adjugate = (matrix: Matrix): Matrix => {
    const n = matrix.length;
    const adjugate: Matrix = [];
    for (let i = 0; i < n; i++) {
        adjugate[i] = [];
        for (let j = 0; j < n; j++) {
            const subMatrix = matrix
                .slice(0, i)
                .concat(matrix.slice(i + 1))
                .map(row => row.slice(0, j).concat(row.slice(j + 1)));
            const cofactor = (i + j) % 2 === 0 ? 1 : -1 * determinant(subMatrix);
            adjugate[i][j] = cofactor;
        }
    }
    return transpose(adjugate);
};

export const inverse = (matrix: Matrix): Matrix => {
    const det = determinant(matrix);
    if (det === 0) {
        throw new Error('Matrix is singular and has no inverse.');
    }
    const _adjugate = adjugate(matrix);
    const scalar = 1 / det;
    const inverse: Matrix = [];
    for (let i = 0; i < matrix.length; i++) {
        inverse[i] = [];
        for (let j = 0; j < matrix[i].length; j++) {
            inverse[i][j] = _adjugate[i][j] * scalar;
        }
    }
    return inverse;
};

export const determinant = (matrix: Matrix): number => {
    const n = matrix.length;
    if (n === 1) {
        return matrix[0][0];
    } else if (n === 2) {
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    } else {
        let det = 0;
        for (let i = 0; i < n; i++) {
            const subMatrix = matrix.slice(1).map(row => row.slice(0, i).concat(row.slice(i + 1)));
            det += (i % 2 === 0 ? 1 : -1) * matrix[0][i] * determinant(subMatrix);
        }
        return det;
    }
};
