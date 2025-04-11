export interface body_state_t {
    omega: number;
    phi: number;
    psi: number;
    xm: number;
    ym: number;
    zm: number;
    feet: number[][];
}

export interface position {
    x: number;
    y: number;
    z: number;
}

export interface target_position {
    x: number;
    z: number;
    yaw: number;
}

const { cos, sin, atan2, sqrt } = Math;

const DEG2RAD = 0.017453292519943;

export interface HexapodConfig {
    legMountX: number[];
    legMountY: number[];
    legRootToJoint1: number;
    legJoint1ToJoint2: number;
    legJoint2ToJoint3: number;
    legJoint3ToTip: number;
    legMountAngle: number[];
    legScale: number[][];
}

export function computeDefaultPosition(config: HexapodConfig): number[][] {
    const footPositions: number[][] = Array(6)
        .fill(0)
        .map(() => Array(3).fill(0));

    const l1 = config.legRootToJoint1;
    const l2 = config.legJoint1ToJoint2;
    const l3 = config.legJoint2ToJoint3;
    const l4 = config.legJoint3ToTip;

    const forward = l2 + l3 * sin(30) + l4 * sin(15);
    const down = l3 * cos(30) + l4 * cos(15);

    const spreadScale = 1;
    const localTip: number[] = [(l1 + forward) * spreadScale, 0, -down];

    for (let i = 0; i < 6; i++) {
        const angle = (config.legMountAngle[i] * Math.PI) / 180;
        const rot = [
            [Math.cos(angle), -Math.sin(angle), 0],
            [Math.sin(angle), Math.cos(angle), 0],
            [0, 0, 1]
        ];

        // Matrix multiplication (rot @ local_tip)
        const rotated = [
            rot[0][0] * localTip[0] + rot[0][1] * localTip[1] + rot[0][2] * localTip[2],
            rot[1][0] * localTip[0] + rot[1][1] * localTip[1] + rot[1][2] * localTip[2],
            rot[2][0] * localTip[0] + rot[2][1] * localTip[1] + rot[2][2] * localTip[2]
        ];

        footPositions[i] = [
            config.legMountX[i] + rotated[0],
            config.legMountY[i] + rotated[1],
            rotated[2]
        ];
    }

    return footPositions;
}

export function gen_posture(j2_angle: number, j3_angle: number, config: HexapodConfig): number[][] {
    const mountX = config.legMountX;
    const mountY = config.legMountY;
    const rootJ1 = config.legRootToJoint1;
    const j1_j2 = config.legJoint1ToJoint2;
    const j2_j3 = config.legJoint2ToJoint3;
    const j3_tip = config.legJoint3ToTip;
    const mountAngle = config.legMountAngle.map(a => (a / 180) * Math.PI);
    const j2_rad = (j2_angle / 180) * Math.PI;
    const j3_rad = (j3_angle / 180) * Math.PI;
    const expr = rootJ1 + j1_j2 + j2_j3 * Math.sin(j2_rad) + j3_tip * Math.cos(j3_rad);
    const posture: number[][] = [];
    for (let i = 0; i < 6; i++) {
        posture.push([
            mountX[i] + expr * Math.cos(mountAngle[i]),
            mountY[i] + expr * Math.sin(mountAngle[i]),
            j2_j3 * Math.cos(j2_rad) - j3_tip * Math.sin(j3_rad)
        ]);
    }
    return posture;
}

export default class Kinematics {
    l1: number; // legRootToJoint1
    l2: number; // legJoint1ToJoint2
    l3: number; // legJoint2ToJoint3
    l4: number; // legJoint3ToTip

    legMountX: number[];
    legMountY: number[];
    legMountAngle: number[];

    DEG2RAD = DEG2RAD;

    // Transformation matrices for the hip mounting points relative to the body center
    T_hip: number[][][] = [];

    constructor(config: HexapodConfig) {
        this.l1 = config.legRootToJoint1 / 100; // Convert to meters if necessary
        this.l2 = config.legJoint1ToJoint2 / 100;
        this.l3 = config.legJoint2ToJoint3 / 100;
        this.l4 = config.legJoint3ToTip / 100;

        this.legMountX = config.legMountX.map((val: number) => val / 100); // Convert to meters
        this.legMountY = config.legMountY.map((val: number) => val / 100);
        this.legMountAngle = config.legMountAngle;

        for (let i = 0; i < 6; i++) {
            const angleRad = this.legMountAngle[i] * this.DEG2RAD;
            this.T_hip[i] = [
                [cos(angleRad), -sin(angleRad), 0, this.legMountX[i]],
                [sin(angleRad), cos(angleRad), 0, this.legMountY[i]],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ];
        }
    }

    public calcIK(body_state: body_state_t): number[] {
        this.bodyIK(body_state);

        const jointAngles: number[] = [];
        for (let i = 0; i < 6; i++) {
            const footInBodyFrame = body_state.feet[i];
            // Transform the foot position from the body frame to the hip frame
            const hipFrameInverse = this.inverse(this.T_hip[i]);
            const footInHipFrame = this.multiplyVector(hipFrameInverse, [
                ...footInBodyFrame,
                1
            ]).slice(0, 3);
            jointAngles.push(...this.legIK(footInHipFrame));
        }
        return jointAngles;
    }

    bodyIK(p: body_state_t) {
        const cos_omega = cos(p.omega * this.DEG2RAD);
        const sin_omega = sin(p.omega * this.DEG2RAD);
        const cos_phi = cos(p.phi * this.DEG2RAD);
        const sin_phi = sin(p.phi * this.DEG2RAD);
        const cos_psi = cos(p.psi * this.DEG2RAD);
        const sin_psi = sin(p.psi * this.DEG2RAD);

        const Tm: number[][] = [
            [cos_phi * cos_psi, -sin_psi * cos_phi, sin_phi, p.xm],
            [
                sin_omega * sin_phi * cos_psi + sin_psi * cos_omega,
                -sin_omega * sin_phi * sin_psi + cos_omega * cos_psi,
                -sin_omega * cos_phi,
                p.ym
            ],
            [
                sin_omega * sin_psi - sin_phi * cos_omega * cos_psi,
                sin_omega * cos_psi + sin_phi * sin_psi * cos_omega,
                cos_omega * cos_phi,
                p.zm
            ],
            [0, 0, 0, 1]
        ];

        // Update T_hip to include body orientation (translation is already in constructor)
        for (let i = 0; i < 6; i++) {
            const hipOffset = [
                this.T_hip[i][0][3],
                this.T_hip[i][1][3],
                this.T_hip[i][2][3],
                this.T_hip[i][3][3]
            ];
            const rotatedOffset = this.multiplyVector(Tm, hipOffset).slice(0, 3);
            const angleRad = this.legMountAngle[i] * this.DEG2RAD;
            this.T_hip[i] = [
                [
                    cos(angleRad) * Tm[0][0] + sin(angleRad) * Tm[1][0],
                    cos(angleRad) * Tm[0][1] + sin(angleRad) * Tm[1][1],
                    cos(angleRad) * Tm[0][2] + sin(angleRad) * Tm[1][2],
                    rotatedOffset[0]
                ],
                [
                    -sin(angleRad) * Tm[0][0] + cos(angleRad) * Tm[1][0],
                    -sin(angleRad) * Tm[0][1] + cos(angleRad) * Tm[1][1],
                    -sin(angleRad) * Tm[0][2] + cos(angleRad) * Tm[1][2],
                    rotatedOffset[1]
                ],
                [Tm[2][0], Tm[2][1], Tm[2][2], rotatedOffset[2]],
                [0, 0, 0, 1]
            ];
        }
    }

    public legIK(point: number[]): number[] {
        const [x, y, z] = point;

        // Assuming the first joint rotates around Z, the next two around Y in their local frames
        const theta1 = atan2(y, x); // Hip/Coxa angle

        const effectiveX = sqrt(x ** 2 + y ** 2) - this.l1 * cos(theta1);
        const targetDistSq = effectiveX ** 2 + z ** 2;
        const targetDist = sqrt(targetDistSq);

        const cosTheta3Num = targetDistSq - this.l2 ** 2 - this.l3 ** 2;
        const cosTheta3Den = 2 * this.l2 * this.l3;
        const cosTheta3 = cosTheta3Num / cosTheta3Den;

        // Clamp cosTheta3 to avoid NaN from acos
        const clampedCosTheta3 = Math.max(-1, Math.min(1, cosTheta3));
        const theta3 = atan2(-sqrt(1 - clampedCosTheta3 ** 2), clampedCosTheta3); // Knee/Femur angle (assuming downward bend is negative)

        const theta2 =
            atan2(z, effectiveX) - atan2(this.l3 * sin(theta3), this.l2 + this.l3 * cos(theta3)); // Ankle/Tibia angle

        return [theta1, theta2, theta3];
    }

    matrixMultiply(a: number[][], b: number[][]): number[][] {
        const result: number[][] = [];
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
    }

    multiplyVector(matrix: number[][], vector: number[]): number[] {
        const result: number[] = [];
        for (let i = 0; i < matrix.length; i++) {
            let sum = 0;
            for (let j = 0; j < matrix[0].length; j++) {
                sum += matrix[i][j] * vector[j];
            }
            result[i] = sum;
        }
        return result;
    }

    private inverse(matrix: number[][]): number[][] {
        const det = this.determinant(matrix);
        if (det === 0) {
            throw new Error('Matrix is singular and has no inverse.');
        }
        const adjugate = this.adjugate(matrix);
        const scalar = 1 / det;
        const inverse: number[][] = [];
        for (let i = 0; i < matrix.length; i++) {
            inverse[i] = [];
            for (let j = 0; j < matrix[i].length; j++) {
                inverse[i][j] = adjugate[i][j] * scalar;
            }
        }
        return inverse;
    }

    private determinant(matrix: number[][]): number {
        const n = matrix.length;
        if (n === 1) {
            return matrix[0][0];
        } else if (n === 2) {
            return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        } else {
            let det = 0;
            for (let i = 0; i < n; i++) {
                const subMatrix = matrix
                    .slice(1)
                    .map(row => row.slice(0, i).concat(row.slice(i + 1)));
                det += (i % 2 === 0 ? 1 : -1) * matrix[0][i] * this.determinant(subMatrix);
            }
            return det;
        }
    }

    private adjugate(matrix: number[][]): number[][] {
        const n = matrix.length;
        const adjugate: number[][] = [];
        for (let i = 0; i < n; i++) {
            adjugate[i] = [];
            for (let j = 0; j < n; j++) {
                const subMatrix = matrix
                    .slice(0, i)
                    .concat(matrix.slice(i + 1))
                    .map(row => row.slice(0, j).concat(row.slice(j + 1)));
                const cofactor = (i + j) % 2 === 0 ? 1 : -1 * this.determinant(subMatrix);
                adjugate[i][j] = cofactor;
            }
        }
        return this.transpose(adjugate);
    }

    private transpose(matrix: number[][]): number[][] {
        const rows = matrix.length;
        const cols = matrix[0].length;
        const transposed: number[][] = [];
        for (let j = 0; j < cols; j++) {
            transposed[j] = [];
            for (let i = 0; i < rows; i++) {
                transposed[j][i] = matrix[i][j];
            }
        }
        return transposed;
    }
}
