import { inverse, matrixMultiply, multiplyVector, rot_translation } from './math';

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
            j2_j3 * Math.cos(j2_rad) - j3_tip * Math.sin(j3_rad),
            1
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
        this.l1 = config.legRootToJoint1; // Convert to meters if necessary
        this.l2 = config.legJoint1ToJoint2;
        this.l3 = config.legJoint2ToJoint3;
        this.l4 = config.legJoint3ToTip;

        this.legMountX = config.legMountX.map((val: number) => val); // Convert to meters
        this.legMountY = config.legMountY.map((val: number) => val);
        this.legMountAngle = config.legMountAngle.map(a => (a / 180) * Math.PI); // Convert to radians

        for (let i = 0; i < 6; i++) {
            const angleRad = this.legMountAngle[i];
            this.T_hip[i] = [
                [cos(angleRad), -sin(angleRad), 0, this.legMountX[i]],
                [sin(angleRad), cos(angleRad), 0, this.legMountY[i]],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ];
        }
    }

    leg_ik(body_state: body_state_t): number[][] {
        const mountPosition = Array.from({ length: 6 }, (_, i) => [
            this.legMountX[i],
            this.legMountY[i],
            0
        ]);

        const transformation = rot_translation(
            body_state.omega,
            body_state.phi,
            body_state.psi,
            body_state.xm,
            body_state.ym,
            body_state.zm
        );

        const rotated_dest = Array.from({ length: 6 }, () => [0, 0, 0]);
        for (let i = 0; i < 6; i++) {
            const point = [...body_state.feet[i], 1];
            const transformed = multiplyVector(transformation, point);
            rotated_dest[i] = transformed.slice(0, 3);
        }

        const temp_dest = rotated_dest.map((d, i) => d.map((v, j) => v - mountPosition[i][j]));

        const localDest = temp_dest.map((t, i) => {
            const angle = this.legMountAngle[i];
            return [
                t[0] * Math.cos(angle) + t[1] * Math.sin(angle),
                t[0] * Math.sin(angle) - t[1] * Math.cos(angle),
                t[2]
            ];
        });

        const angles = Array.from({ length: 6 }, () => [0, 0, 0]);

        for (let i = 0; i < 6; i++) {
            let x = localDest[i][0] - this.l1;
            let y = localDest[i][1];
            angles[i][0] = (-Math.atan2(y, x) * 180) / Math.PI;

            x = Math.sqrt(x * x + y * y) - this.l2;
            y = localDest[i][2];
            const ar = Math.atan2(y, x);
            const lr2 = x * x + y * y;
            const lr = Math.sqrt(lr2);
            const a1 = Math.acos(
                (lr2 + this.l3 * this.l3 - this.l4 * this.l4) / (2 * this.l3 * lr)
            );
            const a2 = Math.acos(
                (lr2 - this.l3 * this.l3 + this.l4 * this.l4) / (2 * this.l4 * lr)
            );

            angles[i][1] = ((ar + a1) * 180) / Math.PI;
            angles[i][2] = 90 - ((a1 + a2) * 180) / Math.PI - 90;
        }

        return angles;
    }

    public calcIK(body_state: body_state_t): number[] {
        this.bodyIK(body_state);

        const jointAngles: number[] = [];
        for (let i = 0; i < 6; i++) {
            const footInBodyFrame = body_state.feet[i];
            // Transform the foot position from the body frame to the hip frame
            const hipFrameInverse = inverse(this.T_hip[i]);
            const footInHipFrame = multiplyVector(hipFrameInverse, [...footInBodyFrame, 1]).slice(
                0,
                3
            );
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
            const rotatedOffset = multiplyVector(Tm, hipOffset).slice(0, 3);
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
        const theta1 = atan2(y, x);
        const theta2 = atan2(z, sqrt(x * x + y * y) - this.l1);
        const theta3 = atan2(y, x) - atan2(z, sqrt(x * x + y * y) - this.l1);
        return [theta1, theta2, theta3];
    }
}
