import { multiplyVector, get_transformation_matrix } from './math';

export interface body_state_t {
    omega: number;
    phi: number;
    psi: number;
    xm: number;
    ym: number;
    zm: number;
    feet: number[][];
}

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
    mountX: number[];
    mountY: number[];
    rootJ1: number;
    j1J2: number;
    j2J3: number;
    j3Tip: number;
    mountAngles: number[];
    mountPosition: [number, number, number][];
    ca: number[];
    sa: number[];

    constructor(config: HexapodConfig) {
        this.mountX = config.legMountX;
        this.mountY = config.legMountY;
        this.rootJ1 = config.legRootToJoint1;
        this.j1J2 = config.legJoint1ToJoint2;
        this.j2J3 = config.legJoint2ToJoint3;
        this.j3Tip = config.legJoint3ToTip;
        this.mountAngles = config.legMountAngle.map(a => (a * Math.PI) / 180);
        this.mountPosition = this.mountX.map((x, i) => [x, this.mountY[i], 0]);
        this.ca = this.mountAngles.map(Math.cos);
        this.sa = this.mountAngles.map(Math.sin);
    }

    genPosture(j2: number, j3: number): [number, number, number, number][] {
        const ext = this.rootJ1 + this.j1J2 + this.j2J3 * Math.sin(j2) + this.j3Tip * Math.cos(j3);
        return this.mountX.map((mx, i) => {
            const x = mx + ext * this.ca[i];
            const y = this.mountY[i] + ext * this.sa[i];
            const z = this.j2J3 * Math.cos(j2) - this.j3Tip * Math.sin(j3);
            return [x, y, z, 1] as [number, number, number, number];
        });
    }

    inverseKinematics(bodyState: body_state_t): [number, number, number][] {
        const T = get_transformation_matrix(bodyState);
        const angles: [number, number, number][] = [];

        for (let i = 0; i < bodyState.feet.length; i++) {
            const [wx, wy, wz] = multiplyVector(T, bodyState.feet[i])
                .slice(0, 3)
                .map((v, idx) => v - this.mountPosition[i][idx]);

            const lx = wx * this.ca[i] + wy * this.sa[i];
            const ly = wx * this.sa[i] - wy * this.ca[i];
            const lz = wz;

            const dx = lx - this.rootJ1;
            const dy = ly;
            const a0 = -Math.atan2(dy, dx);

            const radial = Math.hypot(dx, dy) - this.j1J2;
            const vertical = lz;
            const base = Math.atan2(vertical, radial);
            const lr2 = radial * radial + vertical * vertical;
            const lr = Math.sqrt(lr2);

            let cosA1 = (lr2 + this.j2J3 ** 2 - this.j3Tip ** 2) / (2 * this.j2J3 * lr);
            let cosA2 = (lr2 - this.j2J3 ** 2 + this.j3Tip ** 2) / (2 * this.j3Tip * lr);
            cosA1 = Math.max(-1, Math.min(1, cosA1));
            cosA2 = Math.max(-1, Math.min(1, cosA2));

            const a1 = Math.acos(cosA1);
            const a2 = Math.acos(cosA2);

            angles.push([a0, base + a1, -(a1 + a2)]);
        }

        return angles;
    }
}
