import type { body_state_t } from './kinematic';
import type { Matrix } from './math';

export enum GaitType {
    TRI_GATE = 'Tri Gate',
    BI_GATE = 'Bi Gate',
    WAVE = 'Wave',
    RIPPLE = 'Ripple'
}

export const GaitLabels: Record<string, GaitType> = {
    'Tri Gate': GaitType.TRI_GATE,
    'Bi Gate': GaitType.BI_GATE,
    Wave: GaitType.WAVE,
    Ripple: GaitType.RIPPLE
};

export const default_offset: Record<GaitType, number[]> = {
    [GaitType.TRI_GATE]: [0, 0.5, 0, 0.5, 0, 0.5],
    [GaitType.BI_GATE]: [0, 1 / 3, 2 / 3, 2 / 3, 1 / 3, 0],
    [GaitType.WAVE]: [0, 1 / 6, 2 / 6, 5 / 6, 4 / 6, 3 / 6],
    [GaitType.RIPPLE]: [0, 4 / 6, 2 / 6, 5 / 6, 1 / 6, 3 / 6]
};

export const default_stand_frac: Record<GaitType, number> = {
    [GaitType.TRI_GATE]: 3.1 / 6,
    [GaitType.BI_GATE]: 2.1 / 6,
    [GaitType.WAVE]: 5 / 6,
    [GaitType.RIPPLE]: 5 / 6
};

export interface gait_state_t {
    step_height: number;
    step_x: number;
    step_z: number;
    step_angle: number;
    step_speed: number;
    step_depth: number;
    stand_frac: number;
    offset: number[];
    gait_type: GaitType;
}

export interface ControllerCommand {
    stop: number;
    lx: number;
    ly: number;
    rx: number;
    ry: number;
    h: number;
    s: number;
    s1: number;
}

export class GaitController {
    defaultPosition: Matrix;
    phase = 0;

    constructor(defaultPosition: Matrix) {
        this.defaultPosition = defaultPosition;
    }

    step(gait: gait_state_t, body: body_state_t, dt: number) {
        const { step_x, step_z, step_angle: angle } = gait;
        if (Math.abs(step_x) < 2 && Math.abs(step_z) < 2 && !angle) {
            body.feet = body.feet.map((f, i) =>
                f.map((v, j) => v + (this.defaultPosition[i][j] - v) * dt * 10)
            );
            this.phase = 0;
            return;
        }

        const lengthRaw = Math.hypot(step_x, step_z);
        const length = step_x < 0 ? -lengthRaw : lengthRaw;
        const speed =
            gait.step_speed *
            Math.min(1.5, Math.max(0.75, Math.abs(length) / 25, Math.abs(angle * 1.5)));
        const turnAmplitude = Math.atan2(step_z, length) * 2;

        this.advancePhase(dt, speed);

        const newFeet = this.defaultPosition.map(fp => fp.map(() => 0));

        for (let i = 0; i < this.defaultPosition.length; i++) {
            const defaultFoot = this.defaultPosition[i];
            const currentFoot = body.feet[i];
            const phase = (this.phase + gait.offset[i]) % 1;
            const [phNorm, curveFn, amp] = this.phaseParams(
                phase,
                gait.stand_frac,
                gait.step_depth,
                gait.step_height
            );
            const deltaPos = curveFn(length / 2, turnAmplitude, amp, phNorm);
            const deltaRot = curveFn(
                (angle * 180) / Math.PI,
                yawArc(defaultFoot, currentFoot),
                amp,
                phNorm
            );
            newFeet[i] = defaultFoot.map((v, j) => v + deltaPos[j] + deltaRot[j]);
            newFeet[i][3] = 1;
        }

        body.feet = newFeet;
    }

    private advancePhase(dt: number, velocity: number) {
        this.phase = (this.phase + dt * velocity) % 1;
    }

    private phaseParams(
        phase: number,
        standFrac: number,
        depth: number,
        height: number
    ): [number, typeof sine_curve | typeof bezier_curve, number] {
        if (phase < standFrac) {
            return [phase / standFrac, sine_curve, -depth];
        }
        return [(phase - standFrac) / (1 - standFrac), bezier_curve, height];
    }
}

export function sine_curve(length: number, angle: number, height: number, phase: number): number[] {
    const step = length * (1 - 2 * phase);
    const x = step * Math.cos(angle);
    const z = step * Math.sin(angle);
    const y = length ? height * Math.cos((Math.PI * (x + z)) / (2 * length)) : 0;
    return [x, z, y];
}

const yawArc = (default_foot_pos: number[], current_foot_pos: number[]): number => {
    const foot_mag = Math.hypot(default_foot_pos[0] + default_foot_pos[1]);
    const foot_dir = Math.atan2(default_foot_pos[1], default_foot_pos[0]);
    const offsets = [
        current_foot_pos[0] - default_foot_pos[0],
        current_foot_pos[2] - default_foot_pos[2],
        current_foot_pos[1] - default_foot_pos[1]
    ];
    const offset_mag = Math.hypot(offsets[0] + offsets[1]);
    const offset_mod = Math.atan2(offset_mag, foot_mag);

    return Math.PI / 2.0 + foot_dir + offset_mod;
};

const bezier_curve = (length: number, angle: number, height: number, phase: number): number[] => {
    const control_points = get_control_points(length, angle, height);
    const n = control_points.length - 1;

    const point = [0, 0, 0];
    for (let i = 0; i <= n; i++) {
        const bernstein_poly = comb(n, i) * Math.pow(phase, i) * Math.pow(1 - phase, n - i);
        point[0] += bernstein_poly * control_points[i][0];
        point[1] += bernstein_poly * control_points[i][1];
        point[2] += bernstein_poly * control_points[i][2];
    }
    return point;
};
const get_control_points = (length: number, angle: number, height: number): number[][] => {
    const X_POLAR = Math.cos(angle);
    const Z_POLAR = Math.sin(angle);

    const STEP = [
        -length,
        -length * 1.4,
        -length * 1.5,
        -length * 1.5,
        -length * 1.5,
        0.0,
        0.0,
        0.0,
        length * 1.5,
        length * 1.5,
        length * 1.4,
        length
    ];

    const Y = [
        0.0,
        0.0,
        height * 0.9,
        height * 0.9,
        height * 0.9,
        height * 0.9,
        height * 0.9,
        height * 1.1,
        height * 1.1,
        height * 1.1,
        0.0,
        0.0
    ];

    const control_points: number[][] = [];

    for (let i = 0; i < STEP.length; i++) {
        const X = STEP[i] * X_POLAR;
        const Z = STEP[i] * Z_POLAR;
        control_points.push([X, Z, Y[i]]);
    }

    return control_points;
};

const comb = (n: number, k: number): number => {
    if (k < 0 || k > n) return 0;
    if (k === 0 || k === n) return 1;
    k = Math.min(k, n - k);
    let c = 1;
    for (let i = 0; i < k; i++) {
        c = (c * (n - i)) / (i + 1);
    }
    return c;
};
