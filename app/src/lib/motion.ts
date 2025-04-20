import { degToRad } from 'three/src/math/MathUtils';
import { config } from './components/config';
import Kinematics, { type body_state_t } from './kinematic';
import {
    default_offset,
    default_stand_frac,
    GaitController,
    GaitType,
    type gait_state_t
} from './gait';

export enum MotionModes {
    DEACTIVATED = 'deactivated',
    IDLE = 'idle',
    POSE = 'pose',
    STAND = 'stand',
    WALK = 'walk'
}

export default class Motion {
    mode: MotionModes;
    kinematics: Kinematics;
    gait: GaitController;
    body_state: body_state_t;
    gait_state: gait_state_t;
    defaultPosition: [number, number, number, number][];
    angles = new Array(18).fill(0);
    targetAngles = new Array(18).fill(0);

    poses = [new Array(18).fill(0)];
    current_pose_idx = 0;

    lastTick: number = 0;

    constructor() {
        this.mode = MotionModes.STAND;
        this.kinematics = new Kinematics(config);
        this.defaultPosition = this.kinematics.genPosture(degToRad(60), degToRad(75));
        this.gait = new GaitController(this.defaultPosition);
        this.body_state = {
            omega: 0,
            phi: 0,
            psi: 0,
            xm: 0,
            ym: 0,
            zm: 15,
            feet: this.defaultPosition
        };
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
        };
    }

    setMode(mode: MotionModes) {
        this.mode = mode;
    }

    setGait(gait: GaitType) {
        this.gait_state.gait_type = gait;
        this.gait_state.offset = default_offset[gait];
        this.gait_state.stand_frac = default_stand_frac[gait];
    }

    handleCommand(command: number[]) {
        this.body_state.zm = command[5] / 2.4;
        switch (this.mode) {
            case MotionModes.STAND:
                this.body_state.xm = command[1] / 2.4;
                this.body_state.ym = command[2] / 2.4;
                this.body_state.psi = command[3] / 500;
                this.body_state.omega = command[4] / 500;
                break;
            case MotionModes.WALK:
                this.gait_state.step_x = command[1];
                this.gait_state.step_z = command[2];
                this.gait_state.step_angle = command[3] / 150;
                this.gait_state.step_speed = command[6] / 128 + 1;
                this.gait_state.step_height = command[7] / 2.4;

                this.body_state.omega = command[4] / 500;
                break;
        }
    }

    step() {
        switch (this.mode) {
            case MotionModes.DEACTIVATED:
            case MotionModes.IDLE:
                return false;
            case MotionModes.POSE:
                this.targetAngles = this.poses[this.current_pose_idx];
                break;
            case MotionModes.STAND: {
                this.targetAngles = this.order(
                    this.kinematics.inverseKinematics(this.body_state).flat()
                );
                break;
            }
            case MotionModes.WALK: {
                const delta = performance.now() - this.lastTick;
                this.lastTick = performance.now();
                this.gait.step(this.gait_state, this.body_state, delta / 1000);
                this.targetAngles = this.order(
                    this.kinematics.inverseKinematics(this.body_state).flat()
                );
                break;
            }
        }
        return true;
    }

    private order(a: number[]) {
        return [3, 0, 4, 1, 5, 2].flatMap(i => a.slice(i * 3, i * 3 + 3));
    }
}
