import type { HexapodConfig } from '$lib/kinematic';

const kLegMountLeftRightX = 29.87;
const kLegMountOtherX = 22.41;
const kLegMountOtherY = 55.41;

const kLegRootToJoint1 = 20.75;
const kLegJoint1ToJoint2 = 28.0;
const kLegJoint2ToJoint3 = 42.6;
const kLegJoint3ToTip = 89.07;

const SIN30 = 0.5;
const COS30 = 0.866;
const SIN45 = 0.7071;
const COS45 = 0.7071;
const SIN15 = 0.2588;
const COS15 = 0.9659;

const STANDBY_Z = kLegJoint3ToTip * COS15 - kLegJoint2ToJoint3 * SIN30;
const LEFTRIGHT_X =
    kLegMountLeftRightX +
    kLegRootToJoint1 +
    kLegJoint1ToJoint2 +
    kLegJoint2ToJoint3 * COS30 +
    kLegJoint3ToTip * SIN15;
const OTHER_X =
    kLegMountOtherX +
    (kLegRootToJoint1 + kLegJoint1ToJoint2 + kLegJoint2ToJoint3 * COS30 + kLegJoint3ToTip * SIN15) *
        COS45;
const OTHER_Y =
    kLegMountOtherY +
    (kLegRootToJoint1 + kLegJoint1ToJoint2 + kLegJoint2ToJoint3 * COS30 + kLegJoint3ToTip * SIN15) *
        SIN45;

export const defaultPosition: [number, number, number][] = [
    [OTHER_X, OTHER_Y, -STANDBY_Z],
    [LEFTRIGHT_X, 0, -STANDBY_Z],
    [OTHER_X, -OTHER_Y, -STANDBY_Z],
    [-OTHER_X, -OTHER_Y, -STANDBY_Z],
    [-LEFTRIGHT_X, 0, -STANDBY_Z],
    [-OTHER_X, OTHER_Y, -STANDBY_Z]
];

export const mountPosition: [number, number, number][] = [
    [kLegMountOtherX, kLegMountOtherY, 0],
    [kLegMountLeftRightX, 0, 0],
    [kLegMountOtherX, -kLegMountOtherY, 0],
    [-kLegMountOtherX, -kLegMountOtherY, 0],
    [-kLegMountLeftRightX, 0, 0],
    [-kLegMountOtherX, kLegMountOtherY, 0]
];

export const defaultAngle = [-45, 0, 45, 135, 180, 225];

export const angleLimitation: [number, number][] = [
    [-45, 45],
    [-45, 75],
    [-60, 60]
];

export const config: HexapodConfig = {
    legMountX: [22.41, 29.87, 22.41, -22.41, -29.87, -22.41],
    legMountY: [55.41, 0, -55.41, 55.41, 0, -55.41],
    legMountAngle: [45, 0, -45, -225, -180, -135],
    legRootToJoint1: 20.75,
    legJoint1ToJoint2: 28.0,
    legJoint2ToJoint3: 42.6,
    legJoint3ToTip: 89.07,
    legScale: [
        [1, -1, -1],
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1],
        [1, -1, -1],
        [1, -1, -1]
    ]
};
