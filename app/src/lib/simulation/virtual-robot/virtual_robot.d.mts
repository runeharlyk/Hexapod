// The Emscripten module scripts/build_virtual_robot.js writes beside this file: the virtual robot
// (firmware/host/virtual_robot_api.cpp) and the animation sessions (firmware/host/animation_api.cpp).
// Pointers and session handles are byte offsets into the module's memory.
export interface VirtualRobotModule {
  HEAPF32: Float32Array
  HEAPU8: Uint8Array
  UTF8ToString(pointer: number): string
  _reset(): void
  _setMode(mode: number): void
  _setGait(gait: number): void
  _command(
    lx: number,
    ly: number,
    rx: number,
    ry: number,
    h: number,
    s: number,
    s1: number,
    fd: number,
    stride: number
  ): void
  _setImu(...values: number[]): void
  _tick(): void
  _servosDriven(): number
  _servoAngles(): number

  _animCreate(): number
  _animDestroy(session: number): void
  _animIo(session: number): number
  _animStance(session: number): number
  _animClipBuffer(session: number, size: number): number
  _animLoad(session: number, size: number): number
  _animClearParams(session: number): void
  _animSetParam(session: number, id: number, value: number): void
  _animEvaluate(session: number, t: number, baseZ: number): void
  _animPoseToAngles(session: number): number
  _animLegJoints(session: number, leg: number, baseZ: number): void
  _animPlay(session: number, live: boolean, baseZ: number): void
  _animStop(session: number, baseZ: number): void
  _animUpdate(session: number, dt: number, baseZ: number): void
  _animLastPose(session: number): void
  _animState(session: number): number
  _animTime(session: number): number
  _animBlendFraction(session: number): number
  _animEntryBase(session: number): number
  _animEase(kind: number, t: number): number
}

export default function createVirtualRobot(): Promise<VirtualRobotModule>
