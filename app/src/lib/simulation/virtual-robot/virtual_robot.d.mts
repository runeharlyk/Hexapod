// The Emscripten module scripts/build_virtual_robot.js writes beside this file (firmware/host/wasm_api.cpp).
export interface VirtualRobotModule {
  HEAPF32: Float32Array
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
    fd: number
  ): void
  _setImu(...values: number[]): void
  _tick(): void
  _angles(): number
  _mode(): number
  _gait(): number
}

export default function createVirtualRobot(): Promise<VirtualRobotModule>
