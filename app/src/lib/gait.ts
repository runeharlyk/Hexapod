// Declaration order IS the wire contract: GaitData.gait carries the GaitEnum position from
// platform_shared/message.proto, which control.ts and +layout.svelte map via Object.values order.
// Append new gaits at the end and mirror them in the proto.
export enum GaitType {
  TRI_GATE = 'Tri Gate',
  BI_GATE = 'Bi Gate',
  WAVE = 'Wave',
  RIPPLE = 'Ripple',
  TUNED = 'Tuned',
  AUTO = 'Auto'
}

// TUNED replays a searched joint trajectory at a fixed stroke, so the stride slider has no effect on it.
export const strideApplies = (gait: GaitType) => gait !== GaitType.TUNED

// TUNED takes its sticks as a velocity when they arrive, so the robot holds no heading on it (heading_hold.h).
export const headingHoldApplies = (gait: GaitType) => gait !== GaitType.TUNED
