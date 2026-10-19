// The robot's BLE frames: [uint16 LE length][payload], sent as notifications of a fixed chunk size and a shorter last
// one (firmware/include/communication/ble_frame.h). Every frame starts a notification, so a notification never holds
// bytes of two frames, and a frame's first notification, when it does not hold the whole frame, is a full chunk.
// A length prefix alone cannot find its way back once a frame was cut short: it would complete the cut frame with the
// next frames' bytes and then read lengths from the middle of payloads. These rules let the reader see a stream out of
// step at the first notification that breaks them; the robot ends the link rather than send a frame cut short, so
// this is the reader's own guard.

/** The robot's largest encoded message (PROTO_BUFFER_SIZE in firmware/include/communication/proto_helpers.h). */
export const BLE_MAX_FRAME = 2048

/** A whole frame's payload, `'partial'` while a frame is still arriving, `'out-of-step'` when the stream broke. */
export type Reassembled = Uint8Array | 'partial' | 'out-of-step'

export class BleReassembler {
  private frame: Uint8Array | null = null
  private filled = 0
  private chunk = 0

  push(notification: Uint8Array): Reassembled {
    if (!this.frame) return this.start(notification)
    const remaining = this.frame.length - this.filled
    if (notification.length !== Math.min(this.chunk, remaining)) return this.outOfStep()
    this.frame.set(notification, this.filled)
    this.filled += notification.length
    if (this.filled < this.frame.length) return 'partial'
    const payload = this.frame.subarray(2)
    this.reset()
    return payload
  }

  reset() {
    this.frame = null
    this.filled = 0
    this.chunk = 0
  }

  private start(notification: Uint8Array): Reassembled {
    if (notification.length < 2) return this.outOfStep()
    const length = notification[0] | (notification[1] << 8)
    if (length === 0 || length > BLE_MAX_FRAME) return this.outOfStep()
    const total = 2 + length
    if (notification.length > total) return this.outOfStep()
    if (notification.length === total) return notification.slice(2)
    this.frame = new Uint8Array(total)
    this.frame.set(notification)
    this.filled = notification.length
    this.chunk = notification.length
    return 'partial'
  }

  private outOfStep(): Reassembled {
    this.reset()
    return 'out-of-step'
  }
}
