const resets = new Set<() => void>()

/**
 * What the app knows of the robot it reaches: the state topics, the feature flags and the scans each register how to
 * forget it. The transports cannot tell a fail-over to another link of the same robot from another robot, so the
 * user's choice of a robot (an address to connect to, a Bluetooth device, a serial port) resets it all.
 */
export const robotData = {
  /** Adds what a robot switch forgets; returns the function that removes it. */
  register(reset: () => void): () => void {
    resets.add(reset)
    return () => resets.delete(reset)
  },
  reset() {
    resets.forEach(reset => reset())
  }
}
