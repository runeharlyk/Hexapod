// The simulated servo: a port of HexapodSim._apply_servo (simulation/src/sim/mj_runtime.py), pinned to it
// by tests/unit/fixtures/servo_model.json, which simulation/scripts/make_servo_fixture.py writes.
// A cascaded loop: the position error sets a speed, capped by the no-load speed and by the speed from which
// stall torque can still stop the joint at its target, and a stiff speed loop turns that into torque.

export const SERVO = { kp: 250, kv: 1.5, jEff: 0.004, stall: 0.34, kHold: 2.5, noload: 16 }

// The motor gives the mean of stall and hold; the gear's friction adds the rest when a load back-drives it.
export const MOTOR_TORQUE_CAP = 0.5 * (1 + SERVO.kHold) * SERVO.stall
export const GEAR_FRICTION = 0.5 * (SERVO.kHold - 1) * SERVO.stall

/** The torque (N.m) a powered servo applies to a joint at `position` and `velocity` holding `target`. */
export function servoTorque(target: number, position: number, velocity: number): number {
  const error = target - position
  const magnitude = Math.abs(error)
  const stopping = Math.sqrt((2 * SERVO.stall * magnitude) / SERVO.jEff)
  const speed = Math.sign(error) * Math.min(SERVO.kp * magnitude, stopping, SERVO.noload)
  return Math.min(MOTOR_TORQUE_CAP, Math.max(-MOTOR_TORQUE_CAP, SERVO.kv * (speed - velocity)))
}
