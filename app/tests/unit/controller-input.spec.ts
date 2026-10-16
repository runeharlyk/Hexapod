import { describe, expect, it } from 'vitest'
import { commandOf, toControllerInput } from '../../src/lib/controller-input'
import { Message } from '../../src/lib/platform_shared/message'
import type { ControllerInput } from '../../src/lib/types/models'

const controls = (stride: number): ControllerInput => ({
  left: { x: 0.1, y: 0.2 },
  right: { x: 0.3, y: 0.4 },
  height: 0.5,
  speed: 0.6,
  s1: 0.7,
  feetDistance: 0.8,
  stride
})

// What the robot decodes from the message the controls are sent as.
const received = (input: ControllerInput) =>
  Message.decode(Message.encode({ input: toControllerInput(commandOf(input)) }).finish()).input

describe('controller input', () => {
  it('lists the controls in the firmware command order, the stride last', () => {
    expect(commandOf(controls(-0.9))).toEqual([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, -0.9])
  })

  it('sends the stride slider in the message the robot decodes, beside every other control', () => {
    const decoded = received(controls(-0.9))
    expect(decoded?.left?.x).toBeCloseTo(0.1, 6)
    expect(decoded?.left?.y).toBeCloseTo(0.2, 6)
    expect(decoded?.right?.x).toBeCloseTo(0.3, 6)
    expect(decoded?.right?.y).toBeCloseTo(0.4, 6)
    expect(decoded?.height).toBeCloseTo(0.5, 6)
    expect(decoded?.speed).toBeCloseTo(0.6, 6)
    expect(decoded?.s1).toBeCloseTo(0.7, 6)
    expect(decoded?.feetDistance).toBeCloseTo(0.8, 6)
    expect(decoded?.stride).toBeCloseTo(-0.9, 6)
  })
})
