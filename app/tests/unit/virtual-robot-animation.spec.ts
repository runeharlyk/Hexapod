// @vitest-environment node
import { describe, expect, it } from 'vitest'
import { AnimationState } from '../../src/lib/platform_shared/message'
import { loadVirtualRobot } from '../../src/lib/simulation/virtual-robot'

describe('the simulated firmware animation controls', () => {
  it('plays an embedded animation through the motion runner and stops back at rest', async () => {
    const robot = await loadVirtualRobot()
    robot.setMode(3)
    for (let i = 0; i < 200; i++) robot.tick()
    const before = Array.from(robot.servoAngles())
    robot.setMode(6)
    robot.tick()
    robot.playAnimation('wave', new Map())
    for (let i = 0; i < 200; i++) robot.tick()
    const status = robot.animationStatus()
    expect(status.name).toBe('wave')
    expect(status.state).not.toBe(AnimationState.ANIM_IDLE)
    expect(
      Array.from(robot.servoAngles()).some((value, i) => Math.abs(value - before[i]) > 1)
    ).toBe(true)
    robot.stopAnimation()
    for (let i = 0; i < 600; i++) robot.tick()
    expect(robot.animationStatus().state).toBe(AnimationState.ANIM_IDLE)
    expect(robot.servosDriven()).toBe(true)
  })

  it('rejects playback while deactivated and clears playback status on reset', async () => {
    const robot = await loadVirtualRobot()
    expect(() => robot.playAnimation('wave', new Map())).toThrow('could not start')
    expect(robot.animationStatus().state).toBe(AnimationState.ANIM_IDLE)
    robot.setMode(6)
    robot.tick()
    robot.playAnimation('wave', new Map())
    for (let i = 0; i < 100; i++) robot.tick()
    expect(robot.animationStatus().name).toBe('wave')
    robot.standPose()
    expect(robot.animationStatus().state).toBe(AnimationState.ANIM_IDLE)
    expect(robot.animationStatus().name).toBe('')
  })
})
