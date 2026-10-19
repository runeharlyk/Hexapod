// The controls drive either the robot or the simulation view's own robot. While the simulation view is shown, its
// sticks, mode and gait must never reach the robot's command path: the stick store the layout sends to the robot
// (outControllerData) and the mode and gait requests sent over the data broker. The robot's mode, gait and goal come from
// the fake robot's pushes, as the layout follows them.
import { afterAll, afterEach, beforeAll, beforeEach, describe, expect, it, vi } from 'vitest'
import { get } from 'svelte/store'
import { dataBroker } from '../../src/lib/transport/databroker'
import { input, outControllerData } from '../../src/lib/stores'
import { gait, goalStatus, mode, modeState } from '../../src/lib/robot/state'
import { FakeRobot, fakeLink } from '../fake-robot/transport'
import {
  drive,
  driveGait,
  driveMode,
  driveSimulation,
  drivenGait,
  drivenGoal,
  drivenMode,
  simulation
} from '../../src/lib/stores/drive'
import {
  GaitData,
  GaitEnum,
  GoalKind,
  GoalState,
  GoalStatus,
  ModeData,
  ModesEnum
} from '../../src/lib/platform_shared/message'
import { commandOf } from '../../src/lib/controller-input'
import { GaitType } from '../../src/lib/gait'
import { MotionModes } from '../../src/lib/motion'
import type { Command } from '../../src/lib/simulation/virtual-robot'

const CENTRED: Command = [0, 0, 0, 0, 0, 0, 0, 0, 0]
const DEFLECTED: Command = [0.5, 0.4, -0.3, 0.2, 0.1, 0, 0, 0, 0]
const sticks = (c: Command) => c.slice(0, 4)

// Lets the robot's pushes, which arrive on later microtasks, land.
const settle = () => new Promise(resolve => setTimeout(resolve, 0))

describe('the controls', () => {
  const robot = new FakeRobot()
  const link = fakeLink(robot)
  let emit: ReturnType<typeof vi.spyOn>
  let following: (() => void)[] = []

  beforeAll(async () => {
    dataBroker.addTransport(link)
    await link.connect()
    following = [mode, gait, goalStatus, modeState].map(store => store.subscribe(() => {}))
  })

  afterAll(() => following.forEach(stop => stop()))

  beforeEach(async () => {
    emit = vi.spyOn(dataBroker, 'emit').mockImplementation(() => true)
    outControllerData.set(CENTRED)
    robot.set(ModeData, { mode: ModesEnum.STAND, handheld: false })
    robot.set(GaitData, { gait: GaitEnum.TRI_GATE })
    await settle()
  })

  afterEach(() => {
    driveSimulation(false)
    emit.mockRestore()
  })

  it("keep the simulation's sticks, mode and gait off the robot's command path", () => {
    driveSimulation(true)
    drive(DEFLECTED)
    driveMode(MotionModes.WALK)
    driveGait(GaitType.WAVE)
    expect(get(outControllerData)).toEqual(CENTRED)
    expect(get(mode)).toBe(MotionModes.STAND)
    expect(get(gait)).toBe(GaitType.TRI_GATE)
    expect(emit).not.toHaveBeenCalled()
    expect(get(simulation.command)).toEqual(DEFLECTED)
    expect(get(simulation.mode)).toBe(MotionModes.WALK)
    expect(get(simulation.gait)).toBe(GaitType.WAVE)
    expect([get(drivenMode), get(drivenGait)]).toEqual([MotionModes.WALK, GaitType.WAVE])
  })

  it("leave the simulation's mode where it is when the robot reports another", async () => {
    driveSimulation(true)
    driveMode(MotionModes.IDLE)
    robot.set(ModeData, { mode: ModesEnum.WALK, handheld: false })
    await settle()
    expect(get(mode)).toBe(MotionModes.WALK)
    expect(get(simulation.mode)).toBe(MotionModes.IDLE)
    expect(get(drivenMode)).toBe(MotionModes.IDLE)
  })

  it('centre the robot sticks when the simulation takes the controls, keeping its sliders', () => {
    drive(DEFLECTED)
    driveSimulation(true)
    expect(sticks(get(outControllerData))).toEqual([0, 0, 0, 0])
    expect(get(outControllerData)[4]).toBe(DEFLECTED[4])
  })

  it('hand no simulation stick to the robot when they go back to it', () => {
    driveSimulation(true)
    input.update(i => ({ ...i, left: { x: 0.7, y: 0.2 }, right: { x: -0.4, y: 0 } }))
    drive(DEFLECTED)
    driveSimulation(false)
    expect(get(input).left).toEqual({ x: 0, y: 0 })
    expect(get(input).right).toEqual({ x: 0, y: 0 })
    expect(sticks(get(outControllerData))).toEqual([0, 0, 0, 0])
    expect(sticks(get(simulation.command))).toEqual([0, 0, 0, 0])
    expect(emit).not.toHaveBeenCalled()
  })

  it("show each robot's own sliders, so neither takes the other's on its next stick", () => {
    const robotSliders = { height: 0.3, speed: 0.5, s1: 0.2, feetDistance: -0.1, stride: 0.4 }
    const simSliders = { height: -0.2, speed: 0.1, s1: 0.6, feetDistance: 0.3, stride: -0.5 }
    const sliders = () => {
      const { height, speed, s1, feetDistance, stride } = get(input)
      return { height, speed, s1, feetDistance, stride }
    }
    simulation.command.set(CENTRED)
    input.update(i => ({ ...i, ...robotSliders }))
    drive(commandOf(get(input)))

    driveSimulation(true)
    expect(sliders()).toEqual({ height: 0, speed: 0, s1: 0, feetDistance: 0, stride: 0 })
    input.update(i => ({ ...i, ...simSliders }))
    drive(commandOf(get(input)))

    driveSimulation(false)
    expect(sliders()).toEqual(robotSliders)
    input.update(i => ({ ...i, left: { x: 0.5, y: 0 } }))
    drive(commandOf(get(input)))
    expect(get(outControllerData).slice(4)).toEqual(
      commandOf({ ...get(input), ...robotSliders }).slice(4)
    )

    driveSimulation(true)
    expect(sliders()).toEqual(simSliders)
  })

  it("show the robot's goal only while they drive the robot", async () => {
    robot.set(
      GoalStatus,
      GoalStatus.create({
        id: 1,
        state: GoalState.GOAL_RUNNING,
        kind: GoalKind.GOAL_FORWARD,
        target: 1,
        progress: 0.5,
        passes: 1
      })
    )
    await settle()
    expect(get(drivenGoal)?.id).toBe(1)
    driveSimulation(true)
    expect(get(drivenGoal)).toBeNull()
    driveSimulation(false)
    expect(get(drivenGoal)?.id).toBe(1)
    robot.set(GoalStatus, GoalStatus.create())
    await settle()
  })

  // The mode shown is the robot's: the request leaves it pending until the robot pushes the change.
  it('drive the robot when the simulation view is not shown', async () => {
    drive(DEFLECTED)
    driveMode(MotionModes.WALK)
    expect(get(outControllerData)).toEqual(DEFLECTED)
    expect(emit).toHaveBeenCalledTimes(1)
    expect([get(mode), get(modeState).pending]).toEqual([MotionModes.STAND, true])

    robot.set(ModeData, { mode: ModesEnum.WALK, handheld: false })
    await settle()
    expect([get(mode), get(modeState).pending]).toEqual([MotionModes.WALK, false])
  })
})
