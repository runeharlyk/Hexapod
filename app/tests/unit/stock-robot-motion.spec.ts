// The stock fake robot takes the mode, the gait and the stance as the firmware does (firmware/test/test_stance), so the
// Playwright tests of the controller run against its rules: AUTO, TUNED and WALK_NN refused while the crab is in force
// or its feet still stand on it, the crab refused in WALK_NN, the crab asked for in AUTO or TUNED switching the gait to
// TRI_GATE first, and every refusal answered with the value in force on its topic. simulation/test_real_backend.py's
// fake robot holds the same rules.
import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import {
  GaitData,
  GaitEnum,
  ModeData,
  ModesEnum,
  StanceData,
  StanceEnum
} from '$lib/platform_shared/message'
import { DataBroker } from '$lib/transport/databroker'
import { fakeLink } from '../fake-robot/transport'
import { stockRobot, type StockRobot } from '../fake-robot/stock'

const settle = () => vi.advanceTimersByTimeAsync(0)

describe("the stock robot's motion", () => {
  let robot: StockRobot
  let broker: DataBroker
  // What the robot pushed since the last check, in order, as `topic value`.
  let heard: string[]

  const pushes = async () => {
    await settle()
    return heard.splice(0)
  }

  const mode = (mode: ModesEnum) => broker.emit(ModeData, { mode, handheld: false })
  const gait = (gait: GaitEnum) => broker.emit(GaitData, { gait })
  const stance = (stance: StanceEnum) => broker.emit(StanceData, { stance })

  beforeEach(async () => {
    vi.useFakeTimers()
    robot = stockRobot()
    const link = fakeLink(robot)
    broker = new DataBroker()
    broker.addTransport(link)
    await link.connect()
    heard = []
    broker.on(ModeData, ({ mode }) => heard.push(`mode ${ModesEnum[mode]}`))
    broker.on(GaitData, ({ gait }) => heard.push(`gait ${GaitEnum[gait]}`))
    broker.on(StanceData, ({ stance }) => heard.push(`stance ${StanceEnum[stance]}`))
    await pushes()
    mode(ModesEnum.STAND)
    await pushes()
  })

  afterEach(() => vi.useRealTimers())

  it('pushes a stance once per change and none for a repeat', async () => {
    stance(StanceEnum.STANCE_CRAB)
    stance(StanceEnum.STANCE_CRAB)
    stance(StanceEnum.STANCE_DEFAULT)
    expect(await pushes()).toEqual(['stance STANCE_CRAB', 'stance STANCE_DEFAULT'])
  })

  for (const asked of [GaitEnum.AUTO, GaitEnum.TUNED])
    it(`switches ${GaitEnum[asked]} to TRI_GATE, pushed before the crab it then takes`, async () => {
      gait(asked)
      await pushes()
      stance(StanceEnum.STANCE_CRAB)
      expect(await pushes()).toEqual(['gait TRI_GATE', 'stance STANCE_CRAB'])
    })

  it('answers AUTO and TUNED in the crab with the gait in force, and takes a classic gait', async () => {
    gait(GaitEnum.WAVE)
    stance(StanceEnum.STANCE_CRAB)
    await pushes()
    gait(GaitEnum.AUTO)
    gait(GaitEnum.TUNED)
    gait(GaitEnum.RIPPLE)
    expect(await pushes()).toEqual(['gait WAVE', 'gait WAVE', 'gait RIPPLE'])
  })

  it('restates the mode for WALK_NN in the crab, and answers the crab in WALK_NN with the stance in force', async () => {
    stance(StanceEnum.STANCE_CRAB)
    await pushes()
    mode(ModesEnum.WALK_NN)
    expect(await pushes()).toEqual(['mode STAND'])

    stance(StanceEnum.STANCE_DEFAULT)
    mode(ModesEnum.WALK_NN)
    await pushes()
    stance(StanceEnum.STANCE_CRAB)
    expect(await pushes()).toEqual(['stance STANCE_DEFAULT'])
  })

  // The firmware steps the feet off the crab in a stand or a walk alone, and refuses meanwhile what the crab refuses.
  it("keeps the crab's feet after the default stance asked for outside STAND and WALK, until a stand", async () => {
    stance(StanceEnum.STANCE_CRAB)
    mode(ModesEnum.IDLE)
    stance(StanceEnum.STANCE_DEFAULT)
    await pushes()
    gait(GaitEnum.AUTO)
    mode(ModesEnum.WALK_NN)
    expect(await pushes()).toEqual(['gait TRI_GATE', 'mode IDLE'])

    mode(ModesEnum.STAND)
    gait(GaitEnum.AUTO)
    mode(ModesEnum.WALK_NN)
    expect(await pushes()).toEqual(['mode STAND', 'gait AUTO', 'mode WALK_NN'])
  })
})
