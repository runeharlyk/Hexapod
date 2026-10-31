import { expect, test, type Locator, type Page } from '@playwright/test'
import {
  GaitData,
  GaitEnum,
  Message,
  ModeData,
  ModesEnum,
  StanceData,
  StanceEnum
} from '../../src/lib/platform_shared/message'
import { tagOf } from '../../src/lib/transport/message-codec'
import { serveStockRobot, stockRobot, type StockRobot } from '../fake-robot/stock'

const base = process.env.BASE_PATH ?? ''

const CLASSIC_GAITS = ['Tri Gate', 'Bi Gate', 'Wave', 'Ripple']

// The controller of the stock robot, which takes the mode, the gait and the stance as the firmware does.
const controller = async (page: Page, mode: ModesEnum, gait = GaitEnum.TRI_GATE) => {
  const robot = stockRobot()
  robot.set(ModeData, ModeData.create({ mode }))
  robot.set(GaitData, { gait })
  await serveStockRobot(page, robot)
  await page.goto(`${base}/controller`)
  return robot
}

const selector = (page: Page, name: string) => page.getByRole('combobox', { name, exact: true })

const offered = (select: Locator) =>
  select.locator('option').evaluateAll(options => options.map(o => (o as HTMLOptionElement).value))

const sent = (robot: StockRobot) =>
  robot.received.flatMap(frame => (frame.stance ? [frame.stance.stance] : []))

// Another client's request, as the Python facade or a second app makes one.
const elsewhere = (robot: StockRobot, message: Partial<Message>) =>
  robot.take(Message.create(message))

test('the stance selector sends the stance and shows the one the robot reports', async ({
  page
}) => {
  const robot = await controller(page, ModesEnum.STAND)
  const stance = selector(page, 'Stance')
  await expect(stance).toHaveValue('default')
  expect(await offered(stance)).toEqual(['default', 'crab'])

  await stance.selectOption('crab')
  await expect.poll(() => sent(robot)).toEqual([StanceEnum.STANCE_CRAB])
  await expect(stance).toHaveValue('crab')

  elsewhere(robot, { stance: { stance: StanceEnum.STANCE_DEFAULT } })
  await expect(stance).toHaveValue('default')
})

test('the crab offers no AUTO or TUNED, and the gait follows the switch to TRI_GATE it makes from AUTO', async ({
  page
}) => {
  const robot = await controller(page, ModesEnum.WALK, GaitEnum.AUTO)
  const gait = selector(page, 'Gait')
  await expect(gait).toHaveValue('Auto')
  expect(await offered(gait)).toEqual([...CLASSIC_GAITS, 'Tuned', 'Auto'])

  await selector(page, 'Stance').selectOption('crab')
  await expect(selector(page, 'Stance')).toHaveValue('crab')
  await expect(gait).toHaveValue('Tri Gate')
  expect(await offered(gait)).toEqual(CLASSIC_GAITS)
  expect(robot.received.filter(frame => frame.gait)).toHaveLength(0)

  await selector(page, 'Stance').selectOption('default')
  await expect(selector(page, 'Stance')).toHaveValue('default')
  expect(await offered(gait)).toEqual([...CLASSIC_GAITS, 'Tuned', 'Auto'])
})

test('the crab offers no WALK_NN among the modes', async ({ page }) => {
  await controller(page, ModesEnum.STAND)
  const mode = selector(page, 'Mode')
  await expect(mode).toHaveValue('stand')
  expect(await offered(mode)).toContain('walk nn')

  await selector(page, 'Stance').selectOption('crab')
  await expect(selector(page, 'Stance')).toHaveValue('crab')
  expect(await offered(mode)).not.toContain('walk nn')
  expect(await offered(mode)).toContain('walk')
})

// A request this app sends before it hears another client's change can be one the robot refuses; the robot answers with
// the value in force, often the one shown already, and the selector shows that, never the refused request. The robot's
// push of the other client's change is lost here, so the app keeps offering what the robot then refuses.
test('a refused gait, mode or stance leaves its selector on the value the robot answers with', async ({
  page
}) => {
  const robot = await controller(page, ModesEnum.WALK)
  await expect(selector(page, 'Gait')).toHaveValue('Tri Gate')
  robot.dropNext(tagOf(StanceData))
  elsewhere(robot, { stance: { stance: StanceEnum.STANCE_CRAB } })
  await expect(selector(page, 'Stance')).toHaveValue('default')

  await selector(page, 'Gait').selectOption('Auto')
  await expect
    .poll(() => robot.received.flatMap(frame => (frame.gait ? [frame.gait.gait] : [])))
    .toEqual([GaitEnum.AUTO])
  await expect(selector(page, 'Gait')).toHaveValue('Tri Gate')

  await selector(page, 'Mode').selectOption('walk nn')
  await expect
    .poll(() => robot.received.flatMap(frame => (frame.mode ? [frame.mode.mode] : [])))
    .toEqual([ModesEnum.WALK_NN])
  await expect(selector(page, 'Mode')).toHaveValue('walk')

  elsewhere(robot, { stance: { stance: StanceEnum.STANCE_DEFAULT } })
  await expect(selector(page, 'Stance')).toHaveValue('default')
  await selector(page, 'Mode').selectOption('stand')
  await expect(selector(page, 'Mode')).toHaveValue('stand')
  robot.dropNext(tagOf(ModeData))
  elsewhere(robot, { mode: { mode: ModesEnum.WALK_NN, handheld: false } })
  await selector(page, 'Stance').selectOption('crab')
  await expect.poll(() => sent(robot)).toEqual([StanceEnum.STANCE_CRAB])
  await expect(selector(page, 'Stance')).toHaveValue('default')
})
