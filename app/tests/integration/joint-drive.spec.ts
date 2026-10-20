import { expect, test, type Locator, type Page } from '@playwright/test'
import { AnglesData, ModeData, ModesEnum } from '../../src/lib/platform_shared/message'
import { ALL_SERVOS } from '../../src/lib/motion'
import { FakeRobot, serveRobot } from '../fake-robot/playwright'

const base = process.env.BASE_PATH ?? ''

const DRIVING = "Driving the robot's joints"

// A drag waits for the model to load (findJoint), which a parallel run's software rendering can stretch.
test.describe.configure({ timeout: 90_000 })

type Point = { x: number; y: number }

const jointFrames = (robot: FakeRobot) => robot.received.filter(m => m.jointAngle)

const cursorOf = (canvas: Locator) => canvas.evaluate(c => (c as HTMLCanvasElement).style.cursor)

// Points over the 3D view, row by row, `step` of its size apart: wherever the model's legs are drawn, some lie on one.
const grid = async (canvas: Locator, step: number): Promise<Point[]> => {
  const box = await canvas.boundingBox()
  if (!box) throw new Error('the 3D view is not drawn')
  const points: Point[] = []
  for (let fy = 0.15; fy < 0.9; fy += step)
    for (let fx = 0.1; fx < 0.9; fx += step)
      points.push({ x: box.x + box.width * fx, y: box.y + box.height * fy })
  return points
}

// A point over a leg: while the view drives the joints it shows the grab cursor there.
const scanForJoint = async (
  page: Page,
  canvas: Locator,
  step: number
): Promise<Point | undefined> => {
  for (const point of await grid(canvas, step)) {
    await page.mouse.move(point.x, point.y)
    if ((await cursorOf(canvas)) === 'grab') return point
  }
  return undefined
}

// The model and its meshes load after the page, later still under the load of a parallel run, so a view that drives
// the joints is scanned until a leg shows under the pointer, within `timeout`.
const findJoint = async (
  page: Page,
  canvas: Locator,
  timeout = 30_000
): Promise<Point | undefined> => {
  const deadline = Date.now() + timeout
  do {
    const point = await scanForJoint(page, canvas, 0.04)
    if (point) return point
  } while (Date.now() < deadline)
  return undefined
}

const drag = async (page: Page, { x, y }: Point) => {
  await page.mouse.move(x, y)
  await page.mouse.down()
  for (let step = 1; step <= 10; step++) await page.mouse.move(x + 4 * step, y - 3 * step)
  await page.mouse.up()
}

const idleRobot = async (page: Page) => {
  const robot = new FakeRobot()
  robot.set(ModeData, ModeData.create({ mode: ModesEnum.IDLE }))
  await serveRobot(page, robot)
  return robot
}

// The preview origin is http, so the layout opens ws://<host>/api/ws by itself, which the fake robot answers.
test("in IDLE a drag of the controller's 3D model sends its joint, and in STAND it sends none", async ({
  page
}) => {
  const robot = await idleRobot(page)
  await page.addInitScript(() =>
    localStorage.setItem('selected_view', JSON.stringify('3D representation'))
  )
  await page.goto(`${base}/controller`)
  await expect(page.getByText(DRIVING)).toBeVisible()
  await expect(page.locator('#height')).toHaveCount(0)
  const canvas = page.locator('canvas').first()

  const point = await findJoint(page, canvas)
  expect(point, 'a leg of the model under the pointer').toBeDefined()
  await drag(page, point!)
  await expect.poll(() => jointFrames(robot).length).toBeGreaterThan(0)
  for (const { jointAngle } of jointFrames(robot)) {
    expect(jointAngle!.servo).toBeLessThan(18)
    expect(Number.isFinite(jointAngle!.degrees)).toBe(true)
  }

  robot.set(ModeData, ModeData.create({ mode: ModesEnum.STAND }))
  await expect(page.getByText(DRIVING)).toHaveCount(0)
  await expect(page.locator('#height')).toBeVisible()
  const sent = jointFrames(robot).length
  await page.mouse.move(point!.x, point!.y)
  expect(await cursorOf(canvas)).toBe('')
  await drag(page, point!)
  await page.waitForTimeout(500)
  expect(jointFrames(robot)).toHaveLength(sent)
})

// The angles an idle robot reports holding, far from the stand the model rests in before any report, on every joint.
const HELD = Array.from({ length: 18 }, (_, servo) => [-25, 70, -10][servo % 3])

// In IDLE the robot reports the angles its servos hold, and the model shows them, so a drag turns a joint from where
// the servo is: the first angle sent is the reported one moved by the first pointer step only.
test('in IDLE a drag starts from the pose the robot reports', async ({ page }) => {
  const robot = await idleRobot(page)
  robot.set(AnglesData, AnglesData.create({ angles: HELD }))
  await page.addInitScript(() =>
    localStorage.setItem('selected_view', JSON.stringify('3D representation'))
  )
  await page.goto(`${base}/controller`)
  await expect(page.getByText(DRIVING)).toBeVisible()
  const canvas = page.locator('canvas').first()

  const point = await findJoint(page, canvas)
  expect(point, 'a leg of the model under the pointer').toBeDefined()
  await page.mouse.move(point!.x, point!.y)
  await page.mouse.down()
  await page.mouse.move(point!.x + 1, point!.y)
  await expect.poll(() => jointFrames(robot).length).toBeGreaterThan(0)
  await page.mouse.up()
  const { servo, degrees } = jointFrames(robot)[0].jointAngle!
  expect(Math.abs(degrees - HELD[servo]), `servo ${servo} sent ${degrees}`).toBeLessThan(8)
})

// A robot that wakes its servos limp, as after a deactivation, reports no angles until one is written: the model keeps
// the pose it draws, which the controls mark limp, and a drag turns a joint from there rather than from the report's 0.
test('in IDLE a limp servo is dragged from the pose the model draws', async ({ page }) => {
  const robot = await idleRobot(page)
  robot.set(AnglesData, AnglesData.create({ angles: HELD }))
  await page.addInitScript(() =>
    localStorage.setItem('selected_view', JSON.stringify('3D representation'))
  )
  await page.goto(`${base}/controller`)
  await expect(page.getByText(DRIVING)).toBeVisible()
  await expect(page.getByText(/servos limp/)).toHaveCount(0)
  const canvas = page.locator('canvas').first()
  const point = await findJoint(page, canvas)
  expect(point, 'a leg of the model under the pointer').toBeDefined()

  robot.set(AnglesData, AnglesData.create())
  await expect(page.getByText('18 of 18 servos limp')).toBeVisible()
  // One servo written, every other limp: its angle is the report's, theirs the model's.
  const angles = Array.from({ length: 18 }, (_, servo) => (servo === 0 ? -20 : 0))
  robot.set(AnglesData, AnglesData.create({ angles, limp: ALL_SERVOS & ~1 }))
  await expect(page.getByText('17 of 18 servos limp')).toBeVisible()

  await page.mouse.move(point!.x, point!.y)
  await page.mouse.down()
  await page.mouse.move(point!.x + 1, point!.y)
  await expect.poll(() => jointFrames(robot).length).toBeGreaterThan(0)
  await page.mouse.up()
  const { servo, degrees } = jointFrames(robot)[0].jointAngle!
  const drawn = servo === 0 ? -20 : HELD[servo]
  expect(Math.abs(degrees - drawn), `servo ${servo} sent ${degrees}`).toBeLessThan(8)
})

// The landing page's view of the same idle robot is a pure view: no leg can be taken, and a drag only turns the camera.
test('no other 3D view drives the joints, even of an idle robot', async ({ page }) => {
  // The landing page's view turns on its own, and each software-rendered turn holds the page.
  test.setTimeout(120_000)
  const robot = await idleRobot(page)
  await page.goto(`${base}/`)
  await expect(page.getByText('Connected', { exact: true })).toBeVisible()
  const canvas = page.locator('canvas').first()

  expect(await scanForJoint(page, canvas, 0.1)).toBeUndefined()
  for (const point of await grid(canvas, 0.25)) await drag(page, point)
  await page.waitForTimeout(500)
  expect(jointFrames(robot)).toHaveLength(0)
})
