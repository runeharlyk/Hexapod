import { expect, test, type Locator, type Page } from '@playwright/test'
import { AnglesData, ModeData, ModesEnum } from '../../src/lib/platform_shared/message'
import { ALL_SERVOS } from '../../src/lib/motion'
import { FakeRobot, serveRobot } from '../fake-robot/playwright'

const base = process.env.BASE_PATH ?? ''

const DRIVING = "Driving the robot's joints"

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

// Probe hover positions in one browser call; the drag itself uses native mouse input.
const scanForJoint = async (canvas: Locator, step: number): Promise<Point | undefined> =>
  canvas.evaluate(async (element, step) => {
    await new Promise<void>(resolve => requestAnimationFrame(() => resolve()))
    const box = element.getBoundingClientRect()
    for (let fy = 0.15; fy < 0.9; fy += step) {
      for (let fx = 0.1; fx < 0.9; fx += step) {
        const point = { x: box.x + box.width * fx, y: box.y + box.height * fy }
        element.dispatchEvent(
          new PointerEvent('pointermove', {
            clientX: point.x,
            clientY: point.y,
            pointerId: 1,
            pointerType: 'mouse',
            bubbles: true
          })
        )
        if ((element as HTMLCanvasElement).style.cursor === 'grab') return point
      }
    }
    return undefined
  }, step)

// Wait for the loaded model to expose a draggable leg before sending native mouse input.
const findJoint = async (canvas: Locator): Promise<Point> => {
  let point: Point | undefined
  await expect
    .poll(
      async () => {
        point = await scanForJoint(canvas, 0.04)
        return point
      },
      { timeout: 10_000 }
    )
    .toBeDefined()
  return point!
}

const drag = async (page: Page, { x, y }: Point) => {
  await page.mouse.move(x, y)
  await page.mouse.down()
  await page.mouse.move(x + 40, y - 30, { steps: 2 })
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

  const point = await findJoint(canvas)
  expect(point, 'a leg of the model under the pointer').toBeDefined()
  await drag(page, point)
  await expect.poll(() => jointFrames(robot).length).toBeGreaterThan(0)
  for (const { jointAngle } of jointFrames(robot)) {
    expect(jointAngle!.servo).toBeLessThan(18)
    expect(Number.isFinite(jointAngle!.degrees)).toBe(true)
  }

  robot.set(ModeData, ModeData.create({ mode: ModesEnum.STAND }))
  await expect(page.getByText(DRIVING)).toHaveCount(0)
  await page.getByText('Adjustments', { exact: true }).click()
  await expect(page.locator('#height')).toBeVisible()
  const sent = jointFrames(robot).length
  await page.mouse.move(point.x, point.y)
  expect(await cursorOf(canvas)).toBe('')
  await drag(page, point)
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

  const point = await findJoint(canvas)
  expect(point, 'a leg of the model under the pointer').toBeDefined()
  await page.mouse.move(point.x, point.y)
  await page.mouse.down()
  await page.mouse.move(point.x + 1, point.y)
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

  robot.set(AnglesData, AnglesData.create())
  await expect(page.getByText('18 of 18 servos limp')).toBeVisible()
  // One servo written, every other limp: its angle is the report's, theirs the model's.
  const angles = Array.from({ length: 18 }, (_, servo) => (servo === 0 ? -20 : 0))
  robot.set(AnglesData, AnglesData.create({ angles, limp: ALL_SERVOS & ~1 }))
  await expect(page.getByText('17 of 18 servos limp')).toBeVisible()

  const point = await findJoint(canvas)
  await page.mouse.move(point.x, point.y)
  await page.mouse.down()
  await page.mouse.move(point.x + 1, point.y)
  await expect.poll(() => jointFrames(robot).length).toBeGreaterThan(0)
  await page.mouse.up()
  const { servo, degrees } = jointFrames(robot)[0].jointAngle!
  const drawn = servo === 0 ? -20 : HELD[servo]
  expect(Math.abs(degrees - drawn), `servo ${servo} sent ${degrees}`).toBeLessThan(8)
})

// The landing page's view of the same idle robot is a pure view: no leg can be taken, and a drag only turns the camera.
test('no other 3D view drives the joints, even of an idle robot', async ({ page }) => {
  const robot = await idleRobot(page)
  robot.set(AnglesData, AnglesData.create({ angles: HELD }))
  await page.goto(`${base}/`)
  await expect(page.getByText('Connected', { exact: true })).toBeVisible()
  const canvas = page.locator('canvas').first()
  await expect
    .poll(() =>
      canvas.evaluate(
        element =>
          (element as HTMLCanvasElement).width === Math.floor(element.getBoundingClientRect().width)
      )
    )
    .toBe(true)

  expect(await scanForJoint(canvas, 0.1)).toBeUndefined()
  for (const point of await grid(canvas, 0.4)) await drag(page, point)
  await page.waitForTimeout(500)
  expect(jointFrames(robot)).toHaveLength(0)
})
