import { expect, test } from '@playwright/test'
import { SystemCommand } from '../../src/lib/platform_shared/message'
import { serveStockRobot, stockRobot } from '../fake-robot/stock'

const base = process.env.BASE_PATH ?? ''

// Each button asks the robot for its own command, and says what the robot answered.
const commands = [
  {
    button: 'Restart',
    confirm: 'Restart',
    command: SystemCommand.SYS_RESTART,
    done: 'The robot is restarting'
  },
  {
    button: 'Factory reset',
    confirm: 'Factory Reset',
    command: SystemCommand.SYS_RESET,
    done: 'The robot is reset to its factory defaults and restarting'
  },
  {
    button: 'Sleep',
    confirm: 'Sleep',
    command: SystemCommand.SYS_SLEEP,
    done: 'The robot is going to sleep'
  }
]

for (const { button, confirm, command, done } of commands)
  test(`${button} sends its command and reports the robot's answer`, async ({ page }) => {
    const robot = stockRobot()
    await serveStockRobot(page, robot)
    await page.goto(`${base}/system/status`)
    await page.getByRole('button', { name: button, exact: true }).click()
    await page.getByRole('dialog').getByRole('button', { name: confirm, exact: true }).click()

    await expect(page.getByText(done, { exact: true })).toBeVisible()
    expect(robot.requests('systemCommand').map(r => r.systemCommand?.command)).toEqual([command])
  })

// The robot answers a reset whose erase failed with a 500 and its reason, and restarts all the same.
test('a reset whose erase failed is reported as a failure', async ({ page }) => {
  const robot = stockRobot()
  robot.refuse('systemCommand', 500, 'Some settings could not be erased; restarting anyway')
  await serveStockRobot(page, robot)
  await page.goto(`${base}/system/status`)
  await page.getByRole('button', { name: 'Factory reset', exact: true }).click()
  await page.getByRole('dialog').getByRole('button', { name: 'Factory Reset', exact: true }).click()

  await expect(
    page.getByText('Some settings could not be erased; restarting anyway', { exact: true })
  ).toBeVisible()
  await expect(
    page.getByText('The robot is reset to its factory defaults and restarting', { exact: true })
  ).toHaveCount(0)
})

// Firmware up to v0.1.1 took the command as a bare message and answers this request with a bare 400.
test('a restart on firmware up to v0.1.1 is named as needing an update', async ({ page }) => {
  const robot = stockRobot()
  robot.refuse('systemCommand', 400)
  await serveStockRobot(page, robot)
  await page.goto(`${base}/system/status`)
  await page.getByRole('button', { name: 'Restart', exact: true }).click()
  await page.getByRole('dialog').getByRole('button', { name: 'Restart', exact: true }).click()

  await expect(
    page.getByText("the robot's firmware does not support a restart; update it", { exact: true })
  ).toBeVisible()
})
