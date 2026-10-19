import { expect, test } from '@playwright/test'
import { APStatus, WifiStatus } from '../../src/lib/platform_shared/api'
import { FakeRobot, serveRobot } from '../fake-robot/playwright'
import { fakeRobot, tooOld } from './fake-robot'

const base = process.env.BASE_PATH ?? ''

const outdated = (feature: string) => `the robot's firmware does not support ${feature}; update it`

// Firmware from before the snapshots answers pings and requests but never acks a subscribe, so no state the app follows
// ever goes live; the layout says why, in the words a request such firmware lacks gets, rather than failing in silence.
test('a firmware without snapshots is named as too old for the live state', async ({ page }) => {
  await fakeRobot(page, tooOld)
  await page.goto(`${base}/controller`)

  await expect(
    page.getByText(`Robot state: ${outdated('live state')}`, { exact: true })
  ).toBeVisible({ timeout: 15000 })
})

test('the Wi-Fi page shows the status the robot answers the subscribe with, and its changes', async ({
  page
}) => {
  const robot = new FakeRobot()
  robot.set(WifiStatus, WifiStatus.create({ status: 3, ssid: 'home-network', rssi: -60 }))
  await serveRobot(page, robot)
  await page.goto(`${base}/wifi/sta`)
  await expect(page.getByText('home-network', { exact: true })).toBeVisible()

  robot.set(WifiStatus, WifiStatus.create({ status: 3, ssid: 'garden-network', rssi: -70 }))

  await expect(page.getByText('garden-network', { exact: true })).toBeVisible()
})

test('the access point page shows the status the robot answers the subscribe with', async ({
  page
}) => {
  const robot = new FakeRobot()
  robot.set(
    APStatus,
    APStatus.create({ status: 0, macAddress: 'AA:BB:CC:DD:EE:FF', stationNum: 2 })
  )
  await serveRobot(page, robot)
  await page.goto(`${base}/wifi/ap`)

  await expect(page.getByText('AA:BB:CC:DD:EE:FF', { exact: true })).toBeVisible()
})

test('the Wi-Fi and access point pages name a firmware without snapshots as too old', async ({
  page
}) => {
  await fakeRobot(page, tooOld)
  await page.goto(`${base}/wifi/sta`)
  await expect(
    page.getByText(`Could not read the Wi-Fi status: ${outdated('the Wi-Fi status')}`, {
      exact: true
    })
  ).toBeVisible({ timeout: 15000 })

  await page.goto(`${base}/wifi/ap`)
  await expect(
    page.getByText(
      `Could not read the access point status: ${outdated('the access point status')}`,
      { exact: true }
    )
  ).toBeVisible({ timeout: 15000 })
})

// After a drop the status shown is the last the robot sent, which may no longer hold; the page says so.
const LAST_KNOWN = 'The robot is not connected: this is the last status it sent.'

test('the Wi-Fi page marks the status as the last known once the link drops', async ({ page }) => {
  const robot = new FakeRobot()
  robot.set(WifiStatus, WifiStatus.create({ status: 3, ssid: 'home-network', rssi: -60 }))
  const link = await serveRobot(page, robot)
  await page.goto(`${base}/wifi/sta`)
  await expect(page.getByText('home-network', { exact: true })).toBeVisible()
  await expect(page.getByText(LAST_KNOWN)).toHaveCount(0)

  link.drop()

  await expect(page.getByText(LAST_KNOWN)).toBeVisible()
  await expect(page.getByText('home-network', { exact: true })).toBeVisible()
})

test('the access point page marks the status as the last known once the link drops', async ({
  page
}) => {
  const robot = new FakeRobot()
  robot.set(APStatus, APStatus.create({ status: 0, macAddress: 'AA:BB:CC:DD:EE:FF' }))
  const link = await serveRobot(page, robot)
  await page.goto(`${base}/wifi/ap`)
  await expect(page.getByText('AA:BB:CC:DD:EE:FF', { exact: true })).toBeVisible()
  await expect(page.getByText(LAST_KNOWN)).toHaveCount(0)

  link.drop()

  await expect(page.getByText(LAST_KNOWN)).toBeVisible()
  await expect(page.getByText('AA:BB:CC:DD:EE:FF', { exact: true })).toBeVisible()
})
