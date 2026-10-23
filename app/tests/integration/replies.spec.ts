import { expect, test } from '@playwright/test'
import { fakeRobot, tooOld, tooOldUpToV011 } from './fake-robot'
import { FeaturesDataResponse } from '../../src/lib/platform_shared/message'
import { serveStockRobot, stockRobot, STOCK } from '../fake-robot/stock'
import type { RequestKey } from '../fake-robot/robot'

const base = process.env.BASE_PATH ?? ''

const outdated = (feature: string) => `the robot's firmware does not support ${feature}; update it`

// A robot whose firmware serves no settings topics is not subscribed to for them, and each settings page says so.
const settingsPages = [
  { path: '/peripherals/servo', what: 'the servo calibration' },
  {
    path: '/peripherals/i2c',
    what: 'the I2C settings',
    scan: { request: 'i2cScanDataRequest', what: 'the I2C scan' }
  },
  { path: '/wifi/sta', what: 'the Wi-Fi settings' },
  { path: '/wifi/ap', what: 'the access point settings' },
  {
    path: '/wifi/mdns',
    what: 'the mDNS settings',
    scan: { request: 'mdnsQuery', what: 'the mDNS scan' }
  }
] satisfies { path: string; what: string; scan?: { request: RequestKey; what: string } }[]

for (const { path, what, scan } of settingsPages)
  test(`${path} tells firmware without settings topics from a refusal`, async ({ page }) => {
    const robot = stockRobot()
    robot.respond('featuresDataRequest', () => ({
      featuresDataResponse: FeaturesDataResponse.create({
        ...STOCK.features,
        settingsTopics: false
      })
    }))
    if (scan) robot.refuse(scan.request, 501)
    await serveStockRobot(page, robot)
    await page.goto(`${base}${path}`)

    await expect(
      page.getByText(`Could not read ${what}: ${outdated(what)}`, { exact: true })
    ).toBeVisible()
    if (scan)
      await expect(
        page.getByText(`Scan failed: ${outdated(scan.what)}`, { exact: true })
      ).toBeVisible()
  })

// A robot whose firmware knows none of the requests answers each with a 501, and the pages that still ask say so.
const loads = [
  {
    path: '/system/status',
    shown: [`System status could not be loaded: ${outdated('the system status')}`]
  }
]

for (const { path, shown } of loads)
  test(`${path} tells an outdated firmware from a refusal`, async ({ page }) => {
    await fakeRobot(page, tooOld)
    await page.goto(`${base}${path}`)
    for (const text of shown) await expect(page.getByText(text, { exact: true })).toBeVisible()
  })

// Firmware up to v0.1.1 answers a request it does not know with a bare 400, which it also answers to a malformed one:
// only a request without fields, which cannot be malformed, reads as one the firmware lacks.
test('a firmware up to v0.1.1 is named as too old only for a request without fields', async ({
  page
}) => {
  await fakeRobot(page, tooOldUpToV011)
  await page.goto(`${base}/wifi/mdns`)
  await expect(
    page.getByText('Scan failed: the robot refused the mDNS scan', { exact: true })
  ).toBeVisible()
})

// A refusal answers the same on every poll, so the scan stops at once and says why. The page offers the scan once it
// has read the saved networks it adds to.
test('a Wi-Fi scan names the firmware it needs from a robot that does not know the request', async ({
  page
}) => {
  const robot = stockRobot()
  robot.refuse('wifiNetworksGet', 501)
  await serveStockRobot(page, robot)
  await page.goto(`${base}/wifi/sta`)
  await page.getByRole('button', { name: 'Scan for networks' }).click()

  await expect(
    page.getByText(`Wi-Fi scan failed: ${outdated('the Wi-Fi scan')}`, { exact: true }).first()
  ).toBeVisible()
})

// The robot answers a settings update it refuses with a 400 and the settings it kept.
test('a Wi-Fi settings update the robot refuses is reported, not confirmed', async ({ page }) => {
  const robot = stockRobot()
  robot.refuseUpdate('wifiSettingsUpdate')
  await serveStockRobot(page, robot)
  await page.goto(`${base}/wifi/sta`)
  await page.getByLabel('Host name').fill('hexapod-2')
  await page.getByRole('button', { name: 'Save', exact: true }).click()

  await expect(
    page.getByText('Not saved: the robot refused the Wi-Fi settings', { exact: true })
  ).toBeVisible()
  await expect(page.getByText('Wi-Fi settings updated.')).toHaveCount(0)
})

test('an access point settings update the robot refuses is reported, not confirmed', async ({
  page
}) => {
  const robot = stockRobot()
  robot.refuseUpdate('apSettingsUpdate')
  await serveStockRobot(page, robot)
  await page.goto(`${base}/wifi/ap`)
  await page.getByLabel('SSID', { exact: true }).fill('hexapod-2')
  await page.getByRole('button', { name: 'Save', exact: true }).click()

  await expect(
    page.getByText('Not saved: the robot refused the access point settings', { exact: true })
  ).toBeVisible()
  await expect(page.getByText('Access Point settings updated.')).toHaveCount(0)
})

test('a servo calibration the robot refuses is reported, not confirmed', async ({ page }) => {
  const robot = stockRobot()
  robot.refuseUpdate('servoSettingsUpdate')
  await serveStockRobot(page, robot)
  await page.goto(`${base}/peripherals/servo`)
  await page.getByLabel('Servo 0 centre PWM').fill('310')
  await page.getByRole('button', { name: 'Save', exact: true }).click()

  await expect(
    page.getByText('Not saved: the robot refused the servo calibration', { exact: true })
  ).toBeVisible()
})

test('an I2C configuration the robot refuses is reported, not confirmed', async ({ page }) => {
  const robot = stockRobot()
  robot.refuseUpdate('peripheralSettingsUpdate')
  await serveStockRobot(page, robot)
  await page.goto(`${base}/peripherals/i2c`)
  await page.getByLabel('Frequency').fill('100000')
  await page.getByRole('button', { name: 'Save', exact: true }).click()
  await page.getByRole('button', { name: 'Confirm' }).click()

  await expect(
    page.getByText('Not saved: the robot refused the I2C settings', { exact: true })
  ).toBeVisible()
})
