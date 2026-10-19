import { expect, test } from '@playwright/test'
import { fakeRobot, tooOld } from './fake-robot'
import { PeripheralSettings, WifiSettings } from '../../src/lib/platform_shared/api'

const base = process.env.BASE_PATH ?? ''

const outdated = (feature: string) => `the robot's firmware does not support ${feature}; update it`

// A robot whose firmware knows none of the requests answers each with a bare 400. Those without fields can only be
// unknown to it, and the page says the firmware lacks them; the mDNS query has fields, so its 400 is a refusal.
const loads = [
  {
    path: '/peripherals/servo',
    shown: [`Could not read the servo calibration: ${outdated('the servo calibration')}`]
  },
  {
    path: '/peripherals/i2c',
    shown: [
      `Could not read the I2C settings: ${outdated('the I2C settings')}`,
      `Scan failed: ${outdated('the I2C scan')}`
    ]
  },
  {
    path: '/wifi/sta',
    shown: [`Could not read the Wi-Fi settings: ${outdated('the Wi-Fi settings')}`]
  },
  {
    path: '/wifi/ap',
    shown: [`Could not read the access point settings: ${outdated('the access point settings')}`]
  },
  {
    path: '/wifi/mdns',
    shown: [
      `The robot did not report its mDNS status: ${outdated('the mDNS status')}.`,
      'Scan failed: the robot refused the mDNS scan (status 400)'
    ]
  },
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

// A refusal answers the same on every poll, so the scan stops at once and says why.
test('a Wi-Fi scan names the firmware it needs from a robot that does not know the request', async ({
  page
}) => {
  await fakeRobot(page, tooOld)
  await page.goto(`${base}/wifi/sta`)
  await page.getByRole('button', { name: 'Scan for networks' }).click()

  await expect(
    page.getByText(`Wi-Fi scan failed: ${outdated('the Wi-Fi scan')}`, { exact: true }).first()
  ).toBeVisible()
})

// The robot answers a settings update it refuses with a 400 and the settings it kept.
test('a Wi-Fi settings update the robot refuses is reported, not confirmed', async ({ page }) => {
  const settings = WifiSettings.create({ hostname: 'hexapod', priorityRssi: true })
  await fakeRobot(page, request => {
    if (request.wifiSettingsGet) return { wifiSettings: settings }
    if (request.wifiSettingsUpdate) return { statusCode: 400, wifiSettings: settings }
    return tooOld()
  })
  await page.goto(`${base}/wifi/sta`)
  await page.getByRole('button', { name: 'Apply Settings' }).click()

  await expect(
    page.getByText(
      'Failed to update the Wi-Fi settings: the robot refused the Wi-Fi settings (status 400)',
      { exact: true }
    )
  ).toBeVisible()
  await expect(page.getByText('Wi-Fi settings updated.')).toHaveCount(0)
})

test('an access point settings update the robot refuses is reported, not confirmed', async ({
  page
}) => {
  const ip = (a: number, b: number, c: number, d: number) =>
    (a | (b << 8) | (c << 16) | (d << 24)) >>> 0
  const settings = {
    provisionMode: 0,
    ssid: 'hexapod',
    password: 'password',
    channel: 1,
    ssidHidden: false,
    maxClients: 4,
    localIp: ip(192, 168, 4, 1),
    gatewayIp: ip(192, 168, 4, 1),
    subnetMask: ip(255, 255, 255, 0)
  }
  await fakeRobot(page, request => {
    if (request.apSettingsGet) return { apSettings: settings }
    if (request.apSettingsUpdate) return { statusCode: 400, apSettings: settings }
    return tooOld()
  })
  await page.goto(`${base}/wifi/ap`)
  await page.getByRole('button', { name: 'Apply Settings' }).click()

  await expect(
    page.getByText(
      'Failed to update the access point settings: the robot refused the access point settings (status 400)',
      { exact: true }
    )
  ).toBeVisible()
  await expect(page.getByText('Access Point settings updated.')).toHaveCount(0)
})

test('a servo calibration the robot refuses is reported, not confirmed', async ({ page }) => {
  await fakeRobot(page, request =>
    request.servoSettingsGet ? { servoSettings: { servos: [] } } : tooOld()
  )
  await page.goto(`${base}/peripherals/servo`)
  await page.getByRole('button', { name: 'Save calibration' }).click()

  await expect(
    page.getByText(
      'Failed to save the servo calibration: the robot refused the servo calibration (status 400)',
      { exact: true }
    )
  ).toBeVisible()
})

test('an I2C configuration the robot refuses is reported, not confirmed', async ({ page }) => {
  const settings = PeripheralSettings.create({ sda: 1, scl: 2, frequency: 400000 })
  await fakeRobot(page, request =>
    request.peripheralSettingsGet ? { peripheralSettings: settings } : tooOld()
  )
  await page.goto(`${base}/peripherals/i2c`)
  await page.locator('.collapse').filter({ hasText: 'Configuration' }).getByRole('checkbox').check()
  await page.getByRole('button', { name: 'Edit the I2C configuration' }).click()
  await page.getByRole('button', { name: 'Save', exact: true }).click()
  await page.getByRole('button', { name: 'Confirm' }).click()

  await expect(
    page.getByText(
      'The I2C configuration was not saved: the robot refused the I2C configuration (status 400)',
      { exact: true }
    )
  ).toBeVisible()
})
