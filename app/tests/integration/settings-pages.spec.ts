import { expect, test, type Locator, type Page } from '@playwright/test'
import { APSettings, ServoSettings, WifiSettings } from '../../src/lib/platform_shared/api'
import type { FakeRobot } from '../fake-robot/playwright'
import { serveStockRobot, STOCK, stockRobot, type SettingsUpdate } from '../fake-robot/stock'

// Each settings page against the stock robot: it shows what the robot holds, saves only a change, withholds an invalid
// value from the robot, sends the edit, and keeps the edit when the robot refuses it.

const base = process.env.BASE_PATH ?? ''

const save = (page: Page) => page.getByRole('button', { name: 'Save', exact: true })

// What the stock robot shows of its settings: the passwords left out.
const stock = stockRobot()

const errorColour = (page: Page) =>
  page.evaluate(() => {
    const probe = document.createElement('span')
    probe.style.color = 'var(--color-error)'
    document.body.append(probe)
    const colour = getComputedStyle(probe).color
    probe.remove()
    return colour
  })

// What the robot decodes: protobuf floats are float32, so 2.2 arrives as 2.200000047683716.
const overTheWire = <T>(
  codec: { encode(m: T): { finish(): Uint8Array }; decode(b: Uint8Array): T },
  m: T
) => codec.decode(codec.encode(m).finish())

interface SettingsPage {
  name: string
  path: string
  update: SettingsUpdate
  /** The field the test edits, as read, as edited and as out of its constraints. */
  field: (page: Page) => Locator
  read: string
  edited: string
  invalid: string
  /** The field's rule, shown below it; a table cell has none, and is only marked invalid. */
  hint?: string
  /** What the edited save sends. */
  sent: unknown
  /** The toast once the robot stored it, and the reason a refusal gives. */
  saved: string
  refused: string
  /** The update as the robot received it. */
  received: (robot: FakeRobot) => unknown[]
}

const PAGES: SettingsPage[] = [
  {
    name: 'Wi-Fi',
    path: '/wifi/sta',
    update: 'wifiSettingsUpdate',
    field: page => page.getByLabel('Host name'),
    read: 'hexapod',
    edited: 'hexapod-lab',
    invalid: 'hx',
    hint: '3 to 32 characters',
    sent: overTheWire(WifiSettings, { ...stock.shown('wifiSettings'), hostname: 'hexapod-lab' }),
    saved: 'Wi-Fi settings updated.',
    refused: 'the robot refused the Wi-Fi settings',
    received: robot => robot.requests('wifiSettingsUpdate').map(r => r.wifiSettingsUpdate)
  },
  {
    name: 'access point',
    path: '/wifi/ap',
    update: 'apSettingsUpdate',
    field: page => page.getByLabel('SSID', { exact: true }),
    read: 'hexapod-setup',
    edited: 'hexapod-lab',
    invalid: 'hx',
    hint: '3 to 32 characters',
    sent: overTheWire(APSettings, { ...stock.shown('apSettings'), ssid: 'hexapod-lab' }),
    saved: 'Access Point settings updated.',
    refused: 'the robot refused the access point settings',
    received: robot => robot.requests('apSettingsUpdate').map(r => r.apSettingsUpdate)
  },
  {
    name: 'servo calibration',
    path: '/peripherals/servo',
    update: 'servoSettingsUpdate',
    field: page => page.getByLabel('Servo 3 centre PWM'),
    read: String(STOCK.servos[3].centerPwm),
    edited: '320',
    invalid: '700',
    sent: overTheWire(ServoSettings, {
      servos: STOCK.servos.map((servo, id) => (id === 3 ? { ...servo, centerPwm: 320 } : servo))
    }),
    saved: 'Servo calibration saved.',
    refused: 'the robot refused the servo calibration',
    received: robot => robot.requests('servoSettingsUpdate').map(r => r.servoSettingsUpdate)
  }
]

for (const shown of PAGES) {
  test(`the ${shown.name} page shows the settings the robot holds`, async ({ page }) => {
    await serveStockRobot(page)
    await page.goto(`${base}${shown.path}`)
    await expect(shown.field(page)).toHaveValue(shown.read)
  })

  test(`the ${shown.name} page offers Save only for a change`, async ({ page }) => {
    await serveStockRobot(page)
    await page.goto(`${base}${shown.path}`)
    await expect(shown.field(page)).toHaveValue(shown.read)
    await expect(save(page)).toBeDisabled()

    await shown.field(page).fill(shown.edited)
    await expect(save(page)).toBeEnabled()
    await shown.field(page).fill(shown.read)
    await expect(save(page)).toBeDisabled()
  })

  test(`the ${shown.name} page withholds an invalid value from the robot`, async ({ page }) => {
    const { robot } = await serveStockRobot(page)
    await page.goto(`${base}${shown.path}`)
    await shown.field(page).fill(shown.invalid)
    await shown.field(page).blur()

    if (shown.hint) {
      const hint = page.getByText(shown.hint, { exact: true }).first()
      await expect(hint).toHaveCSS('color', await errorColour(page))
    }
    await save(page).click()

    // The browser refuses the submit and points at the field.
    await expect(shown.field(page)).toBeFocused()
    expect(
      await shown.field(page).evaluate(field => (field as HTMLInputElement).validity.valid)
    ).toBe(false)

    // A valid save after it is the first update the robot gets: the link keeps order, so the refused one never left.
    await shown.field(page).fill(shown.edited)
    await save(page).click()
    await expect(page.getByText(shown.saved)).toBeVisible()
    expect(shown.received(robot)).toEqual([shown.sent])
  })

  test(`the ${shown.name} page sends the edited settings`, async ({ page }) => {
    const { robot } = await serveStockRobot(page)
    await page.goto(`${base}${shown.path}`)
    await shown.field(page).fill(shown.edited)
    await save(page).click()

    await expect(page.getByText(shown.saved)).toBeVisible()
    expect(shown.received(robot)).toEqual([shown.sent])
    await expect(shown.field(page)).toHaveValue(shown.edited)
    await expect(save(page)).toBeDisabled()
  })

  test(`the ${shown.name} page keeps the edit and says why when the robot refuses it`, async ({
    page
  }) => {
    const robot = stockRobot()
    robot.refuseUpdate(shown.update)
    await serveStockRobot(page, robot)
    await page.goto(`${base}${shown.path}`)
    await shown.field(page).fill(shown.edited)
    await save(page).click()

    await expect(page.getByText(`Not saved: ${shown.refused}`, { exact: true })).toBeVisible()
    await expect(shown.field(page)).toHaveValue(shown.edited)
    await expect(save(page)).toBeEnabled()
  })
}

test('the servo calibration refuses a conversion of 0, as the robot does', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/servo`)
  const conversion = page.getByLabel('Servo 3 conversion')
  await conversion.fill('0')
  await save(page).click()

  await expect(conversion).toBeFocused()
  expect(await conversion.evaluate(field => (field as HTMLInputElement).validity.valid)).toBe(false)

  await conversion.fill('0.5')
  await save(page).click()
  await expect(page.getByText('Servo calibration saved.')).toBeVisible()
  expect(
    robot.requests('servoSettingsUpdate').map(r => r.servoSettingsUpdate?.servos[3].conversion)
  ).toEqual([0.5])
})

test('the access point page holds an address to the IPv4 rule', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/ap`)
  await page.getByText('Addresses', { exact: true }).click()
  const localIp = page.getByLabel('Local IP')
  await expect(localIp).toHaveValue('192.168.4.1')

  await localIp.fill('192.168.4.300')
  await localIp.blur()
  const hint = page.getByText('An IPv4 address, such as 192.168.1.10').first()
  await expect(hint).toHaveCSS('color', await errorColour(page))
  await save(page).click()
  await expect(localIp).toBeFocused()
  expect(robot.requests('apSettingsUpdate')).toEqual([])

  await localIp.fill('192.168.4.2')
  await save(page).click()
  await expect(page.getByText('Access Point settings updated.')).toBeVisible()
  expect(robot.requests('apSettingsUpdate').map(r => r.apSettingsUpdate?.localIp)).toEqual([
    (2 << 24) + (4 << 16) + (168 << 8) + 192
  ])
})

// The selection is an index into the list, which has to follow its network.
test('deleting the selected network leaves none marked selected', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await expect(page.getByText('Selected', { exact: true })).toHaveCount(1)

  await page.getByRole('button', { name: 'Delete home-network' }).click()
  await page.getByRole('button', { name: 'Delete', exact: true }).click()

  await expect(page.getByRole('button', { name: 'Edit home-network' })).toHaveCount(0)
  await expect(page.getByText('Selected', { exact: true })).toHaveCount(0)
})

test('deleting a network before the selected one keeps the selection on its network', async ({
  page
}) => {
  const robot = stockRobot()
  robot.hold('wifiSettings', { ...robot.held.wifiSettings, selectedNetwork: 1 })
  const { robot: served } = await serveStockRobot(page, robot)
  await page.goto(`${base}/wifi/sta`)

  await page.getByRole('button', { name: 'Delete home-network' }).click()
  await page.getByRole('button', { name: 'Delete', exact: true }).click()
  await save(page).click()

  await expect(page.getByText('Wi-Fi settings updated.')).toBeVisible()
  const sent = served.requests('wifiSettingsUpdate')[0].wifiSettingsUpdate!
  expect(sent.wifiNetworks[sent.selectedNetwork].ssid).toBe('Workshop-Extender-5GHz-Upstairs1')
})

test('connecting to a saved network checks the form first', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await page.getByLabel('Host name').fill('hx')

  await page.getByRole('button', { name: 'Connect to home-network' }).click()

  await expect(page.getByLabel('Host name')).toBeFocused()
  await page.getByLabel('Host name').fill('hexapod-lab')
  await page.getByRole('button', { name: 'Connect to home-network' }).click()
  await expect(page.getByText('Wi-Fi settings updated.')).toBeVisible()
  expect(robot.requests('wifiSettingsUpdate').map(r => r.wifiSettingsUpdate?.hostname)).toEqual([
    'hexapod-lab'
  ])
})

test('a network edited in its dialog waits for Save, which sends it', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  await page.getByRole('button', { name: 'Add' }).click()
  const dialog = page.getByRole('dialog')
  await dialog.getByLabel('SSID').fill('garage')
  await dialog.getByRole('button', { name: 'Add network' }).click()

  await expect(page.getByText('garage', { exact: true })).toBeVisible()
  expect(robot.requests('wifiSettingsUpdate')).toEqual([])
  await save(page).click()

  await expect(page.getByText('Wi-Fi settings updated.')).toBeVisible()
  await expect(save(page)).toBeDisabled()
  expect(
    robot
      .requests('wifiSettingsUpdate')
      .map(r => r.wifiSettingsUpdate?.wifiNetworks.map(n => n.ssid))
  ).toEqual([[...STOCK.wifiSettings.wifiNetworks.map(n => n.ssid), 'garage']])
})

// The camera's settings are a topic like the others: the page follows the robot's snapshot and pushes, and no HTTP route
// carries them. Its sliders hold no value outside their range; the folded numbers are held to the firmware's.
const camera = (page: Page) => ({
  brightness: page.getByLabel('Brightness'),
  contrast: page.getByLabel('Contrast'),
  vflip: page.getByLabel('Vertical flip'),
  quality: page.getByLabel('Quality'),
  sharpness: page.getByLabel('Sharpness'),
  more: page.getByText('More camera settings', { exact: true })
})

test("the camera page shows the robot's settings over the link and saves one", async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/camera`)
  const { brightness, contrast, vflip } = camera(page)
  await expect(brightness).toHaveValue('1')
  await expect(contrast).toHaveValue('-1')
  await expect(vflip).toBeChecked()
  await expect(save(page)).toBeDisabled()
  expect(robot.requests('cameraSettingsUpdate')).toEqual([])

  await brightness.fill('2')
  await save(page).click()

  await expect(page.getByText('Camera settings updated.')).toBeVisible()
  expect(robot.requests('cameraSettingsUpdate').map(r => r.cameraSettingsUpdate)).toEqual([
    { ...STOCK.cameraSettings, brightness: 2 }
  ])
  expect(robot.held.cameraSettings.brightness).toBe(2)
  await expect(save(page)).toBeDisabled()
})

test("a second client's camera change shows in an untouched field", async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/camera`)
  const { brightness, contrast } = camera(page)
  await expect(contrast).toHaveValue('-1')
  await brightness.fill('2')

  robot.hold('cameraSettings', { ...robot.held.cameraSettings, contrast: 1, brightness: -2 })

  await expect(contrast).toHaveValue('1')
  await expect(brightness).toHaveValue('2')
  await expect(page.getByText('changed on the robot: -2')).toBeVisible()
  await expect(page.getByText('changed on the robot')).toHaveCount(1)
})

test('an out-of-range camera value is refused and the draft kept', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/camera`)
  const { quality, more } = camera(page)
  await more.click()
  await quality.fill('64')
  await save(page).click()

  // The browser refuses the submit before the robot is asked.
  await expect(quality).toBeFocused()
  expect(await quality.evaluate(field => (field as HTMLInputElement).validity.valid)).toBe(false)
  expect(robot.requests('cameraSettingsUpdate')).toEqual([])
  await expect(quality).toHaveValue('64')
  await expect(save(page)).toBeEnabled()
})

test("the camera page says why the robot's sensor refuses a value, and keeps the draft", async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/camera`)
  const { sharpness, more } = camera(page)
  await more.click()
  await sharpness.fill('2')
  await save(page).click()

  await expect(
    page.getByText('Not saved: this camera has no sharpness setting; leave it at 0', {
      exact: true
    })
  ).toBeVisible()
  await expect(sharpness).toHaveValue('2')
  await expect(save(page)).toBeEnabled()
  expect(robot.held.cameraSettings).toEqual(STOCK.cameraSettings)
})

test('the camera page works with no HTTP route served', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  const http: string[] = []
  page.on('request', request => {
    const { pathname } = new URL(request.url())
    if (pathname.includes('/api/') && !/\/api\/(ws|camera\/stream)$/.test(pathname))
      http.push(pathname)
  })
  await page.goto(`${base}/peripherals/camera`)
  const { brightness } = camera(page)
  await expect(brightness).toHaveValue('1')
  await brightness.fill('0')
  await save(page).click()

  await expect(page.getByText('Camera settings updated.')).toBeVisible()
  expect(robot.held.cameraSettings.brightness).toBe(0)
  expect(http).toEqual([])
})
