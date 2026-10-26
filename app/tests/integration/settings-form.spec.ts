import { expect, test, type Page } from '@playwright/test'
import {
  MDNSSettings,
  PeripheralSettings,
  ServoSettings,
  WifiSettings
} from '../../src/lib/platform_shared/api'
import {
  CorrelationRequest,
  FeaturesDataResponse,
  Message,
  type MessageFns
} from '../../src/lib/platform_shared/message'
import { encodeMessage, tagOf } from '../../src/lib/transport/message-codec'
import { serveStockRobot, stockRobot, STOCK, type StockRobot } from '../fake-robot/stock'

// The shared settings form (lib/components/layout), as the I2C page uses it against the stock robot, and the five
// settings pages following their topics.

const base = process.env.BASE_PATH ?? ''

const form = (page: Page) =>
  page.locator('form').filter({ has: page.getByRole('heading', { name: 'Bus configuration' }) })
const save = (page: Page) => form(page).getByRole('button', { name: 'Save', exact: true })
const reset = (page: Page) => form(page).getByRole('button', { name: 'Reset', exact: true })
const frequency = (page: Page) => page.getByLabel('Frequency')

test('the form shows a skeleton while the robot reads the settings, then the fields', async ({
  page
}) => {
  const robot = stockRobot()
  robot.ackless()
  await serveStockRobot(page, robot)
  await page.goto(`${base}/peripherals/i2c`)

  const loading = form(page).getByRole('status', { name: 'Reading the I2C settings' })
  await expect(loading.locator('.skeleton').first()).toBeVisible()
  await expect(frequency(page)).toHaveCount(0)
  await expect(save(page)).toHaveCount(0)

  robot.set(PeripheralSettings, STOCK.peripheralSettings)

  await expect(frequency(page)).toHaveValue('400000')
  await expect(loading).toHaveCount(0)
})

test('save and reset stay disabled until a field changes, and reset restores the read value', async ({
  page
}) => {
  await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await expect(frequency(page)).toHaveValue('400000')
  await expect(save(page)).toBeDisabled()
  await expect(reset(page)).toBeDisabled()

  await frequency(page).fill('100000')
  await expect(save(page)).toBeEnabled()
  await expect(reset(page)).toBeEnabled()

  // Typing the read value back is no change.
  await frequency(page).fill('400000')
  await expect(save(page)).toBeDisabled()

  await frequency(page).fill('100000')
  await reset(page).click()
  await expect(frequency(page)).toHaveValue('400000')
  await expect(save(page)).toBeDisabled()
  await expect(reset(page)).toBeDisabled()
})

test('a saved value is sent once confirmed and becomes the one reset returns to', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await frequency(page).fill('100000')
  await save(page).click()
  await page.getByRole('button', { name: 'Confirm' }).click()

  await expect(
    page.getByText('I2C configuration saved - restart the robot to apply it.')
  ).toBeVisible()
  expect(robot.requests('peripheralSettingsUpdate').map(r => r.peripheralSettingsUpdate)).toEqual([
    PeripheralSettings.create({ ...STOCK.peripheralSettings, frequency: 100000, pins: [] })
  ])
  await expect(save(page)).toBeDisabled()
  await expect(frequency(page)).toHaveValue('100000')

  await frequency(page).fill('200000')
  await reset(page).click()
  await expect(frequency(page)).toHaveValue('100000')
})

const errorColour = (page: Page) =>
  page.evaluate(() => {
    const probe = document.createElement('span')
    probe.style.color = 'var(--color-error)'
    document.body.append(probe)
    const colour = getComputedStyle(probe).color
    probe.remove()
    return colour
  })

test('a value outside its constraints turns its hint to the error colour', async ({ page }) => {
  await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  const hint = form(page).getByText('100000 to 400000 Hz', { exact: true })
  await expect(hint).toBeVisible()
  const red = await errorColour(page)
  await expect(hint).not.toHaveCSS('color', red)

  await frequency(page).fill('50')
  await frequency(page).blur()

  await expect(hint).toHaveCSS('color', red)
})

test('a save with an invalid folded field opens its section and sends nothing', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await expect(frequency(page)).toHaveValue('400000')
  const pins = form(page).locator('details')
  await expect(pins).not.toHaveAttribute('open')

  await pins.getByText('Pins', { exact: true }).click()
  await page.getByLabel('SDA').fill('99')
  await pins.getByText('Pins', { exact: true }).click()
  await expect(pins).not.toHaveAttribute('open')
  await save(page).click()

  await expect(pins).toHaveAttribute('open')
  await expect(page.getByLabel('SDA')).toBeFocused()
  await expect(page.getByRole('button', { name: 'Confirm' })).toHaveCount(0)
  expect(robot.requests('peripheralSettingsUpdate')).toEqual([])
})

// Another client of the robot: sends requests as the app does, and hears the topics it subscribed to.
const anotherClient = (robot: StockRobot) => {
  const heard: Message[] = []
  const client = { deliver: (bytes: Uint8Array) => heard.push(Message.decode(bytes)) }
  robot.connect(client)
  return {
    heard,
    subscribe: (topic: MessageFns<unknown>) =>
      robot.receive(client, encodeMessage(Message.create({ subNotif: { tag: tagOf(topic) } }))),
    send: (request: Omit<CorrelationRequest, 'correlationId'>) =>
      robot.receive(
        client,
        encodeMessage(
          Message.create({
            correlationRequest: CorrelationRequest.create({ correlationId: 1, ...request })
          })
        )
      )
  }
}

test("a second client's change shows in untouched fields and as a note on a changed one", async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/servo`)
  const centre = (servo: number) => page.getByLabel(`Servo ${servo} centre PWM`)
  await expect(centre(3)).toHaveValue('308')
  await centre(3).fill('320')

  const servos = robot.held.servoSettings.servos.map((servo, id) =>
    id === 3 ? { ...servo, centerPwm: 312 }
    : id === 5 ? { ...servo, centerPwm: 330 }
    : servo
  )
  anotherClient(robot).send({ servoSettingsUpdate: ServoSettings.create({ servos }) })

  await expect(centre(5)).toHaveValue('330')
  await expect(centre(3)).toHaveValue('320')
  await expect(page.getByText('changed on the robot: 312')).toBeVisible()
  await expect(page.getByText('changed on the robot')).toHaveCount(1)
})

test('a save after a push keeps the stored password', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  const hostname = page.getByLabel('Host name')
  await expect(hostname).toHaveValue('hexapod')

  const renamed = 'Workshop-Extender-Renamed'
  const [home, extender] = robot.held.wifiSettings.wifiNetworks
  anotherClient(robot).send({
    wifiSettingsUpdate: WifiSettings.create({
      ...robot.held.wifiSettings,
      wifiNetworks: [home, { ...extender, ssid: renamed, password: 'upstairs-relay-8' }]
    })
  })
  await expect(page.getByText(renamed)).toBeVisible()

  await hostname.fill('hexapod-two')
  await page
    .locator('form')
    .filter({ has: page.getByRole('heading', { name: 'Saved networks' }) })
    .getByRole('button', { name: 'Save', exact: true })
    .click()
  await expect(page.getByText('Wi-Fi settings updated.')).toBeVisible()

  const { hostname: stored, wifiNetworks } = robot.held.wifiSettings
  expect(stored).toBe('hexapod-two')
  expect(wifiNetworks.map(({ ssid, password }) => [ssid, password])).toEqual([
    ['home-network', 'kitchen-table-42'],
    [renamed, 'upstairs-relay-8']
  ])
})

test('the mDNS page renames the robot', async ({ page }) => {
  const { robot } = await serveStockRobot(page)
  const listener = anotherClient(robot)
  listener.subscribe(MDNSSettings)
  await page.goto(`${base}/wifi/mdns`)
  const hostname = page.getByLabel('Host name')
  await expect(hostname).toHaveValue('hexapod')

  await hostname.fill('hexapod-two')
  await page.getByRole('button', { name: 'Save', exact: true }).click()

  await expect(page.getByText('mDNS settings saved.')).toBeVisible()
  expect(robot.held.mdnsSettings.hostname).toBe('hexapod-two')
  await expect
    .poll(() => listener.heard.filter(m => m.mdnsSettings).at(-1)?.mdnsSettings?.hostname)
    .toBe('hexapod-two')
  await expect(hostname).toHaveValue('hexapod-two')
  await expect(page.getByRole('button', { name: 'Save', exact: true })).toBeDisabled()
})

test('a page on firmware without settings topics says the firmware is too old', async ({
  page
}) => {
  const robot = stockRobot()
  robot.respond('featuresDataRequest', () => ({
    featuresDataResponse: FeaturesDataResponse.create({ ...STOCK.features, settingsTopics: false })
  }))
  await serveStockRobot(page, robot)
  await page.goto(`${base}/peripherals/servo`)

  await expect(
    page.getByText(
      "Could not read the servo calibration: the robot's firmware does not support the servo calibration; update it"
    )
  ).toBeVisible()
  expect(robot.subscriptions()).not.toContain(tagOf(ServoSettings))
})

test('a list the user edited keeps their edit and names the robot list that changed under it', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  const networks = page
    .locator('form')
    .filter({ has: page.getByRole('heading', { name: 'Saved networks' }) })
  await expect(networks.getByText('home-network')).toBeVisible()
  await page.getByRole('button', { name: 'Delete home-network' }).click()
  await page.getByRole('button', { name: 'Delete', exact: true }).click()
  await expect(networks.getByText('home-network')).toHaveCount(0)

  const [home, extender] = robot.held.wifiSettings.wifiNetworks
  anotherClient(robot).send({
    wifiSettingsUpdate: WifiSettings.create({
      ...robot.held.wifiSettings,
      wifiNetworks: [home, extender, { ...extender, ssid: 'garage', password: 'g' }]
    })
  })

  await expect(
    page.getByText(`changed on the robot: home-network, ${extender.ssid}, garage`)
  ).toBeVisible()
  await expect(networks.getByText('garage', { exact: true })).toHaveCount(0)
})

test('a network edited while the robot reorders the list is written to the network the editor opened', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/sta`)
  const networks = page
    .locator('form')
    .filter({ has: page.getByRole('heading', { name: 'Saved networks' }) })
  await expect(networks.getByText('home-network')).toBeVisible()
  await page.getByRole('button', { name: 'Edit home-network' }).click()
  const dialog = page.getByRole('dialog')
  await expect(dialog.getByLabel('SSID')).toHaveValue('home-network')

  const [home, extender] = robot.held.wifiSettings.wifiNetworks
  anotherClient(robot).send({
    wifiSettingsUpdate: WifiSettings.create({
      ...robot.held.wifiSettings,
      wifiNetworks: [extender, home]
    })
  })
  await expect.poll(() => robot.held.wifiSettings.wifiNetworks[0].ssid).toBe(extender.ssid)
  await dialog.getByLabel('SSID').fill('home-renamed')
  await dialog.getByRole('button', { name: 'Update network' }).click()

  await expect(networks.getByText('home-renamed', { exact: true })).toBeVisible()
  await expect(networks.getByText(extender.ssid, { exact: true })).toBeVisible()
  await expect(networks.getByText('home-network', { exact: true })).toHaveCount(0)
  await expect(page.getByText(`changed on the robot: ${extender.ssid}, home-network`)).toBeVisible()
})

test('the mDNS editor stops at the caps the wire carries and at a host name the robot refuses', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/mdns`)
  const hostname = page.getByLabel('Host name')
  await expect(hostname).toHaveValue('hexapod')
  await expect(hostname).toHaveAttribute('maxlength', '32')

  const addService = page.getByRole('button', { name: 'Add service' })
  await addService.click()
  await expect(addService).toBeDisabled()
  const addRecord = page.getByRole('button', { name: 'Add record' })
  await addRecord.click()
  await addRecord.click()
  await expect(addRecord).toBeDisabled()
  await page.getByRole('button', { name: 'Add TXT record' }).first().click()
  await expect(page.getByRole('button', { name: 'Add TXT record' })).toHaveCount(2)

  await hostname.fill('Hexapod Two')
  await page.getByRole('button', { name: 'Save', exact: true }).click()
  await expect(hostname).toBeFocused()
  expect(robot.requests('mdnsSettingsUpdate')).toEqual([])
})

test("the access point page follows the robot's address and notes a changed SSID", async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/ap`)
  const ssid = page.getByLabel('SSID', { exact: true })
  const localIp = page.getByLabel('Local IP')
  await expect(localIp).toHaveValue('192.168.4.1')
  await ssid.fill('mine-setup')

  const address = (192 | (168 << 8) | (7 << 16) | (1 << 24)) >>> 0
  anotherClient(robot).send({
    apSettingsUpdate: {
      ...robot.held.apSettings,
      ssid: 'theirs-setup',
      localIp: address,
      gatewayIp: address
    }
  })

  await expect(localIp).toHaveValue('192.168.7.1')
  await expect(ssid).toHaveValue('mine-setup')
  await expect(page.getByText('changed on the robot: theirs-setup')).toBeVisible()
})

test('a push while an address holds invalid text keeps the text, and the browser check still stops a save', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/wifi/ap`)
  await page.getByText('Addresses', { exact: true }).click()
  const localIp = page.getByLabel('Local IP')
  await localIp.fill('192.168.4.300')
  await localIp.blur()

  anotherClient(robot).send({
    apSettingsUpdate: { ...robot.held.apSettings, maxClients: 6, localIp: 0x0107a8c0 }
  })

  await expect(page.getByLabel('Max clients')).toHaveValue('6')
  await expect(localIp).toHaveValue('192.168.4.300')
  const updatesBefore = robot.requests('apSettingsUpdate').length
  await page.getByRole('button', { name: 'Save', exact: true }).click()
  await expect(localIp).toBeFocused()
  expect(robot.requests('apSettingsUpdate')).toHaveLength(updatesBefore)
})

test('a push that shortens a list the page does not edit leaves the form clean', async ({
  page
}) => {
  const { robot } = await serveStockRobot(page)
  await page.goto(`${base}/peripherals/i2c`)
  await expect(frequency(page)).toHaveValue('400000')

  const pin = (number: number) => ({ pin: number, mode: 'output', type: 'digital', role: 'led' })
  const other = anotherClient(robot)
  const settings = robot.held.peripheralSettings
  other.send({ peripheralSettingsUpdate: { ...settings, pins: [pin(4), pin(5)] } })
  other.send({ peripheralSettingsUpdate: { ...settings, pins: [pin(4)] } })
  await expect.poll(() => robot.held.peripheralSettings.pins.length).toBe(1)

  await expect(save(page)).toBeDisabled()
})
