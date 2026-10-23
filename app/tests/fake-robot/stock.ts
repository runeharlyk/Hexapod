import type { Page } from '@playwright/test'
import {
  AnalyticsData,
  AnglesData,
  AnimationStatus,
  CoreDumpReport,
  CorrelationRequest,
  FeaturesDataResponse,
  GaitData,
  GoalStatus,
  IMUData,
  ModeData,
  ModesEnum,
  OtaStatusData,
  StaticSystemInformation,
  TaskState,
  type MessageFns
} from '../../src/lib/platform_shared/message'
import {
  APProvisionMode,
  APSettings,
  APStatus,
  MDNSSettings,
  PeripheralSettings,
  Servo,
  ServoSettings,
  WifiSettings,
  WifiStatus
} from '../../src/lib/platform_shared/api'
import type { StateTopic } from '../../src/lib/transport/topics'
import { FakeRobot, serveRobot } from './playwright'
import { FACTORY_MDNS_SETTINGS, type RequestKey } from './robot'

// The firmware's IPAddress packing, as proto-api's ipToU32, which cannot load outside the app for its store imports.
const ip = (dotted: string) =>
  dotted.split('.').reduceRight((packed, octet) => packed * 256 + Number(octet), 0)

/** A robot on its home network, idle, with the parts the firmware's single build carries. */
export const STOCK = {
  features: FeaturesDataResponse.create({
    firmwareName: 'hexapod',
    firmwareVersion: '0.1.1',
    camera: true,
    imu: true,
    mag: true,
    servo: true,
    mdns: true,
    settingsTopics: true,
    robotId: '240ac4129f3c',
    firmwareElfSha256: '3f1c9a0d'.repeat(8)
  }),
  wifiStatus: WifiStatus.create({
    status: 3,
    localIp: ip('192.168.1.42'),
    macAddress: '24:0A:C4:12:9F:3C',
    rssi: -58,
    ssid: 'home-network',
    bssid: 'A0:63:91:5E:22:10',
    channel: 6,
    subnetMask: ip('255.255.255.0'),
    gatewayIp: ip('192.168.1.1'),
    dnsIp1: ip('192.168.1.1'),
    dnsIp2: ip('1.1.1.1')
  }),
  wifiSettings: WifiSettings.create({
    hostname: 'hexapod',
    priorityRssi: true,
    selectedNetwork: 0,
    wifiNetworks: [
      { ssid: 'home-network', password: 'kitchen-table-42', staticIpConfig: false },
      // As long as an SSID gets, to show how the pages fit one.
      {
        ssid: 'Workshop-Extender-5GHz-Upstairs1',
        password: 'upstairs-relay-7',
        staticIpConfig: false
      }
    ].map(network => ({
      ...network,
      localIp: 0,
      gatewayIp: 0,
      subnetMask: 0,
      dnsIp1: 0,
      dnsIp2: 0
    }))
  }),
  apStatus: APStatus.create({
    status: 1,
    ipAddress: ip('192.168.4.1'),
    macAddress: '24:0A:C4:12:9F:3D',
    stationNum: 0
  }),
  apSettings: APSettings.create({
    provisionMode: APProvisionMode.AP_MODE_DISCONNECTED,
    ssid: 'hexapod-setup',
    password: 'hexapod-setup-key',
    channel: 1,
    ssidHidden: false,
    maxClients: 4,
    localIp: ip('192.168.4.1'),
    gatewayIp: ip('192.168.4.1'),
    subnetMask: ip('255.255.255.0')
  }),
  peripheralSettings: PeripheralSettings.create({ sda: 8, scl: 9, frequency: 400000 }),
  i2cAddresses: [0x1e, 0x40, 0x41, 0x68],
  servos: Array.from({ length: 18 }, (_, id) =>
    Servo.create({
      centerPwm: 306 + ((id * 7) % 13) - 6,
      direction: id % 3 === 2 ? -1 : 1,
      centerAngle: 0,
      conversion: 2.2,
      name: ''
    })
  ),
  staticInfo: StaticSystemInformation.create({
    espPlatform: 'ESP32-S3',
    firmwareVersion: '0.1.1',
    cpuFreqMhz: 240,
    cpuType: 'ESP32-S3',
    cpuCores: 2,
    sdkVersion: 'v5.4.1',
    flashChipSize: 16000000,
    cpuResetReason: 'Power on'
  }),
  coreDump: CoreDumpReport.create({ stored: false })
}

/** The analytics frame `tick` seconds after boot: slow drifts, so a chart of a few frames has a shape. */
export const analyticsAt = (tick: number) =>
  AnalyticsData.create({
    maxAllocHeap: 110000,
    psramSize: 8386295,
    freePsram: 6100000 - tick * 900,
    freeHeap: 142000 - (tick % 7) * 1500,
    totalHeap: 297000,
    minFreeHeap: 120000,
    coreTemp: 41 + Math.sin(tick / 5) * 1.5,
    fsTotal: 1441792,
    fsUsed: 389120,
    uptime: 3600 + tick,
    cpu0Usage: 22 + Math.sin(tick / 3) * 8,
    cpu1Usage: 48 + Math.cos(tick / 4) * 10,
    cpuUsage: 35 + Math.sin(tick / 4) * 9,
    i2cErrors: [{ address: 0x40, nacks: 0, timeouts: 0 }],
    i2cBusRecoveries: 0
  })

/** The IMU frame of `tick`: a robot rocking gently on its feet. */
export const imuAt = (tick: number) =>
  IMUData.create({
    x: Math.sin(tick / 6) * 2.5,
    y: Math.cos(tick / 8) * 1.8,
    z: 0.3 * Math.sin(tick / 4),
    heading: 92 + Math.sin(tick / 10) * 4
  })

interface Held {
  wifiSettings: WifiSettings
  apSettings: APSettings
  servoSettings: ServoSettings
  peripheralSettings: PeripheralSettings
  mdnsSettings: MDNSSettings
}

const HELD_KEYS = [
  'wifiSettings',
  'apSettings',
  'servoSettings',
  'peripheralSettings',
  'mdnsSettings'
] as const satisfies readonly (keyof Held)[]

const SETTINGS_TYPES = {
  wifiSettings: WifiSettings,
  apSettings: APSettings,
  servoSettings: ServoSettings,
  peripheralSettings: PeripheralSettings,
  mdnsSettings: MDNSSettings
} as const satisfies Record<keyof Held, MessageFns<unknown>>

// Each settings update, by the settings it replaces.
const UPDATES = {
  wifiSettingsUpdate: 'wifiSettings',
  apSettingsUpdate: 'apSettings',
  servoSettingsUpdate: 'servoSettings',
  peripheralSettingsUpdate: 'peripheralSettings',
  mdnsSettingsUpdate: 'mdnsSettings'
} as const satisfies Partial<Record<RequestKey, keyof Held>>

export type SettingsUpdate = keyof typeof UPDATES

// The firmware's reply to an mDNS update that names no valid host name (firmware/src/main.cpp).
const MDNS_HOSTNAME_REFUSAL = 'the hostname must be one DNS label: a-z, 0-9 and -'

// firmware/include/settings/servo_settings.h, Servo_isValid.
const servoIsValid = (servo: Servo) =>
  servo.centerPwm >= 0 &&
  servo.centerPwm <= 4095 &&
  servo.conversion > 0 &&
  servo.conversion <= 100 &&
  (servo.direction === 1 || servo.direction === -1) &&
  servo.centerAngle >= -180 &&
  servo.centerAngle <= 180

// firmware/include/settings/mdns_settings.h, MDNSSettings_validHostname: one DNS label of at most the 32 characters the
// wire carries (platform_shared/api.options).
const hostnameIsValid = (hostname: string) => /^[a-z0-9]([a-z0-9-]{0,30}[a-z0-9])?$/.test(hostname)

const withUnderscore = (label: string) =>
  label === '' || label.startsWith('_') ? label : `_${label}`

// firmware/include/settings/mdns_settings.h, MDNSSettings_update: what is stored of a valid update.
const mdnsStored = (update: MDNSSettings): MDNSSettings => ({
  hostname: update.hostname === 'esp32' ? FACTORY_MDNS_SETTINGS.hostname : update.hostname,
  instance: update.instance === 'ESP32 Device' ? FACTORY_MDNS_SETTINGS.instance : update.instance,
  services: update.services
    .map(service => ({
      ...service,
      service: withUnderscore(service.service),
      protocol: withUnderscore(service.protocol)
    }))
    .filter(
      service =>
        service.service !== '' &&
        service.protocol !== '' &&
        service.port > 0 &&
        service.port <= 65535
    ),
  globalTxtRecords: update.globalTxtRecords.filter(record => record.key !== 'Firmware Version')
})

/** The settings as a read, a snapshot, a push or an update's reply carries them: passwords and the secret left out. */
const redacted = <K extends keyof Held>(key: K, value: Held[K]): Held[K] => {
  switch (key) {
    case 'wifiSettings': {
      const wifi = value as WifiSettings
      return {
        ...wifi,
        appliedSecret: 0,
        wifiNetworks: wifi.wifiNetworks.map(network => ({ ...network, password: '' }))
      } as Held[K]
    }
    case 'apSettings':
      return { ...(value as APSettings), password: '' } as Held[K]
    default:
      return value
  }
}

const encoded = <K extends keyof Held>(key: K, value: Held[K]) =>
  Buffer.from((SETTINGS_TYPES[key] as MessageFns<Held[K]>).encode(value).finish()).toString('hex')

const STOCK_HELD: Held = {
  wifiSettings: STOCK.wifiSettings,
  apSettings: STOCK.apSettings,
  servoSettings: ServoSettings.create({ servos: STOCK.servos }),
  peripheralSettings: STOCK.peripheralSettings,
  mdnsSettings: FACTORY_MDNS_SETTINGS
}

/**
 * The stock robot's stored settings, kept and served as the firmware does (firmware/src/main.cpp,
 * firmware/include/settings/settings_topics.h). Each group is a state topic: a subscribe is answered with the stored
 * group and then the ack, and a stored change is pushed to every subscribed client, never an update that leaves the
 * stored value as it was or one that is refused. A read, a snapshot, a push and an update's reply leave the passwords
 * out (`shown`); an update with an empty password keeps the one stored for that SSID; a refused update is a 400 that
 * carries the settings kept.
 */
export class StockRobot extends FakeRobot {
  held: Held

  constructor(boot: Record<string, unknown>, held: Held = STOCK_HELD) {
    super({
      ...boot,
      ...Object.fromEntries(HELD_KEYS.map(key => [key, redacted(key, held[key])]))
    })
    this.held = held
  }

  /** The settings as a read, a snapshot or an update's reply carries them. */
  shown<K extends keyof Held>(key: K): Held[K] {
    return redacted(key, this.held[key])
  }

  /** The robot's own change, or another client's: stored, and pushed to the subscribers when the value differs. */
  hold<K extends keyof Held>(key: K, value: Held[K]) {
    if (encoded(key, value) === encoded(key, this.held[key])) return
    this.held = { ...this.held, [key]: value }
    this.set(SETTINGS_TYPES[key] as StateTopic, this.shown(key) as never)
  }

  /** Refuses the update as the firmware refuses one it cannot store. */
  refuseUpdate(update: SettingsUpdate) {
    const key = UPDATES[update]
    this.respond(update, () => ({ statusCode: 400, [key]: this.shown(key) }))
  }

  /** Stores an update as the firmware does, and answers with what it kept. */
  store(update: SettingsUpdate, request: CorrelationRequest) {
    const key = UPDATES[update]
    const held = this.held
    switch (key) {
      case 'servoSettings': {
        const servos = request.servoSettingsUpdate!
        if (!servos.servos.every(servoIsValid)) return { statusCode: 400, [key]: this.shown(key) }
        this.hold(key, servos)
        break
      }
      case 'wifiSettings': {
        const wifi = request.wifiSettingsUpdate!
        const stored = held.wifiSettings.wifiNetworks
        this.hold(key, {
          ...wifi,
          appliedSecret: held.wifiSettings.appliedSecret,
          wifiNetworks: wifi.wifiNetworks.map(network => ({
            ...network,
            password: network.password || stored.find(s => s.ssid === network.ssid)?.password || ''
          }))
        })
        break
      }
      case 'apSettings': {
        const ap = request.apSettingsUpdate!
        this.hold(key, { ...ap, password: ap.password || held.apSettings.password })
        break
      }
      case 'peripheralSettings':
        this.hold(key, request.peripheralSettingsUpdate!)
        break
      case 'mdnsSettings': {
        const mdns = request.mdnsSettingsUpdate!
        if (!hostnameIsValid(mdns.hostname))
          return { statusCode: 400, errorMessage: MDNS_HOSTNAME_REFUSAL, [key]: this.shown(key) }
        this.hold(key, mdnsStored(mdns))
        break
      }
    }
    return { [key]: this.shown(key) }
  }
}

/**
 * The fake robot answering every request the settings, status and peripheral pages make with the STOCK values, the
 * state topics holding them from the start. A test overrides any answer with `respond` before or after.
 */
export const stockRobot = () => {
  const robot = new StockRobot({
    mode: ModeData.create({ mode: ModesEnum.IDLE }),
    angles: AnglesData.create(),
    gait: GaitData.create(),
    wifiStatus: STOCK.wifiStatus,
    apStatus: STOCK.apStatus,
    otaStatus: OtaStatusData.create(),
    animationStatus: AnimationStatus.create(),
    goalStatus: GoalStatus.create()
  })
  robot.respond('featuresDataRequest', () => ({ featuresDataResponse: STOCK.features }))
  robot.respond('i2cScanDataRequest', () => ({
    i2cScanData: { devices: STOCK.i2cAddresses.map(address => ({ address })) }
  }))
  for (const update of Object.keys(UPDATES) as SettingsUpdate[])
    robot.respond(update, request => robot.store(update, request))
  robot.respond('mdnsQuery', () => ({
    mdnsQueryResponse: {
      services: [
        { name: 'hexapod', ip: '192.168.1.42', port: 80 },
        { name: 'office-colour-laserprinter-second-floor', ip: '192.168.1.17', port: 631 }
      ]
    }
  }))
  robot.respond('systemInformationRequest', () => ({
    systemInformationResponse: {
      staticSystemInformation: STOCK.staticInfo,
      analyticsData: analyticsAt(0)
    }
  }))
  robot.respond('coreDumpGet', () => ({ coreDumpReport: STOCK.coreDump }))
  robot.respond('taskStatsRequest', () => ({
    taskStats: {
      windowUs: 2000000,
      snapshotUs: 180,
      taskCount: 4,
      tasks: [
        ['control', 1, 22, 0, 2100, 31.5],
        ['comm', 0, 12, 1, 2900, 6.2],
        ['IDLE0', 1, 0, 0, 1200, 58.1],
        ['IDLE1', 1, 0, 1, 1250, 61.7]
      ].map(([name, state, priority, core, stack, cpu], number) => ({
        name: name as string,
        number,
        state: state as TaskState,
        priority: priority as number,
        basePriority: priority as number,
        core: core as number,
        stackFreeMin: stack as number,
        cpuPercent: cpu as number
      }))
    }
  }))
  return robot
}

export const CAMERA_SETTINGS = {
  framesize: 6,
  quality: 12,
  brightness: 0,
  contrast: 0,
  saturation: 0,
  sharpness: 0,
  denoise: 0,
  special_effect: 0,
  wb_mode: 0,
  vflip: false,
  hmirror: false
}

const FILES = {
  root: {
    config: { 'wifiSettings.pb': 112, 'servoSettings.pb': 540, 'peripheralSettings.pb': 24 },
    animations: { 'wave.pb': 2210, 'stretch.pb': 1830 }
  }
}

// A 1x1 grey PNG for the camera stream, which a real robot serves as MJPEG.
const GREY_PIXEL = Buffer.from(
  'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mNkaGj4DwAEhgJAr0E6NQAAAABJRU5ErkJggg==',
  'base64'
)

/**
 * Serves the stock robot to the page: the protobuf link through the WebSocket the layout opens on an http origin, and
 * the robot's HTTP endpoints (camera, files) and GitHub's release list through routes, so no page reaches the network.
 */
export const serveStockRobot = async (page: Page, robot = stockRobot()) => {
  await page.route('**/api/camera/settings', route => route.fulfill({ json: CAMERA_SETTINGS }))
  await page.route('**/api/camera/stream', route =>
    route.fulfill({ body: GREY_PIXEL, contentType: 'image/png' })
  )
  await page.route('**/api/files', route => route.fulfill({ json: FILES }))
  await page.route('https://api.github.com/**', route =>
    route.fulfill({
      json: [
        {
          name: 'v0.1.1',
          tag_name: 'v0.1.1',
          html_url: 'https://github.com/runeharlyk/Hexapod/releases/tag/v0.1.1',
          published_at: '2026-10-01T12:00:00Z',
          prerelease: false,
          assets: []
        },
        {
          name: 'v0.1.0',
          tag_name: 'v0.1.0',
          html_url: 'https://github.com/runeharlyk/Hexapod/releases/tag/v0.1.0',
          published_at: '2026-08-14T12:00:00Z',
          prerelease: false,
          assets: []
        }
      ]
    })
  )
  const link = await serveRobot(page, robot)
  return { robot, ...link }
}
