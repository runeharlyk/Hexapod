import { beforeEach, describe, it, expect, vi } from 'vitest'
import { Err, Ok } from '../../src/lib/utilities'
import { findFirmwareAsset, newerRelease } from '../../src/lib/services/firmware-update'

vi.mock('../../src/lib/api', () => ({ api: { get: vi.fn(), post: vi.fn() } }))

const asset = (name: string) => ({ name, browser_download_url: `https://example.com/${name}` })

describe('findFirmwareAsset', () => {
  it('picks the app image over the factory image whatever the order', () => {
    const assets = [asset('esp32-wroom-camera.factory.bin'), asset('esp32-wroom-camera.bin')]
    expect(findFirmwareAsset(assets, 'esp32-wroom-camera')?.name).toBe('esp32-wroom-camera.bin')
  })

  it('finds nothing in a release that only carries the factory image', () => {
    expect(
      findFirmwareAsset([asset('esp32-wroom-camera.factory.bin')], 'esp32-wroom-camera')
    ).toBeUndefined()
  })

  it('does not match a target whose name only contains the built target', () => {
    expect(
      findFirmwareAsset([asset('esp32-wroom-camera-v2.bin')], 'esp32-wroom-camera')
    ).toBeUndefined()
  })
})

const release = (tag: string, assets = [asset('esp32-wroom-camera.bin')]) => ({
  name: tag,
  tag_name: tag,
  html_url: `https://example.com/${tag}`,
  published_at: '2026-10-01T00:00:00Z',
  prerelease: false,
  assets
})

describe('newerRelease', () => {
  const target = 'esp32-wroom-camera'

  it('offers a release that is newer than the running firmware', () => {
    expect(newerRelease(release('v0.2.0'), '0.1.0', target)?.tag_name).toBe('v0.2.0')
  })

  it('compares numerically, not as text', () => {
    expect(newerRelease(release('v0.10.0'), '0.9.0', target)).toBeDefined()
    expect(newerRelease(release('v0.9.0'), '0.10.0', target)).toBeUndefined()
  })

  it('offers nothing when the robot runs the release', () => {
    expect(newerRelease(release('v0.2.0'), '0.2.0', target)).toBeUndefined()
  })

  it('offers nothing when the robot is newer than the release index', () => {
    expect(newerRelease(release('v0.2.0'), '0.3.0', target)).toBeUndefined()
    expect(newerRelease(release('v0.2.0'), '0.3.0-rc.1', target)).toBeUndefined()
  })

  it('offers the final release to a robot on its pre-release', () => {
    expect(newerRelease(release('v0.3.0'), '0.3.0-rc.1', target)).toBeDefined()
    expect(newerRelease(release('v0.3.0'), '0.3.0-4-gabc1234', target)).toBeDefined()
  })

  it('does not offer a pre-release over the final release it precedes', () => {
    expect(newerRelease(release('v0.3.0-rc.2'), '0.3.0', target)).toBeUndefined()
  })

  it('never throws on a version that is not semver, and offers nothing', () => {
    expect(newerRelease(release('v0.2.0'), 'dev', target)).toBeUndefined()
    expect(newerRelease(release('v0.2.0'), '', target)).toBeUndefined()
    expect(newerRelease(release('nightly'), '0.1.0', target)).toBeUndefined()
  })

  it('offers nothing when the release has no app image for this target', () => {
    const factoryOnly = release('v0.2.0', [asset('esp32-wroom-camera.factory.bin')])
    expect(newerRelease(factoryOnly, '0.1.0', target)).toBeUndefined()
  })
})

describe('latestReleaseOncePerSession', () => {
  // The once-per-session memory lives in the module, so every test loads a fresh copy of it.
  let get: ReturnType<typeof vi.mocked<typeof import('../../src/lib/api').api.get>>
  let latestReleaseOncePerSession: typeof import('../../src/lib/services/firmware-update').latestReleaseOncePerSession

  beforeEach(async () => {
    vi.resetModules()
    get = vi.mocked((await import('../../src/lib/api')).api.get)
    get.mockReset()
    ;({ latestReleaseOncePerSession } = await import('../../src/lib/services/firmware-update'))
  })

  it('asks the release index once however often it is called', async () => {
    get.mockResolvedValue(Ok.new(release('v0.2.0')) as never)
    const [first, second] = await Promise.all([
      latestReleaseOncePerSession('owner/repo'),
      latestReleaseOncePerSession('owner/repo')
    ])
    await latestReleaseOncePerSession('owner/repo')
    expect(get).toHaveBeenCalledTimes(1)
    expect(first?.tag_name).toBe('v0.2.0')
    expect(second).toBe(first)
  })

  it('stays silent and does not ask again after a failure', async () => {
    get.mockResolvedValue(Err.new(new Error('offline'), 'Request failed') as never)
    await expect(latestReleaseOncePerSession('owner/repo')).resolves.toBeUndefined()
    await latestReleaseOncePerSession('owner/repo')
    expect(get).toHaveBeenCalledTimes(1)
  })

  it('stays silent when the request throws', async () => {
    get.mockRejectedValue(new Error('boom'))
    await expect(latestReleaseOncePerSession('owner/repo')).resolves.toBeUndefined()
  })
})
