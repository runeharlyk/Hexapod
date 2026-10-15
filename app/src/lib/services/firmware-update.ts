import { api } from '$lib/api'
import type { GithubAsset, GithubRelease } from '$lib/types/models'
import { compareVersions, validate } from 'compare-versions'
import { rememberUpdate } from '$lib/firmware/update-outcome'

const GITHUB_HEADERS = {
  accept: 'application/vnd.github+json',
  'X-GitHub-Api-Version': '2022-11-28'
}

export const fetchLatestRelease = (repository: string) =>
  api.get<GithubRelease>(`https://api.github.com/repos/${repository}/releases/latest`, {
    headers: GITHUB_HEADERS
  })

let releaseCheck: Promise<GithubRelease | undefined> | undefined

// The status bar asks for the newest release at most once per page session. A release index that
// cannot be reached (offline, rate limited) is normal and leaves the answer unknown.
export const latestReleaseOncePerSession = (repository: string) =>
  (releaseCheck ??= fetchLatestRelease(repository).then(
    result => (result.isErr() ? undefined : result.inner),
    () => undefined
  ))

export const fetchReleases = (repository: string) =>
  api.get<GithubRelease[]>(`https://api.github.com/repos/${repository}/releases`, {
    headers: GITHUB_HEADERS
  })

// A release carries `<target>.factory.bin` for the web flasher and `<target>.bin` for OTA; only
// the plain app image may go into an app slot.
export const findFirmwareAsset = (assets: GithubAsset[], builtTarget: string) =>
  assets.find(asset => asset.name === `${builtTarget}.bin`)

// A version that is not semver (a development build, or the flags not yet loaded) is never older
// than anything, so it is not nagged. Semver pre-release rules apply: 0.3.0-rc.1 is older than 0.3.0.
export const newerRelease = (release: GithubRelease, running: string, builtTarget: string) =>
  (
    validate(running) &&
    validate(release.tag_name) &&
    compareVersions(release.tag_name, running) === 1 &&
    findFirmwareAsset(release.assets, builtTarget)
  ) ?
    release
  : undefined

// The robot fetches the image itself (firmware/include/ota_service.h, POST /api/firmware/download).
// `runningSha256` is the hash it reports now, so the app can tell the new image from a rollback.
export const requestFirmwareDownload = async (downloadUrl: string, runningSha256: string) => {
  const result = await api.post('/api/firmware/download', { download_url: downloadUrl })
  if (result.isErr()) console.error('Firmware download request failed:', result.inner)
  else rememberUpdate({ previousSha256: runningSha256, requestedAt: Date.now() })
  return result
}
