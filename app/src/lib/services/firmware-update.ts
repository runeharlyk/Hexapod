import { api } from '$lib/api'
import type { GithubAsset, GithubRelease } from '$lib/types/models'

const GITHUB_HEADERS = {
  accept: 'application/vnd.github+json',
  'X-GitHub-Api-Version': '2022-11-28'
}

export const fetchLatestRelease = (repository: string) =>
  api.get<GithubRelease>(`https://api.github.com/repos/${repository}/releases/latest`, {
    headers: GITHUB_HEADERS
  })

export const fetchReleases = (repository: string) =>
  api.get<GithubRelease[]>(`https://api.github.com/repos/${repository}/releases`, {
    headers: GITHUB_HEADERS
  })

export const findFirmwareAsset = (assets: GithubAsset[], builtTarget: string) =>
  assets.find(asset => asset.name.includes('.bin') && asset.name.includes(builtTarget))

// The robot fetches the image itself (firmware/include/ota_service.h, POST /api/firmware/download).
export const requestFirmwareDownload = async (downloadUrl: string) => {
  const result = await api.post('/api/firmware/download', { download_url: downloadUrl })
  if (result.isErr()) console.error('Firmware download request failed:', result.inner)
  return result
}
