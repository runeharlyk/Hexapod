import type { MDNSQueryResult, MDNSServiceDef, MDNSTxtRecord } from '$lib/platform_shared/api'
import { compareIp } from '$lib/utilities'
import { createScan } from '$lib/scan'

export type ServiceType = Pick<MDNSServiceDef, 'service' | 'protocol'>

const HTTP: ServiceType = { service: '_http', protocol: '_tcp' }

export const localName = (hostname: string) => `${hostname}.local`

export const serviceTypeLabel = ({ service, protocol }: ServiceType) => `${service}.${protocol}`

export const advertisedLabel = (service: MDNSServiceDef) =>
  `${serviceTypeLabel(service)} on port ${service.port}`

export const txtLabel = ({ key, value }: MDNSTxtRecord) => (value ? `${key}=${value}` : key)

// Every robot and most devices with a web page answer _http._tcp, so it leads; the robot's own
// types follow, where _ws._tcp finds the other hexapods.
export const scanTypes = (advertised: ServiceType[]): ServiceType[] => {
  const types = new Map<string, ServiceType>([[serviceTypeLabel(HTTP), HTTP]])
  for (const { service, protocol } of advertised) {
    types.set(serviceTypeLabel({ service, protocol }), { service, protocol })
  }
  return [...types.values()]
}

// The firmware reports IPv4 addresses only, so a host whose answer carried none arrives with an empty
// address; it goes last rather than first, where an empty string would sort it.
export const sortScanResults = (results: MDNSQueryResult[]) =>
  [...results].sort(
    (a, b) =>
      Number(a.ip === '') - Number(b.ip === '') ||
      (a.ip && b.ip ? compareIp(a.ip, b.ip) : 0) ||
      a.name.localeCompare(b.name)
  )

export type MdnsScanResult = { type: string; results: MDNSQueryResult[] }

export const mdnsScan = createScan<MdnsScanResult>()
