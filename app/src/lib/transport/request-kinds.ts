import type { CorrelationRequest } from '$lib/platform_shared/message'

export type RequestKey = Exclude<keyof CorrelationRequest, 'correlationId'>

/**
 * What a request is, which decides what becomes of it while no link is up (DataBroker.request). An action has a
 * physical effect and fails at once, since the robot it was meant for may have moved on by the time a link comes. A
 * setting is a stored write: it waits SETTING_GRACE_MS, so a link blip goes unnoticed, and then fails as not saved. A
 * read waits for a link until the request's timeout. Motion is no request: it goes out volatile or not at all
 * (DataBroker.emitVolatile).
 */
export type RequestKind = 'action' | 'setting' | 'read'

/** How long a settings write waits for a link before it fails. */
export const SETTING_GRACE_MS = 3000

/** Every request, assigned once: a request the schema gains does not compile until it is given a kind here. */
export const REQUEST_KINDS: Record<RequestKey, RequestKind> = {
  featuresDataRequest: 'read',
  i2cScanDataRequest: 'read',
  systemInformationRequest: 'read',
  wifiSettingsGet: 'read',
  wifiSettingsUpdate: 'setting',
  wifiNetworksGet: 'read',
  apSettingsGet: 'read',
  apSettingsUpdate: 'setting',
  mdnsStatusGet: 'read',
  mdnsQuery: 'read',
  servoSettingsGet: 'read',
  servoSettingsUpdate: 'setting',
  peripheralSettingsGet: 'read',
  peripheralSettingsUpdate: 'setting',
  imuCalibrate: 'action',
  otaStart: 'action',
  otaChunk: 'action',
  otaFinish: 'action',
  // An animation's upload is a stored write; deleting a file is one too.
  fileWriteChunk: 'setting',
  fileReadChunk: 'read',
  fileDelete: 'setting',
  animationValidate: 'read',
  animationListRequest: 'read',
  // With `reset` it zeroes the pose, which moves nothing; the robot refuses it while a goal runs.
  odometryGet: 'read',
  goalMove: 'action',
  goalCancel: 'action',
  taskStatsRequest: 'read',
  coreDumpGet: 'read',
  coreDumpRead: 'read',
  coreDumpErase: 'action'
}

/** The kind of the request a payload carries. */
export const kindOfRequest = (payload: Omit<CorrelationRequest, 'correlationId'>): RequestKind => {
  const key = (Object.keys(payload) as RequestKey[]).find(k => payload[k] !== undefined)
  return key ? REQUEST_KINDS[key] : 'read'
}
