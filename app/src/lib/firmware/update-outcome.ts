// An update restarts the robot, and a firmware that does not come up rolls back to the previous one
// without a word. The app remembers what it asked for and, once the robot answers again, compares
// the ELF hash it reports (FeaturesDataResponse.firmware_elf_sha256).

const KEY = 'pending-firmware-update'

export type PendingUpdate = {
  // The hash of the image sent, when the app had the image in hand.
  expectedSha256?: string
  // The hash the robot reported before the update.
  previousSha256: string
  requestedAt: number
}

export type UpdateOutcome = 'updated' | 'rolled-back'

// Longer than any flash and restart takes; an older entry is from an update that never finished.
export const PENDING_UPDATE_TTL_MS = 15 * 60 * 1000

const storage = () => {
  try {
    return window.localStorage
  } catch {
    return undefined
  }
}

export const rememberUpdate = (update: PendingUpdate) => {
  try {
    storage()?.setItem(KEY, JSON.stringify(update))
  } catch {
    // Without storage the outcome is simply not reported.
  }
}

export const forgetUpdate = () => {
  try {
    storage()?.removeItem(KEY)
  } catch {
    // Nothing to forget.
  }
}

export const pendingUpdate = (now = Date.now()): PendingUpdate | null => {
  try {
    const raw = storage()?.getItem(KEY)
    if (!raw) return null
    const update = JSON.parse(raw) as PendingUpdate
    if (now - update.requestedAt <= PENDING_UPDATE_TTL_MS) return update
    forgetUpdate()
    return null
  } catch {
    return null
  }
}

// `sawRestart` is whether the link dropped since the update was asked for. Until it has, the old
// image still runs because the update is still being written, which is no rollback.
export const judgeUpdate = (
  update: PendingUpdate,
  reportedSha256: string,
  sawRestart: boolean
): UpdateOutcome | null => {
  if (!reportedSha256) return null
  let outcome: UpdateOutcome
  if (update.expectedSha256) {
    outcome = reportedSha256 === update.expectedSha256 ? 'updated' : 'rolled-back'
  } else if (update.previousSha256) {
    // A download the robot fetched itself: a hash that did not change means the old image runs.
    outcome = reportedSha256 !== update.previousSha256 ? 'updated' : 'rolled-back'
  } else {
    return null
  }
  return outcome === 'rolled-back' && !sawRestart ? null : outcome
}
