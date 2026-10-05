import { get } from 'svelte/store'
import { isLinked } from '$lib/stores/link'
import { refreshFeatureFlags } from '$lib/stores/featureFlags'
import { notifications } from '$lib/components/toasts/notifications'
import { forgetUpdate, judgeUpdate, pendingUpdate } from './update-outcome'

// Reports how a pending update ended each time the link comes back. Returns the unsubscriber.
export function watchUpdateOutcome() {
  let sawRestart = false
  let wasLinked = get(isLinked)
  return isLinked.subscribe(async linked => {
    if (wasLinked && !linked) sawRestart = true
    wasLinked = linked
    if (!linked) return
    const update = pendingUpdate()
    if (!update) return
    try {
      const features = await refreshFeatureFlags()
      const outcome = judgeUpdate(update, features.firmware_elf_sha256, sawRestart)
      if (!outcome) return
      forgetUpdate()
      if (outcome === 'updated') {
        notifications.success(`Firmware ${features.firmware_version} is running.`, 8000)
      } else {
        notifications.error(
          'The new firmware did not come up, so the robot rolled back to the previous one.',
          10000
        )
      }
    } catch (error) {
      console.warn('Could not read the firmware after the update:', error)
    }
  })
}
