<script lang="ts">
  import { onMount } from 'svelte'
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import { Cancel, Edit, EditOff, Power } from '$lib/components/icons'
  import { notifications } from '$lib/components/toasts/notifications'
  import { dataBroker } from '$lib/transport/databroker'
  import type { PeripheralSettings } from '$lib/platform_shared/api'

  let settings: PeripheralSettings | null = $state(null)
  let isEditing = $state(false)

  const load = async () => {
    const res = await dataBroker.request({ peripheralSettingsGet: {} }).catch(() => null)
    if (res?.peripheralSettings) settings = res.peripheralSettings
  }
  onMount(load)

  const save = async () => {
    if (!settings) return
    const res = await dataBroker
      .request({
        peripheralSettingsUpdate: {
          sda: Number(settings.sda),
          scl: Number(settings.scl),
          frequency: Number(settings.frequency),
          pins: []
        }
      })
      .catch(() => null)

    // The firmware rejects out-of-range pins or frequencies with 400 and keeps the stored config.
    if (!res || res.statusCode >= 400) {
      notifications.error('The robot rejected the I2C configuration.', 4000)
      return
    }
    if (res.peripheralSettings) settings = res.peripheralSettings
    // The bus is opened once at boot, so new pins take effect on the next restart.
    notifications.success('I2C configuration saved - restart the robot to apply it.', 5000)
    isEditing = false
  }

  const handleSave = () => {
    modals.open(ConfirmDialog, {
      title: 'Confirm configuration',
      message:
        'Saving wrong pins leaves the robot without servos or IMU until you correct them over serial. Continue?',
      labels: {
        cancel: { label: 'Cancel', icon: Cancel },
        confirm: { label: 'Confirm', icon: Power }
      },
      onConfirm: () => {
        modals.close()
        save()
      }
    })
  }

  const Icon = $derived(isEditing ? EditOff : Edit)
</script>

{#if settings}
  <div class="collapse bg-base-100 border-base-300 border">
    <input type="checkbox" />
    <div class="collapse-title font-semibold">Configuration</div>
    <div class="collapse-content text-sm">
      <div class="flex flex-col gap-2">
        <label for="sda" class="input validator">
          SDA

          <input
            id="sda"
            type="number"
            required
            placeholder="Type a number between 0 to 48"
            min="0"
            max="48"
            title="SDA pin number (0-48)"
            disabled={!isEditing}
            bind:value={settings.sda}
          />
        </label>
        <label for="scl" class="input validator">
          SCL

          <input
            id="scl"
            type="number"
            required
            placeholder="Type a number between 0 to 48"
            min="0"
            max="48"
            title="SCL pin number (0-48)"
            disabled={!isEditing}
            bind:value={settings.scl}
          />
        </label>
        <label class="input validator" for="frequency">
          Frequency
          <input
            id="frequency"
            type="number"
            required
            placeholder="Type a number between 100000 to 1000000"
            min="100000"
            max="1000000"
            title="I2C frequency in Hz"
            disabled={!isEditing}
            bind:value={settings.frequency}
          />
        </label>
        <div>
          <button class="btn btn-outline btn-primary" onclick={() => (isEditing = !isEditing)}>
            <Icon class="h-6 w-6" />
          </button>
          {#if isEditing}
            <button class="btn btn-outline btn-primary" onclick={handleSave}>Save</button>
          {/if}
        </div>
      </div>
    </div>
  </div>
{/if}
