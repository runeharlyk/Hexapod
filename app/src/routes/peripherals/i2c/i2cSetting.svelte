<script lang="ts">
  import { onMount } from 'svelte'
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import { Cancel, Edit, EditOff, Power } from '$lib/components/icons'
  import { notifications } from '$lib/components/toasts/notifications'
  import { dataBroker } from '$lib/transport/databroker'
  import { ask } from '$lib/transport/reply'
  import type { PeripheralSettings } from '$lib/platform_shared/api'

  let settings: PeripheralSettings | null = $state(null)
  let isEditing = $state(false)

  const load = async () => {
    const res = await ask(dataBroker, { peripheralSettingsGet: {} }, 'the I2C settings').catch(
      (error: Error) => {
        notifications.error(`Could not read the I2C settings: ${error.message}`, 5000)
        return null
      }
    )
    if (res?.peripheralSettings) settings = res.peripheralSettings
  }
  onMount(load)

  const save = async () => {
    if (!settings) return
    // The firmware rejects out-of-range pins or frequencies with 400 and keeps the stored config.
    const res = await ask(
      dataBroker,
      {
        peripheralSettingsUpdate: {
          ...settings,
          sda: Number(settings.sda),
          scl: Number(settings.scl),
          frequency: Number(settings.frequency),
          pins: []
        }
      },
      'the I2C configuration'
    ).catch((error: Error) => {
      notifications.error(`The I2C configuration was not saved: ${error.message}`, 5000)
      return null
    })
    if (!res) return
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
  <div class="collapse-arrow bg-base-200 border-base-300 rounded-box collapse border">
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
            placeholder="Type a number between 100000 to 400000"
            min="100000"
            max="400000"
            title="I2C frequency in Hz"
            disabled={!isEditing}
            bind:value={settings.frequency}
          />
        </label>
        <div>
          <button
            class="btn btn-outline btn-primary"
            aria-label={isEditing ?
              'Stop editing the I2C configuration'
            : 'Edit the I2C configuration'}
            onclick={() => (isEditing = !isEditing)}
          >
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
