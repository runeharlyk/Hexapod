<script lang="ts">
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import { Cancel, Power, Settings as SettingsIcon } from '$lib/components/icons'
  import Advanced from '$lib/components/layout/Advanced.svelte'
  import Field from '$lib/components/layout/Field.svelte'
  import SettingsForm from '$lib/components/layout/SettingsForm.svelte'
  import ChangedNote from '$lib/components/layout/ChangedNote.svelte'
  import { Settings } from '$lib/components/layout/settings.svelte'
  import { topicSource } from '$lib/robot/settings-sources'
  import { PeripheralSettings } from '$lib/platform_shared/api'

  const settings = new Settings<PeripheralSettings>(
    topicSource(PeripheralSettings, {
      what: 'the I2C settings',
      // The bus is opened once at boot, so new pins take effect on the next restart.
      saved: 'I2C configuration saved - restart the robot to apply it.',
      // The firmware rejects out-of-range pins or frequencies with 400 and keeps the stored config.
      update: value => ({
        peripheralSettingsUpdate: {
          ...value,
          sda: Number(value.sda),
          scl: Number(value.scl),
          frequency: Number(value.frequency),
          pins: []
        }
      }),
      // The pin list is one field: a push that changes its length replaces it whole.
      together: [['pins']]
    })
  )

  const confirm = (save: () => void) =>
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
</script>

<SettingsForm
  title="Bus configuration"
  description="The robot opens the bus at boot, so a change takes effect after a restart."
  icon={SettingsIcon}
  {settings}
  rows={1}
  {confirm}
>
  {#snippet children(form, notes)}
    <Field label="Frequency" for="frequency" hint="100000 to 400000 Hz">
      <label class="input validator w-full">
        <input
          id="frequency"
          name="frequency"
          type="number"
          required
          min="100000"
          max="400000"
          bind:value={form.frequency}
        />
        <span class="label">Hz</span>
      </label>
      <ChangedNote {notes} field="frequency" />
    </Field>
    <Advanced
      title="Pins"
      description="Wrong pins leave the robot without servos or IMU until you correct them over serial."
    >
      <Field label="SDA" for="sda" hint="GPIO 0 to 48">
        <input
          id="sda"
          name="sda"
          class="input validator w-full"
          type="number"
          required
          min="0"
          max="48"
          bind:value={form.sda}
        />
        <ChangedNote {notes} field="sda" />
      </Field>
      <Field label="SCL" for="scl" hint="GPIO 0 to 48">
        <input
          id="scl"
          name="scl"
          class="input validator w-full"
          type="number"
          required
          min="0"
          max="48"
          bind:value={form.scl}
        />
        <ChangedNote {notes} field="scl" />
      </Field>
    </Advanced>
  {/snippet}
</SettingsForm>
