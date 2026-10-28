<script lang="ts">
  import { Settings as SettingsIcon } from '$lib/components/icons'
  import Advanced from '$lib/components/layout/Advanced.svelte'
  import ChangedNote from '$lib/components/layout/ChangedNote.svelte'
  import Field from '$lib/components/layout/Field.svelte'
  import SettingsForm from '$lib/components/layout/SettingsForm.svelte'
  import { Settings } from '$lib/components/layout/settings.svelte'
  import { topicSource } from '$lib/robot/settings-sources'
  import { CameraSettings } from '$lib/platform_shared/api'

  const SPECIAL_EFFECTS = [
    'No effect',
    'Negative',
    'Grayscale',
    'Red tint',
    'Green tint',
    'Blue tint',
    'Sepia'
  ]

  const WHITE_BALANCES = ['Auto', 'Sunny', 'Cloudy', 'Office', 'Home']

  const settings = new Settings<CameraSettings>(
    topicSource(CameraSettings, {
      what: 'the camera settings',
      saved: 'Camera settings updated.',
      // The firmware refuses a value out of range, or one the fitted sensor has no setting for, with 400 and a reason.
      update: value => ({ cameraSettingsUpdate: value }),
      together: []
    })
  )

  const named = (names: string[]) => (value: unknown) => names[Number(value)]
  const onOff = (value: unknown) => (value ? 'on' : 'off')
</script>

<SettingsForm title="Camera settings" icon={SettingsIcon} {settings} rows={6}>
  {#snippet children(form, notes)}
    {#snippet slider(
      id: string,
      label: string,
      min: number,
      max: number,
      key: 'brightness' | 'contrast' | 'framesize'
    )}
      <Field {label} for={id}>
        <input
          {id}
          name={key}
          class="range range-sm w-full"
          type="range"
          {min}
          {max}
          bind:value={form[key]}
        />
        <p class="mt-1 font-mono text-xs">{form[key]}</p>
        <ChangedNote {notes} field={key} />
      </Field>
    {/snippet}
    {@render slider('camera-brightness', 'Brightness', -2, 2, 'brightness')}
    {@render slider('camera-contrast', 'Contrast', -2, 2, 'contrast')}
    {@render slider('camera-framesize', 'Frame size', 0, 10, 'framesize')}
    <Field label="Vertical flip" for="camera-vflip">
      <input
        id="camera-vflip"
        name="vflip"
        type="checkbox"
        class="toggle"
        bind:checked={form.vflip}
      />
      <ChangedNote {notes} field="vflip" format={onOff} />
    </Field>
    <Field label="Horizontal flip" for="camera-hmirror">
      <input
        id="camera-hmirror"
        name="hmirror"
        type="checkbox"
        class="toggle"
        bind:checked={form.hmirror}
      />
      <ChangedNote {notes} field="hmirror" format={onOff} />
    </Field>
    <Field label="Special effect" for="camera-effect">
      <select
        id="camera-effect"
        name="specialEffect"
        class="select w-full"
        bind:value={form.specialEffect}
      >
        {#each SPECIAL_EFFECTS as effect, value (value)}
          <option {value}>{effect}</option>
        {/each}
      </select>
      <ChangedNote {notes} field="specialEffect" format={named(SPECIAL_EFFECTS)} />
    </Field>
    <Advanced title="More camera settings">
      <Field
        label="Quality"
        for="camera-quality"
        hint="0 to 63; a lower number is a sharper image, but a low one can make a VGA frame outgrow its 61440-byte buffer and stop the stream, which a reboot does not undo"
      >
        <input
          id="camera-quality"
          name="quality"
          class="input validator w-full"
          type="number"
          required
          min="0"
          max="63"
          bind:value={form.quality}
        />
        <ChangedNote {notes} field="quality" />
      </Field>
      <Field label="Saturation" for="camera-saturation" hint="-2 to 2">
        <input
          id="camera-saturation"
          name="saturation"
          class="input validator w-full"
          type="number"
          required
          min="-2"
          max="2"
          bind:value={form.saturation}
        />
        <ChangedNote {notes} field="saturation" />
      </Field>
      <Field label="White balance" for="camera-wb-mode">
        <select id="camera-wb-mode" name="wbMode" class="select w-full" bind:value={form.wbMode}>
          {#each WHITE_BALANCES as balance, value (value)}
            <option {value}>{balance}</option>
          {/each}
        </select>
        <ChangedNote {notes} field="wbMode" format={named(WHITE_BALANCES)} />
      </Field>
      <Field label="Sharpness" for="camera-sharpness" hint="-3 to 3; a camera without it keeps 0">
        <input
          id="camera-sharpness"
          name="sharpness"
          class="input validator w-full"
          type="number"
          required
          min="-3"
          max="3"
          bind:value={form.sharpness}
        />
        <ChangedNote {notes} field="sharpness" />
      </Field>
      <Field label="Denoise" for="camera-denoise" hint="0 to 8; a camera without it keeps 0">
        <input
          id="camera-denoise"
          name="denoise"
          class="input validator w-full"
          type="number"
          required
          min="0"
          max="8"
          bind:value={form.denoise}
        />
        <ChangedNote {notes} field="denoise" />
      </Field>
    </Advanced>
  {/snippet}
</SettingsForm>
