<script lang="ts">
  import { api } from '$lib/api'
  import { Settings as SettingsIcon } from '$lib/components/icons'
  import Field from '$lib/components/layout/Field.svelte'
  import SettingsForm from '$lib/components/layout/SettingsForm.svelte'
  import { Settings } from '$lib/components/layout/settings.svelte'
  import type { CameraSettings } from '$lib/types/models'

  const SPECIAL_EFFECTS = [
    'No effect',
    'Negative',
    'Grayscale',
    'Red tint',
    'Green tint',
    'Blue tint',
    'Sepia'
  ]

  // The camera's settings are served over the robot's HTTP API, not the protobuf link.
  const settings = new Settings<CameraSettings>({
    what: 'the camera settings',
    read: async () => {
      const result = await api.get<CameraSettings | null>('/api/camera/settings')
      if (result.isErr()) throw new Error(String(result.exception))
      // A reply that is not JSON reads as null.
      if (!result.inner) throw new Error('the reply carried no settings')
      return result.inner
    },
    write: async value => {
      const result = await api.post<CameraSettings>('/api/camera/settings', value)
      if (result.isErr()) throw new Error(String(result.exception))
      return result.inner ?? undefined
    },
    saved: 'Camera settings updated.'
  })
</script>

<SettingsForm title="Camera settings" icon={SettingsIcon} {settings} rows={6}>
  {#snippet children(form)}
    {#snippet slider(
      id: string,
      label: string,
      min: number,
      max: number,
      key: 'brightness' | 'contrast' | 'framesize'
    )}
      <Field {label} for={id}>
        <input {id} class="range range-sm w-full" type="range" {min} {max} bind:value={form[key]} />
        <p class="mt-1 font-mono text-xs">{form[key]}</p>
      </Field>
    {/snippet}
    {@render slider('camera-brightness', 'Brightness', -2, 2, 'brightness')}
    {@render slider('camera-contrast', 'Contrast', -2, 2, 'contrast')}
    {@render slider('camera-framesize', 'Frame size', 0, 10, 'framesize')}
    <Field label="Vertical flip" for="camera-vflip">
      <input id="camera-vflip" type="checkbox" class="toggle" bind:checked={form.vflip} />
    </Field>
    <Field label="Horizontal flip" for="camera-hmirror">
      <input id="camera-hmirror" type="checkbox" class="toggle" bind:checked={form.hmirror} />
    </Field>
    <Field label="Special effect" for="camera-effect">
      <select id="camera-effect" class="select w-full" bind:value={form.special_effect}>
        {#each SPECIAL_EFFECTS as effect, value (value)}
          <option {value}>{effect}</option>
        {/each}
      </select>
    </Field>
  {/snippet}
</SettingsForm>
