<script lang="ts">
  import Section from '$lib/components/layout/Section.svelte'
  import { Save, DownloadIcon, UploadIcon } from '$lib/components/icons'
  import {
    applyConfig,
    exportConfig,
    parseConfig,
    serializeConfig,
    type RobotConfig
  } from '$lib/robot-config'

  type Step =
    | { kind: 'idle' }
    | { kind: 'busy'; doing: string }
    | { kind: 'chosen'; config: RobotConfig }
    | { kind: 'done'; message: string }
    | { kind: 'failed'; message: string }

  let step = $state<Step>({ kind: 'idle' })
  let fileInput: HTMLInputElement | undefined = $state()

  const working = $derived(step.kind === 'busy')

  const fail = (error: unknown) => (step = { kind: 'failed', message: (error as Error).message })

  const download = (text: string, filename: string) => {
    const link = document.createElement('a')
    link.href = URL.createObjectURL(new Blob([text], { type: 'application/json' }))
    link.download = filename
    link.click()
    setTimeout(() => URL.revokeObjectURL(link.href))
  }

  async function save() {
    step = { kind: 'busy', doing: 'Reading the settings' }
    try {
      const config = await exportConfig()
      download(serializeConfig(config), `hexapod-config-${config.exportedAt.slice(0, 10)}.json`)
      step = { kind: 'done', message: 'Configuration saved.' }
    } catch (error) {
      fail(error)
    }
  }

  async function pickFile(input: HTMLInputElement) {
    const file = input.files?.[0]
    input.value = ''
    if (!file) return
    try {
      step = { kind: 'chosen', config: parseConfig(await file.text()) }
    } catch (error) {
      fail(error)
    }
  }

  async function load(config: RobotConfig) {
    step = { kind: 'busy', doing: 'Writing the settings' }
    try {
      await applyConfig(config)
      step = {
        kind: 'done',
        message: 'Configuration loaded. New I2C pins take effect when the robot restarts.'
      }
    } catch (error) {
      fail(error)
    }
  }
</script>

<Section
  title="Configuration file"
  description="Saves the servo calibration and the peripheral settings (I2C bus and pins) to a file, and loads them back. A factory reset or a filesystem upload erases both, so keep a copy. WiFi settings stay on the robot: it never sends its passwords."
  icon={Save}
>
  <div class="flex flex-wrap gap-2">
    <button class="btn btn-primary" disabled={working} onclick={save}>
      <DownloadIcon class="size-5" />Save to a file
    </button>
    <button class="btn" disabled={working} onclick={() => fileInput?.click()}>
      <UploadIcon class="size-5" />Load a file
    </button>
    <input
      type="file"
      accept=".json,application/json"
      class="hidden"
      bind:this={fileInput}
      onchange={e => pickFile(e.currentTarget)}
    />
  </div>

  {#if step.kind === 'busy'}
    <p class="text-sm">{step.doing}...</p>
  {:else if step.kind === 'chosen'}
    {@const config = step.config}
    <p class="text-sm">
      Saved {config.exportedAt.slice(0, 10) || 'at an unknown date'}, with {config.servo.servos
        .length} servos. Loading it overwrites the robot's servo calibration and peripheral settings.
    </p>
    <div>
      <button class="btn btn-primary" onclick={() => load(config)}>Load</button>
    </div>
  {:else if step.kind === 'done'}
    <div class="alert alert-success alert-soft" role="alert">{step.message}</div>
  {:else if step.kind === 'failed'}
    <div class="alert alert-error alert-soft" role="alert">{step.message}</div>
  {/if}
</Section>
