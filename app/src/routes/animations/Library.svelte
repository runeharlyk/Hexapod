<script lang="ts">
  import { onMount } from 'svelte'
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import { Cancel, Delete } from '$lib/components/icons'
  import { notifications } from '$lib/components/toasts/notifications'
  import { ParamId, type Animation } from '$lib/platform_shared/animation'
  import { AnimationState } from '$lib/platform_shared/message'
  import { isLinked } from '$lib/stores/link'
  import { animationStatus } from '$lib/stores/animation'
  import { clampedJoints, shown } from '$lib/stores/animation-editor'
  import { playAnimation, stopAnimation } from '$lib/control'
  import { deleteAnimation, downloadAnimation, uploadAnimation } from '$lib/animation/transfer'
  import {
    builtIn,
    deleteDraft,
    draftAnimations,
    refreshRobotList,
    robotAnimations
  } from '$lib/animation/library'

  const JOINT_INDICES = Array.from({ length: 18 }, (_, j) => j)

  interface Props {
    onEdit: (a: Animation) => void
  }

  let { onEdit }: Props = $props()

  // Slider values per row, keyed by section and name so a draft and a built-in do not share them.
  let values = $state<Record<string, Record<number, number>>>({})

  const valuesOf = (key: string) =>
    new Map(Object.entries(values[key] ?? {}).map(([id, v]) => [Number(id) as ParamId, v]))

  const setValue = (key: string, id: ParamId, v: number) => {
    values[key] = { ...values[key], [id]: v }
  }

  onMount(() =>
    isLinked.subscribe(linked => {
      if (linked) refreshRobotList()
      else robotAnimations.set(null)
    })
  )

  const playDraft = async (a: Animation, key: string) => {
    try {
      const result = await uploadAnimation(a)
      if (!result.ok) return notifications.error(`${a.name} was refused: ${result.error}`, 5000)
      refreshRobotList()
      playAnimation(a.name, valuesOf(key))
    } catch (e) {
      notifications.error(`Uploading ${a.name} failed: ${e}`, 5000)
    }
  }

  const openFromRobot = async (name: string) => {
    try {
      onEdit(await downloadAnimation(name))
    } catch (e) {
      notifications.error(`Downloading ${name} failed: ${e}`, 5000)
    }
  }

  const confirmDelete = (name: string) =>
    modals.open(ConfirmDialog, {
      title: 'Delete animation',
      message: `Delete ${name} from the robot?`,
      labels: {
        cancel: { label: 'Keep', icon: Cancel },
        confirm: { label: 'Delete', icon: Delete }
      },
      onConfirm: async () => {
        modals.close()
        try {
          await deleteAnimation(name)
        } catch (e) {
          notifications.error(`Deleting ${name} failed: ${e}`, 5000)
        }
        refreshRobotList()
      }
    })

  const stateLabel = (s: AnimationState) => AnimationState[s]?.replace('ANIM_', '') ?? 'UNKNOWN'
</script>

{#snippet params(a: Animation, key: string)}
  {#each a.params as spec (spec.id)}
    {@const value = values[key]?.[spec.id] ?? spec.defaultValue}
    <label class="flex flex-col text-xs">
      <span class="flex justify-between">
        <span>{ParamId[spec.id]} {value.toFixed(2)}</span>
        <span class="opacity-60"
          >{shown(spec.min)} / {shown(spec.defaultValue)} / {shown(spec.max)}</span
        >
      </span>
      <input
        type="range"
        class="range range-xs"
        min={spec.min}
        max={spec.max}
        step={(spec.max - spec.min) / 100 || 0.01}
        {value}
        oninput={e => setValue(key, spec.id, e.currentTarget.valueAsNumber)}
      />
    </label>
  {/each}
{/snippet}

{#snippet playStop(play: () => void)}
  <button class="btn btn-xs btn-primary" disabled={!$isLinked} onclick={play}>Play</button>
  <button class="btn btn-xs" disabled={!$isLinked} onclick={stopAnimation}>Stop</button>
{/snippet}

<div class="bg-base-200 rounded-box flex flex-wrap items-center gap-3 p-3 text-sm">
  {#if $animationStatus}
    <span class="font-medium">{$animationStatus.name || 'idle'}</span>
    <span class="badge badge-sm">{stateLabel($animationStatus.state)}</span>
    <span class="font-mono">t {$animationStatus.t.toFixed(2)} s</span>
    <span
      class="flex gap-0.5"
      title={clampedJoints($animationStatus.clampedMask).join(', ') || 'no joint clamped'}
    >
      {#each JOINT_INDICES as j (j)}
        <span
          class="size-2 rounded-full {($animationStatus.clampedMask >>> j) & 1 ?
            'bg-error'
          : 'bg-base-content/20'}"
        ></span>
      {/each}
    </span>
  {:else}
    <span class="opacity-60">No animation status: {$isLinked ? 'idle' : 'not connected'}</span>
  {/if}
</div>

<div class="grid grid-cols-1 gap-4 lg:grid-cols-3">
  <section class="card bg-base-200">
    <div class="card-body gap-3 p-4">
      <h2 class="card-title">Built-in</h2>
      {#each builtIn as a (a.name)}
        {@const key = 'builtin:' + a.name}
        <div class="bg-base-100 rounded-box flex flex-col gap-2 p-3">
          <div>
            <div class="font-medium">{a.name}</div>
            <div class="text-xs opacity-70">{a.description}</div>
          </div>
          {@render params(a, key)}
          <div class="flex flex-wrap gap-2">
            {@render playStop(() => playAnimation(a.name, valuesOf(key)))}
            <button class="btn btn-xs" onclick={() => onEdit(a)}>Edit</button>
          </div>
        </div>
      {/each}
    </div>
  </section>

  <section class="card bg-base-200">
    <div class="card-body gap-3 p-4">
      <h2 class="card-title">On robot</h2>
      {#if !$isLinked}
        <p class="text-sm opacity-60">not connected</p>
      {:else if $robotAnimations === null}
        <p class="text-sm opacity-60">Could not list the robot's animations.</p>
        <button class="btn btn-xs self-start" onclick={refreshRobotList}>Retry</button>
      {:else}
        {#each $robotAnimations as entry (entry.name)}
          <div class="bg-base-100 rounded-box flex flex-col gap-2 p-3">
            <div>
              <div class="font-medium">{entry.name}</div>
              <div class="text-xs opacity-70">{entry.size} bytes, played with its defaults</div>
            </div>
            <div class="flex flex-wrap gap-2">
              {@render playStop(() => playAnimation(entry.name, new Map()))}
              <button class="btn btn-xs" onclick={() => openFromRobot(entry.name)}
                >Download to editor</button
              >
              <button class="btn btn-xs btn-error" onclick={() => confirmDelete(entry.name)}
                >Delete</button
              >
            </div>
          </div>
        {:else}
          <p class="text-sm opacity-60">No animations on the robot.</p>
        {/each}
      {/if}
    </div>
  </section>

  <section class="card bg-base-200">
    <div class="card-body gap-3 p-4">
      <h2 class="card-title">Drafts</h2>
      {#each $draftAnimations as a (a.name)}
        {@const key = 'draft:' + a.name}
        <div class="bg-base-100 rounded-box flex flex-col gap-2 p-3">
          <div>
            <div class="font-medium">{a.name}</div>
            <div class="text-xs opacity-70">{a.description}</div>
          </div>
          {@render params(a, key)}
          <div class="flex flex-wrap gap-2">
            {@render playStop(() => playDraft(a, key))}
            <button class="btn btn-xs" onclick={() => onEdit(a)}>Edit</button>
            <button class="btn btn-xs btn-error" onclick={() => deleteDraft(a.name)}>Delete</button>
          </div>
        </div>
      {:else}
        <p class="text-sm opacity-60">No drafts in this browser.</p>
      {/each}
    </div>
  </section>
</div>
