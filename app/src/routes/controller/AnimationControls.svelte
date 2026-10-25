<script lang="ts">
  import { resolve } from '$app/paths'
  import { builtIn } from '$lib/animation/library'
  import { AnimationState } from '$lib/platform_shared/message'
  import type { ParamId } from '$lib/platform_shared/animation'
  import { animationStatus } from '$lib/robot/state'
  import {
    drivesSimulation,
    simulation,
    playDrivenAnimation,
    stopDrivenAnimation
  } from '$lib/stores/drive'
  import { isLinked } from '$lib/stores/link'

  const simulationPlayer = simulation.player
  const simulationStatus = simulation.animationStatus
  let name = $state(builtIn[0]?.name ?? '')

  const defaults = (chosen: string) =>
    new Map<ParamId, number>(
      builtIn.find(a => a.name === chosen)?.params.map(p => [p.id, p.defaultValue]) ?? []
    )

  const usable = $derived($drivesSimulation ? $simulationPlayer !== null : $isLinked)
  const status = $derived($drivesSimulation ? $simulationStatus : $animationStatus)
  const playing = $derived(status && status.state !== AnimationState.ANIM_IDLE ? status.name : null)
</script>

<div class="flex flex-wrap items-center gap-2">
  <select
    class="select select-primary select-sm lg:select-md w-auto"
    aria-label="Animation"
    bind:value={name}
  >
    {#each builtIn as animation (animation.name)}
      <option value={animation.name}>{animation.name}</option>
    {/each}
  </select>
  <button
    class="btn btn-primary btn-sm lg:btn-md"
    disabled={!usable}
    onclick={() => playDrivenAnimation(name, defaults(name))}
  >
    Play
  </button>
  <button class="btn btn-sm lg:btn-md" disabled={!usable} onclick={stopDrivenAnimation}>Stop</button
  >
  {#if $drivesSimulation && !usable}
    <span class="text-xs opacity-70">Waiting for the simulator</span>
  {/if}
  {#if playing}
    <span class="badge badge-info badge-sm" aria-live="polite">Playing {playing}</span>
  {/if}
  <a class="link text-xs" href={resolve('/animations')}>All animations</a>
</div>
