<script lang="ts">
  import { resolve } from '$app/paths'
  import { builtIn } from '$lib/animation/library'
  import { playAnimation, stopAnimation } from '$lib/control'
  import { AnimationState } from '$lib/platform_shared/message'
  import type { ParamId } from '$lib/platform_shared/animation'
  import { animationStatus } from '$lib/robot/state'
  import { drivesSimulation } from '$lib/stores/drive'
  import { isLinked } from '$lib/stores/link'

  // ANIMATE's controls: a bundled animation played on the robot at its default parameters, the stop, what plays now,
  // and the animations page for the rest. The simulation view's robot plays none, so they drive the real robot only.
  let name = $state(builtIn[0]?.name ?? '')

  const defaults = (chosen: string) =>
    new Map<ParamId, number>(
      builtIn.find(a => a.name === chosen)?.params.map(p => [p.id, p.defaultValue]) ?? []
    )

  const usable = $derived($isLinked && !$drivesSimulation)
  const playing = $derived(
    $animationStatus && $animationStatus.state !== AnimationState.ANIM_IDLE ?
      $animationStatus.name
    : null
  )
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
    onclick={() => playAnimation(name, defaults(name))}
  >
    Play
  </button>
  <button class="btn btn-sm lg:btn-md" disabled={!usable} onclick={stopAnimation}>Stop</button>
  {#if playing}
    <span class="badge badge-info badge-sm" aria-live="polite">Playing {playing}</span>
  {/if}
  <a class="link text-xs" href={resolve('/animations')}>All animations</a>
</div>
