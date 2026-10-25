<script lang="ts">
  import nipplejs from 'nipplejs'
  import { onDestroy, onMount } from 'svelte'
  import { capitalize } from '$lib/utilities'
  import { input } from '$lib/stores'
  import {
    drive,
    driveGait,
    driveMode,
    drivenGait,
    drivenGoal,
    drivenMode,
    drivesSimulation
  } from '$lib/stores/drive'
  import { isLinked } from '$lib/stores/link'
  import { goalLine } from '$lib/stores/goal'
  import type { ControllerInput, vector } from '$lib/types/models'
  import { MotionModes, UNKNOWN_MODE_HINT, servoCount } from '$lib/motion'
  import { GaitType, strideApplies } from '$lib/gait'
  import { notifications } from '$lib/components/toasts/notifications'
  import { gamepadAxes, gamepadButtons, hasGamepad } from '$lib/stores/gamepad'
  import { gamepadSticks, shapeStick } from '$lib/utilities/stick'
  import { commandOf } from '$lib/controller-input'
  import { controlsFor } from '$lib/mode-controls'
  import { drivingJoints, limpServos } from '$lib/joint-drive'
  import { resolve } from '$app/paths'
  import AnimationControls from './AnimationControls.svelte'

  type SliderKey = 'height' | 'feetDistance' | 'speed' | 's1' | 'stride'

  let left: nipplejs.JoystickManager
  let right: nipplejs.JoystickManager
  let adjustmentsOpen = $state(false)

  // While the simulation view is shown, the controls drive its robot and never the real one (stores/drive.ts).
  const syncCommand = (i: ControllerInput) => drive(commandOf(i))

  $effect(() => {
    if ($hasGamepad) notifications.success('🎮 Gamepad connected', 3000)
  })

  let gamepadWasConnected = false
  $effect(() => {
    if (!$hasGamepad) {
      if (!gamepadWasConnected) return
      gamepadWasConnected = false
      input.update(i => {
        i.left = { x: 0, y: 0 }
        i.right = { x: 0, y: 0 }
        syncCommand(i)
        return i
      })
      return
    }
    gamepadWasConnected = true
    const sticks = gamepadSticks($gamepadAxes, controls.sticks)
    if (!sticks) return
    input.update(i => {
      i.left = sticks.left
      i.right = sticks.right
      syncCommand(i)
      return i
    })
  })

  $effect(() => {
    if ($gamepadButtons.length === 0) return
    if ($gamepadButtons[0].pressed) driveMode(MotionModes.DEACTIVATED)
    else if ($gamepadButtons[1].pressed) driveMode(MotionModes.IDLE)
    else if ($gamepadButtons[2].pressed) driveMode(MotionModes.STAND)
    else if ($gamepadButtons[3].pressed) driveMode(MotionModes.WALK)
  })

  onMount(() => {
    const options = {
      color: '#15191e80',
      dynamicPage: true,
      mode: 'static' as const,
      restOpacity: 1
    }

    left = nipplejs.create({ zone: document.getElementById('left') as HTMLElement, ...options })
    right = nipplejs.create({ zone: document.getElementById('right') as HTMLElement, ...options })

    left.on('move', (_, d) => handleJoyMove('left', shapeStick(d.vector)))
    left.on('end', () => handleJoyMove('left', { x: 0, y: 0 }))
    right.on('move', (_, d) => handleJoyMove('right', shapeStick(d.vector)))
    right.on('end', () => handleJoyMove('right', { x: 0, y: 0 }))
  })

  onDestroy(() => {
    left?.destroy()
    right?.destroy()
  })

  const handleJoyMove = (key: 'left' | 'right', v: vector) => {
    input.update(i => {
      i[key] = v
      syncCommand(i)
      return i
    })
  }

  const isTypingTarget = (target: EventTarget | null) =>
    target instanceof HTMLElement &&
    (target.isContentEditable || ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName))

  // nipplejs reports +x to the right and +y up; the keys mirror the left stick.
  const DRIVE_KEYS: Record<string, { axis: 'x' | 'y'; sign: 1 | -1 }> = {
    KeyW: { axis: 'y', sign: 1 },
    KeyS: { axis: 'y', sign: -1 },
    KeyD: { axis: 'x', sign: 1 },
    KeyA: { axis: 'x', sign: -1 }
  }
  const heldKeys = new Set<string>()

  const syncHeldKeys = () => {
    input.update(i => {
      const stick = { x: 0, y: 0 }
      for (const code of heldKeys) stick[DRIVE_KEYS[code].axis] += DRIVE_KEYS[code].sign
      i.left = stick
      syncCommand(i)
      return i
    })
  }

  const handleKeyDown = (event: KeyboardEvent) => {
    if (!controls.sticks || !(event.code in DRIVE_KEYS) || event.repeat) return
    if (event.ctrlKey || event.altKey || event.metaKey || isTypingTarget(event.target)) return
    heldKeys.add(event.code)
    syncHeldKeys()
  }

  const handleKeyUp = (event: KeyboardEvent) => {
    if (!heldKeys.delete(event.code)) return
    syncHeldKeys()
  }

  const releaseAllKeys = () => {
    heldKeys.clear()
    handleJoyMove('left', { x: 0, y: 0 })
  }

  // A backgrounded tab cannot see the sticks it holds, and its timers are throttled, so it stops the
  // robot outright rather than leave a deflected stick re-sent by the heartbeat.
  const handleVisibilityChange = () => {
    if (document.visibilityState !== 'hidden') return
    heldKeys.clear()
    input.update(i => {
      i.left = { x: 0, y: 0 }
      i.right = { x: 0, y: 0 }
      syncCommand(i)
      return i
    })
  }

  const handleRange = (event: Event, key: SliderKey) => {
    const value = Number((event.target as HTMLInputElement).value)
    input.update(i => {
      i[key] = value
      syncCommand(i)
      return i
    })
  }

  const goalText = $derived($drivenGoal ? goalLine($drivenGoal) : null)

  // Each mode shows only the controls it reads, beside the mode selector (lib/mode-controls.ts).
  const controls = $derived(controlsFor($drivenMode))
</script>

<div class="absolute top-0 left-0 h-full w-full">
  <div class="absolute top-0 left-0 flex h-full w-full">
    <!-- The sticks are created once and hidden, not removed, in the modes that read none. -->
    <div
      id="left"
      class="flex w-40 shrink-0 items-center justify-center lg:w-60 lg:justify-end"
      class:invisible={!controls.sticks}
    ></div>
    <div class="flex-1"></div>
    <div
      id="right"
      class="flex w-40 shrink-0 items-center justify-center lg:w-60 lg:justify-start"
      class:invisible={!controls.sticks}
    ></div>
  </div>

  {#if controls.sticks}
    <div class="absolute right-0 bottom-0 z-10 hidden flex-col gap-2 p-4 lg:flex">
      <div class="flex w-full justify-center">
        <kbd class="kbd">W</kbd>
      </div>
      <div class="flex w-full justify-center gap-2">
        <kbd class="kbd">A</kbd>
        <kbd class="kbd">S</kbd>
        <kbd class="kbd">D</kbd>
      </div>
    </div>
  {/if}

  <div
    class="bg-base-300/50 border-base-content/10 absolute bottom-0 left-0 z-10 flex max-h-[calc(100%-4rem)] w-full flex-col gap-2 overflow-y-auto border-t p-3 backdrop-blur-md lg:w-auto lg:max-w-3xl lg:rounded-tr-xl"
    style="padding-bottom: max(0.75rem, env(safe-area-inset-bottom))"
  >
    <div class="flex items-center gap-2">
      <select
        class="select select-primary select-sm lg:select-md w-auto min-w-0"
        aria-label="Mode"
        value={$drivenMode ?? ''}
        onchange={e => {
          const select = e.target as HTMLSelectElement
          driveMode(select.value as MotionModes)
          select.value = $drivenMode ?? ''
        }}
      >
        {#if $drivenMode === null}
          <option value="" disabled>Unknown mode</option>
        {/if}
        {#each Object.values(MotionModes) as modeValue}
          <option value={modeValue}>{capitalize(modeValue)}</option>
        {/each}
      </select>
      {#if $isLinked && !$drivesSimulation}
        <button
          class="btn btn-sm lg:btn-md shrink-0"
          class:btn-error={$drivenMode === MotionModes.DEACTIVATED}
          onclick={() => driveMode(MotionModes.DEACTIVATED)}
        >
          Deactivate
        </button>
      {/if}
      {#if controls.height || controls.feetDistance || controls.walk}
        <button
          class="btn btn-sm lg:btn-md shrink-0"
          aria-expanded={adjustmentsOpen}
          aria-controls="controller-adjustments"
          onclick={() => (adjustmentsOpen = !adjustmentsOpen)}
        >
          Adjustments
        </button>
      {/if}
      {#if $drivesSimulation}
        <span class="badge badge-info badge-sm shrink-0">Simulation only</span>
      {:else if !$isLinked}
        <span class="badge badge-warning badge-sm shrink-0">Preview</span>
      {/if}
    </div>

    {#if $drivenMode === null}
      <span class="text-warning max-w-sm text-xs" role="status">{UNKNOWN_MODE_HINT}</span>
    {/if}

    {#if controls.gait || controls.walk}
      <div class="flex items-end gap-3">
        {#if controls.gait}
          <select
            class="select select-primary select-sm lg:select-md w-auto max-w-1/2"
            aria-label="Gait"
            value={$drivenGait}
            onchange={e => driveGait((e.target as HTMLSelectElement).value as GaitType)}
          >
            {#each Object.values(GaitType) as gaitValue}
              <option value={gaitValue}>{capitalize(gaitValue.toString())}</option>
            {/each}
          </select>
        {/if}
        {#if controls.walk}
          <div class="min-w-0 flex-1">
            {@render slider('speed', 'Speed')}
          </div>
        {/if}
      </div>
    {/if}

    {#if controls.height || controls.feetDistance || controls.walk}
      <div id="controller-adjustments" hidden={!adjustmentsOpen}>
        <div class="grid grid-cols-2 gap-3">
          {#if controls.height}
            <div class="min-w-0">{@render slider('height', 'Body height')}</div>
          {/if}
          {#if controls.feetDistance}
            <div class="min-w-0">{@render slider('feetDistance', 'Feet distance')}</div>
          {/if}
          {#if controls.walk}
            <div class="min-w-0">{@render slider('s1', 'Step height')}</div>
            <div class="min-w-0">
              {@render slider('stride', 'Stride', !strideApplies($drivenGait))}
            </div>
          {/if}
        </div>
      </div>
    {/if}

    {#if controls.joints}
      <div class="flex flex-wrap items-center gap-2">
        {#if $drivingJoints}
          <span class="badge badge-warning badge-sm" aria-live="polite"
            >Driving the robot's joints</span
          >
          {#if $limpServos}
            <span class="badge badge-ghost badge-sm" aria-live="polite">
              {servoCount($limpServos)} of 18 servos limp
            </span>
          {/if}
          <span class="text-xs">Drag a leg of the 3D model to move its servo</span>
        {:else if !$drivesSimulation}
          <span class="text-xs">Connect to the robot to drive its joints from the 3D model</span>
        {/if}
        <a class="link text-xs" href={resolve('/peripherals/servo')}>Servos</a>
      </div>
    {/if}

    {#if controls.animation}
      <AnimationControls />
    {/if}

    {#if goalText}
      <div class="flex flex-wrap items-center gap-2">
        <span class="badge badge-info badge-sm">Goal</span>
        <span class="font-mono text-xs tabular-nums" aria-live="polite">{goalText}</span>
      </div>
    {/if}
  </div>
</div>

{#snippet slider(key: SliderKey, label: string, disabled = false)}
  <div class="flex items-baseline justify-between gap-2">
    <label for={key} class="text-xs">{label}</label>
    <span class="font-mono text-xs tabular-nums">{$input[key].toFixed(2)}</span>
  </div>
  <input
    id={key}
    type="range"
    name={key}
    min="-1"
    step="0.01"
    max="1"
    value={$input[key]}
    {disabled}
    oninput={e => handleRange(e, key)}
    class="range range-sm range-primary"
  />
{/snippet}

<svelte:window onkeyup={handleKeyUp} onkeydown={handleKeyDown} onblur={releaseAllKeys} />
<svelte:document onvisibilitychange={handleVisibilityChange} />
