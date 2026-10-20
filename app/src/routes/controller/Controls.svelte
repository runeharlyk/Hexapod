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
  import { VerticalSlider } from '$lib/components/input'
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

  // Each mode shows only the controls it reads, beside the mode buttons (lib/mode-controls.ts).
  const controls = $derived(controlsFor($drivenMode))
</script>

<div class="absolute top-0 left-0 h-screen w-screen">
  <!-- pb keeps the sticks clear of the control cluster on short screens; lg has room to spare -->
  <div class="absolute top-0 left-0 flex h-full w-full pb-40 lg:pb-0">
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

  <div class="absolute bottom-0 left-0 z-10 flex max-w-full items-end overflow-x-auto">
    {#if controls.height}
      <div class="bg-base-300/70 flex shrink-0 flex-col items-center gap-1 p-3 pb-2">
        <VerticalSlider
          id="height"
          min={-1}
          step={0.01}
          max={1}
          value={$input.height}
          oninput={(e: Event) => handleRange(e, 'height')}
        />
        <span class="font-mono text-xs tabular-nums">{$input.height.toFixed(2)}</span>
        <label for="height" class="text-xs">Ht</label>
      </div>
    {/if}

    {#if controls.feetDistance}
      <div class="bg-base-300/70 flex shrink-0 flex-col items-center gap-1 rounded-tr-xl p-3 pb-2">
        <VerticalSlider
          id="feetDistance"
          min={-1}
          step={0.01}
          max={1}
          value={$input.feetDistance}
          oninput={(e: Event) => handleRange(e, 'feetDistance')}
        />
        <span class="font-mono text-xs tabular-nums">{$input.feetDistance.toFixed(2)}</span>
        <label for="feetDistance" class="text-xs">Dist</label>
      </div>
    {/if}

    <div
      class="bg-base-300/70 flex h-min shrink-0 flex-wrap items-end gap-4 rounded-tr-xl p-3 pl-0"
    >
      <div class="join">
        {#each Object.values(MotionModes) as modeValue}
          <button
            class="btn join-item btn-sm lg:btn-md"
            class:btn-primary={$drivenMode === modeValue && modeValue !== MotionModes.DEACTIVATED}
            class:btn-error={$drivenMode === modeValue && modeValue === MotionModes.DEACTIVATED}
            onclick={() => driveMode(modeValue)}
          >
            {capitalize(modeValue)}
          </button>
        {/each}
        {#if controls.gait}
          <select
            class="select select-primary select-sm lg:select-md w-auto"
            aria-label="Gait"
            value={$drivenGait}
            onchange={e => driveGait((e.target as HTMLSelectElement).value as GaitType)}
          >
            {#each Object.values(GaitType) as gaitValue}
              <option value={gaitValue}>
                {capitalize(gaitValue.toString())}
              </option>
            {/each}
          </select>
        {/if}
      </div>

      {#if $drivenMode === null}
        <span class="text-warning max-w-sm text-xs" role="status">{UNKNOWN_MODE_HINT}</span>
      {/if}

      {#if controls.walk}
        <div class="flex gap-4">
          <div>
            <div class="flex items-baseline justify-between gap-2">
              <label for="s1" class="text-xs">Step height</label>
              <span class="font-mono text-xs tabular-nums">{$input.s1.toFixed(2)}</span>
            </div>
            <input
              id="s1"
              type="range"
              name="s1"
              min="-1"
              step="0.01"
              max="1"
              value={$input.s1}
              oninput={e => handleRange(e, 's1')}
              class="range range-sm range-primary"
            />
          </div>
          <div>
            <div class="flex items-baseline justify-between gap-2">
              <label for="stride" class="text-xs">Stride</label>
              <span class="font-mono text-xs tabular-nums">{$input.stride.toFixed(2)}</span>
            </div>
            <input
              id="stride"
              type="range"
              name="stride"
              min="-1"
              step="0.01"
              max="1"
              value={$input.stride}
              disabled={!strideApplies($drivenGait)}
              oninput={e => handleRange(e, 'stride')}
              class="range range-sm range-primary"
            />
          </div>
          <div>
            <div class="flex items-baseline justify-between gap-2">
              <label for="speed" class="text-xs">Speed</label>
              <span class="font-mono text-xs tabular-nums">{$input.speed.toFixed(2)}</span>
            </div>
            <input
              id="speed"
              type="range"
              name="speed"
              min="-1"
              step="0.01"
              max="1"
              value={$input.speed}
              oninput={e => handleRange(e, 'speed')}
              class="range range-sm range-primary"
            />
          </div>
        </div>
      {/if}

      {#if controls.joints}
        <div class="flex flex-wrap items-center gap-2">
          {#if $drivingJoints}
            <span class="badge badge-warning badge-sm shrink-0" aria-live="polite"
              >Driving the robot's joints</span
            >
            {#if $limpServos}
              <span class="badge badge-ghost badge-sm shrink-0" aria-live="polite"
                >{servoCount($limpServos)} of 18 servos limp</span
              >
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
    </div>

    {#if $drivesSimulation}
      <div class="bg-base-300/70 flex shrink-0 items-center gap-2 rounded-tr-xl p-3">
        <span class="badge badge-info badge-sm shrink-0">Simulation only</span>
      </div>
    {:else if !$isLinked}
      <div class="bg-base-300/70 flex shrink-0 items-center gap-2 rounded-tr-xl p-3">
        <span class="badge badge-warning badge-sm shrink-0">Preview</span>
      </div>
    {/if}

    {#if goalText}
      <div class="bg-base-300/70 flex shrink-0 items-center gap-2 rounded-tr-xl p-3">
        <span class="badge badge-info badge-sm shrink-0">Goal</span>
        <span class="font-mono text-xs tabular-nums" aria-live="polite">{goalText}</span>
      </div>
    {/if}
  </div>
</div>

<svelte:window onkeyup={handleKeyUp} onkeydown={handleKeyDown} onblur={releaseAllKeys} />
<svelte:document onvisibilitychange={handleVisibilityChange} />
