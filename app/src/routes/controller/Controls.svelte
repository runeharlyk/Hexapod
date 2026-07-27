<script lang="ts">
  import nipplejs from 'nipplejs'
  import { onDestroy, onMount } from 'svelte'
  import { capitalize } from '$lib/utilities'
  import { input, outControllerData, mode, gait } from '$lib/stores'
  import type { ControllerInput, vector } from '$lib/types/models'
  import { VerticalSlider } from '$lib/components/input'
  import { MotionModes } from '$lib/motion'
  import { GaitType } from '$lib/gait'
  import { notifications } from '$lib/components/toasts/notifications'
  import { gamepadAxes, gamepadButtons, hasGamepad } from '$lib/stores/gamepad'
  import { requestGait, requestMode } from '$lib/control'

  type SliderKey = 'height' | 'feetDistance' | 'speed' | 's1'

  let left: nipplejs.JoystickManager
  let right: nipplejs.JoystickManager

  const syncCommand = (i: ControllerInput) => {
    outControllerData.set([
      i.left.x,
      i.left.y,
      i.right.x,
      i.right.y,
      i.height,
      i.speed,
      i.s1,
      i.feetDistance
    ])
  }

  $effect(() => {
    if ($hasGamepad) notifications.success('🎮 Gamepad connected', 3000)
  })

  $effect(() => {
    if (!$hasGamepad) return
    input.update(i => {
      i.left = { x: $gamepadAxes[0] ?? 0, y: $gamepadAxes[1] ?? 0 }
      i.right = { x: $gamepadAxes[2] ?? 0, y: $gamepadAxes[3] ?? 0 }
      syncCommand(i)
      return i
    })
  })

  $effect(() => {
    if ($gamepadButtons.length === 0) return
    if ($gamepadButtons[0].pressed) requestMode(MotionModes.DEACTIVATED)
    else if ($gamepadButtons[1].pressed) requestMode(MotionModes.IDLE)
    else if ($gamepadButtons[2].pressed) requestMode(MotionModes.STAND)
    else if ($gamepadButtons[3].pressed) requestMode(MotionModes.WALK)
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

    left.on('move', (_, d) => handleJoyMove('left', d.vector))
    left.on('end', () => handleJoyMove('left', { x: 0, y: 0 }))
    right.on('move', (_, d) => handleJoyMove('right', d.vector))
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

  const handleKey = (event: KeyboardEvent) => {
    if (event.repeat || event.ctrlKey || event.altKey || event.metaKey) return
    if (isTypingTarget(event.target)) return
    const down = event.type === 'keydown'
    input.update(i => {
      if (event.key === 'w') i.left.y = down ? 1 : 0
      if (event.key === 'a') i.left.x = down ? 1 : 0
      if (event.key === 's') i.left.y = down ? -1 : 0
      if (event.key === 'd') i.left.x = down ? -1 : 0
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

  const changeGait = (gaitValue: GaitType) => requestGait(gaitValue)
</script>

<div class="absolute top-0 left-0 h-screen w-screen">
  <!-- pb keeps the sticks clear of the control cluster on short screens; lg has room to spare -->
  <div class="absolute top-0 left-0 flex h-full w-full pb-40 lg:pb-0">
    <div
      id="left"
      class="flex w-40 shrink-0 items-center justify-center lg:w-60 lg:justify-end"
    ></div>
    <div class="flex-1"></div>
    <div
      id="right"
      class="flex w-40 shrink-0 items-center justify-center lg:w-60 lg:justify-start"
    ></div>
  </div>

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

  <div class="absolute bottom-0 left-0 z-10 flex max-w-full items-end overflow-x-auto">
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

    <div
      class="bg-base-300/70 flex h-min shrink-0 flex-wrap items-end gap-4 rounded-tr-xl p-3 pl-0"
    >
      <div class="join">
        {#each Object.values(MotionModes) as modeValue}
          <button
            class="btn join-item btn-sm lg:btn-md"
            class:btn-primary={$mode === modeValue && modeValue !== MotionModes.DEACTIVATED}
            class:btn-error={$mode === modeValue && modeValue === MotionModes.DEACTIVATED}
            onclick={() => requestMode(modeValue)}
          >
            {capitalize(modeValue)}
          </button>
        {/each}
        {#if $mode === MotionModes.WALK}
          <select
            class="select select-primary select-sm lg:select-md w-auto"
            aria-label="Gait"
            value={$gait}
            onchange={e => changeGait((e.target as HTMLSelectElement).value as GaitType)}
          >
            {#each Object.values(GaitType) as gaitValue}
              <option value={gaitValue}>
                {capitalize(gaitValue.toString())}
              </option>
            {/each}
          </select>
        {/if}
      </div>

      {#if $mode === MotionModes.WALK}
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
    </div>
  </div>
</div>

<svelte:window onkeyup={handleKey} onkeydown={handleKey} />
