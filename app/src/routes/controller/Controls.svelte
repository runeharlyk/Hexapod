<script lang="ts">
  import nipplejs from 'nipplejs'
  import { onMount } from 'svelte'
  import { capitalize } from '$lib/utilities'
  import { input, outControllerData, mode, gait } from '$lib/stores'
  import type { vector } from '$lib/types/models'
  import { VerticalSlider } from '$lib/components/input'
  import { MotionModes } from '$lib/motion'
  import { GaitType } from '$lib/gait'
  import { notifications } from '$lib/components/toasts/notifications'
  import { gamepadAxes, gamepadButtons, hasGamepad } from '$lib/stores/gamepad'

  let left: nipplejs.JoystickManager
  let right: nipplejs.JoystickManager

  let data = new Array(8)

  $effect(() => {
    if ($hasGamepad) notifications.success('🎮 Gamepad connected', 3000)
  })

  $effect(() => {
    input.update(i => {
      i.left = { x: $gamepadAxes[0] ?? 0, y: $gamepadAxes[1] ?? 0 }
      i.right = { x: $gamepadAxes[2] ?? 0, y: $gamepadAxes[3] ?? 0 }
      data[0] = i.left.x
      data[1] = i.left.y
      data[2] = i.right.x
      data[3] = i.right.y
      data[4] = i.height
      data[5] = i.speed
      data[6] = i.s1
      data[7] = i.feetDistance
      outControllerData.set(data)
      return i
    })
  })

  $effect(() => {
    if ($gamepadButtons.length === 0) return
    if ($gamepadButtons[0].pressed) mode.set(MotionModes.DEACTIVATED)
    else if ($gamepadButtons[1].pressed) mode.set(MotionModes.IDLE)
    else if ($gamepadButtons[2].pressed) mode.set(MotionModes.STAND)
    else if ($gamepadButtons[3].pressed) mode.set(MotionModes.WALK)
    else if ($gamepadButtons[4].pressed) mode.set(MotionModes.CONSTRAINED_RANDOM)
    else if ($gamepadButtons[5].pressed) mode.set(MotionModes.LAYING_TRANSITION)
    else if ($gamepadButtons[6].pressed) mode.set(MotionModes.STANDING_UP)
  })

  onMount(() => {
    left = nipplejs.create({
      zone: document.getElementById('left') as HTMLElement,
      color: '#15191e80',
      dynamicPage: true,
      mode: 'static',
      restOpacity: 1
    })

    right = nipplejs.create({
      zone: document.getElementById('right') as HTMLElement,
      color: '#15191e80',
      dynamicPage: true,
      mode: 'static',
      restOpacity: 1
    })

    left.on('move', (_, d) => handleJoyMove('left', d.vector))
    left.on('end', () => handleJoyMove('left', { x: 0, y: 0 }))
    right.on('move', (_, d) => handleJoyMove('right', d.vector))
    right.on('end', () => handleJoyMove('right', { x: 0, y: 0 }))
  })

  const handleJoyMove = (key: 'left' | 'right', v: vector) => {
    input.update(i => {
      i[key] = v
      data[0] = i.left.x
      data[1] = i.left.y
      data[2] = i.right.x
      data[3] = i.right.y
      data[4] = i.height
      data[5] = i.speed
      data[6] = i.s1
      data[7] = i.feetDistance
      outControllerData.set(data)
      return i
    })
  }

  const handleKeyup = (event: KeyboardEvent) => {
    const down = event.type === 'keydown'
    input.update(i => {
      if (event.key === 'w') i.left.y = down ? 1 : 0
      if (event.key === 'a') i.left.x = down ? 1 : 0
      if (event.key === 's') i.left.y = down ? -1 : 0
      if (event.key === 'd') i.left.x = down ? -1 : 0
      data[0] = i.left.x
      data[1] = i.left.y
      data[2] = i.right.x
      data[3] = i.right.y
      data[4] = i.height
      data[5] = i.speed
      data[6] = i.s1
      data[7] = i.feetDistance
      outControllerData.set(data)
      return i
    })
  }

  const handleRange = (event: Event, key: 'speed' | 'height' | 's1' | 'feetDistance') => {
    const value: number = Number((event.target as HTMLInputElement).value)
    input.update(i => {
      i[key] = value
      data[0] = i.left.x
      data[1] = i.left.y
      data[2] = i.right.x
      data[3] = i.right.y
      data[4] = i.height
      data[5] = i.speed
      data[6] = i.s1
      data[7] = i.feetDistance
      outControllerData.set(data)
      return i
    })
  }

  const changeMode = (modeValue: MotionModes) => {
    mode.set(modeValue)
  }

  const changeGait = (gaitValue: GaitType) => {
    gait.set(gaitValue)
  }
</script>

<div class="absolute top-0 left-0 w-screen h-screen">
  <div class="absolute top-0 left-0 h-full w-full flex portrait:hidden">
    <div id="left" class="flex w-60 items-center justify-end"></div>
    <div class="flex-1"></div>
    <div id="right" class="flex w-60 items-center"></div>
  </div>
  <div class="absolute bottom-0 right-0 p-4 z-10 gap-2 flex-col hidden lg:flex">
    <div class="flex justify-center w-full">
      <kbd class="kbd">W</kbd>
    </div>
    <div class="flex justify-center gap-2 w-full">
      <kbd class="kbd">A</kbd>
      <kbd class="kbd">S</kbd>
      <kbd class="kbd">D</kbd>
    </div>
    <div class="flex justify-center w-full"></div>
  </div>
  <div class="absolute bottom-0 z-10 flex items-end">
    <div class="flex items-center flex-col bg-base-300 bg-opacity-50 p-3 pb-2 gap-2 rounded-tr-xl">
      <VerticalSlider
        min={-1}
        step={0.01}
        max={1}
        oninput={(e: Event) => handleRange(e, 'height')} />
      <label for="height">Ht</label>
    </div>
    <div class="flex items-center flex-col bg-base-300 bg-opacity-50 p-3 pb-2 gap-2">
      <VerticalSlider
        min={-1}
        step={0.01}
        max={1}
        oninput={(e: Event) => handleRange(e, 'feetDistance')} />
      <label for="feetDistance">Dist</label>
    </div>
    <div class="flex items-end gap-4 bg-base-300 bg-opacity-50 h-min rounded-tr-xl pl-0 p-3">
      <div class="join">
        {#each Object.values(MotionModes) as modeValue}
          <button
            class="btn join-item"
            class:btn-primary={$mode === modeValue}
            onclick={() => changeMode(modeValue)}>
            {capitalize(modeValue)}
          </button>
        {/each}
        {#if $mode === MotionModes.WALK}
          <select
            class="select select-primary"
            oninput={e => changeGait((e.target as HTMLSelectElement)?.value as GaitType)}>
            {#each Object.values(GaitType) as gaitValue}
              <option value={gaitValue} selected={$gait === gaitValue}>
                {capitalize(gaitValue.toString())}
              </option>
            {/each}
          </select>
        {/if}
      </div>

      {#if $mode === MotionModes.WALK}
        <div class="flex gap-4">
          <div>
            <label for="s1">S1</label>
            <input
              type="range"
              name="s1"
              min="-1"
              step="0.01"
              max="1"
              oninput={e => handleRange(e, 's1')}
              class="range range-sm range-primary" />
          </div>
          <div>
            <label for="speed">Speed</label>
            <input
              type="range"
              name="speed"
              min="-1"
              step="0.01"
              max="1"
              oninput={e => handleRange(e, 'speed')}
              class="range range-sm range-primary" />
          </div>
        </div>
      {/if}
    </div>
  </div>
</div>

<svelte:window onkeyup={handleKeyup} onkeydown={handleKeyup} />
