<script lang="ts">
  import nipplejs from 'nipplejs'
  import { onMount } from 'svelte'
  import { capitalize, throttler, toInt8 } from '$lib/utilities'
  import { input, outControllerData, mode, gait } from '$lib/stores'
  import type { vector } from '$lib/types/models'
  import { VerticalSlider } from '$lib/components/input'
  import { MotionModes } from '$lib/motion'
  import { GaitType } from '$lib/gait'
  import { notifications } from '$lib/components/toasts/notifications'
  import { gamepadAxes, gamepadButtons, hasGamepad } from '$lib/stores/gamepad'

  let throttle = new throttler()
  let left: nipplejs.JoystickManager
  let right: nipplejs.JoystickManager

  let throttle_timing = 40
  let data = new Array(7)

  $effect(() => {
    if ($hasGamepad) {
      notifications.success('🎮 Gamepad connected', 3000)
    }
  })

  $effect(() => {
    handleJoyMove('left', { x: $gamepadAxes[0], y: $gamepadAxes[1] })
    handleJoyMove('right', { x: $gamepadAxes[2], y: $gamepadAxes[3] })
  })

  $effect(() => {
    if ($gamepadButtons.length === 0) return

    if ($gamepadButtons[0].pressed) {
      mode.set(MotionModes.DEACTIVATED)
    } else if ($gamepadButtons[1].pressed) {
      mode.set(MotionModes.IDLE)
    } else if ($gamepadButtons[2].pressed) {
      mode.set(MotionModes.STAND)
    } else if ($gamepadButtons[3].pressed) {
      mode.set(MotionModes.WALK)
    }
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

    left.on('move', (_, data) => handleJoyMove('left', data.vector))
    left.on('end', (_, __) => handleJoyMove('left', { x: 0, y: 0 }))
    right.on('move', (_, data) => handleJoyMove('right', data.vector))
    right.on('end', (_, __) => handleJoyMove('right', { x: 0, y: 0 }))
  })

  const handleJoyMove = (key: 'left' | 'right', data: vector) => {
    input.update(inputData => {
      inputData[key] = data
      return inputData
    })
    throttle.throttle(updateData, throttle_timing)
  }

  const updateData = () => {
    data[0] = toInt8($input.left.x, -1, 1)
    data[1] = toInt8($input.left.y, -1, 1)
    data[2] = toInt8($input.right.x, -1, 1)
    data[3] = toInt8($input.right.y, -1, 1)
    data[4] = toInt8($input.height, 0, 100)
    data[5] = toInt8($input.speed, 0, 25)
    data[6] = toInt8($input.s1, 0, 25)

    outControllerData.set(data)
  }

  const handleKeyup = (event: KeyboardEvent) => {
    const down = event.type === 'keydown'
    input.update(data => {
      if (event.key === 'w') data.left.y = down ? 1 : 0
      if (event.key === 'a') data.left.x = down ? 1 : 0
      if (event.key === 's') data.left.y = down ? -1 : 0
      if (event.key === 'd') data.left.x = down ? -1 : 0
      return data
    })
    throttle.throttle(updateData, throttle_timing)
  }

  const handleRange = (event: Event, key: 'speed' | 'height' | 's1') => {
    const value: number = Number((event.target as HTMLInputElement).value)

    input.update(inputData => {
      inputData[key] = value
      return inputData
    })
    throttle.throttle(updateData, throttle_timing)
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
      <VerticalSlider min={0} max={100} oninput={(e: Event) => handleRange(e, 'height')} />
      <label for="height">Ht</label>
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
            oninput={e => changeGait(e.target?.value as GaitType)}>
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
              min="0"
              max="25"
              oninput={e => handleRange(e, 's1')}
              class="range range-sm range-primary" />
          </div>
          <div>
            <label for="speed">Speed</label>
            <input
              type="range"
              name="speed"
              min="0"
              max="25"
              oninput={e => handleRange(e, 'speed')}
              class="range range-sm range-primary" />
          </div>
        </div>
      {/if}
    </div>
  </div>
</div>

<svelte:window onkeyup={handleKeyup} onkeydown={handleKeyup} />
