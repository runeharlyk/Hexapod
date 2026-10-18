<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { page } from '$app/state'
  import { Modals, modals } from 'svelte-modals'
  import Toast from '$lib/components/toasts/Toast.svelte'
  import { fade } from 'svelte/transition'
  import '../app.css'
  import Menu from '../lib/components/menu/Menu.svelte'
  import Statusbar from '../lib/components/statusbar/statusbar.svelte'
  import { canReachRobot, gait, location, mode, outControllerData } from '$lib/stores'
  import { MotionModes } from '$lib/motion'
  import { dataBroker } from '$lib/transport/databroker'
  import { serial } from '$lib/transport/serial-adapter'
  import { ble } from '$lib/transport/ble-adapter'
  import { websocket } from '$lib/transport/websocket-adapter'
  import { AnimationStatus, ControllerInputData, GaitData } from '$lib/platform_shared/message'
  import { GaitType } from '$lib/gait'
  import { throttler } from '$lib/utilities'
  import { setAnimationStatus } from '$lib/stores/animation'
  import { followGoalStatus } from '$lib/stores/goal'
  import { followMode } from '$lib/stores/robot-mode'
  import { connectWebsocket, isLinked } from '$lib/stores/link'
  import { COMMAND_HEARTBEAT_MS, needsHeartbeat } from '$lib/control-link'
  import { toControllerInput } from '$lib/controller-input'
  import type { Command } from '$lib/simulation/virtual-robot'
  import { watchUpdateOutcome } from '$lib/firmware/watch-update'

  interface Props {
    children?: import('svelte').Snippet
  }

  dataBroker.addTransport(serial)
  dataBroker.addTransport(websocket)
  dataBroker.addTransport(ble)

  const throttle = new throttler()
  let lastCommand: Command = [0, 0, 0, 0, 0, 0, 0, 0, 0]
  let currentMode = MotionModes.DEACTIVATED
  let linked = false
  let commandHeartbeatId: ReturnType<typeof setInterval> | undefined
  let stopWatchingUpdate: (() => void) | undefined
  let stopFollowingGoal: (() => void) | undefined
  let stopFollowingMode: (() => void) | undefined

  let { children }: Props = $props()

  onMount(() => {
    stopWatchingUpdate = watchUpdateOutcome()
    if (canReachRobot(page.url, $location)) connectWebsocket()

    let wasStop = true
    outControllerData.subscribe(data => {
      lastCommand = [...data]
      const isStop = data[0] === 0 && data[1] === 0 && data[2] === 0 && data[3] === 0

      if (isStop) {
        throttle.cancel()
        if (!wasStop) {
          dataBroker.emit(ControllerInputData, toControllerInput(data))
          wasStop = true
        }
      } else {
        wasStop = false
        throttle.throttle(() => dataBroker.emit(ControllerInputData, toControllerInput(data)), 40)
      }
    })

    stopFollowingMode = followMode(dataBroker, isLinked)

    dataBroker.on(AnimationStatus, setAnimationStatus)
    stopFollowingGoal = followGoalStatus(dataBroker, isLinked)

    dataBroker.on(GaitData, data => {
      const nextGait = Object.values(GaitType)[data.gait]
      if (nextGait !== undefined) gait.set(nextGait)
    })

    mode.subscribe(value => {
      currentMode = value
    })

    isLinked.subscribe(value => {
      linked = value
      if (!value) setAnimationStatus(null)
    })

    commandHeartbeatId = setInterval(() => {
      if (!linked) return
      if (!needsHeartbeat(currentMode)) return
      dataBroker.emit(ControllerInputData, toControllerInput(lastCommand))
    }, COMMAND_HEARTBEAT_MS)
  })

  onDestroy(() => {
    if (commandHeartbeatId) clearInterval(commandHeartbeatId)
    stopWatchingUpdate?.()
    stopFollowingGoal?.()
    stopFollowingMode?.()
  })

  let menuOpen = $state(false)
</script>

<svelte:head>
  <title>{page.data.title}</title>
</svelte:head>

<div class="drawer">
  <input id="main-menu" type="checkbox" class="drawer-toggle" bind:checked={menuOpen} />
  <div class="drawer-content flex flex-col">
    <Statusbar />

    {@render children?.()}
  </div>
  <div class="drawer-side z-30 shadow-lg">
    <label for="main-menu" class="drawer-overlay"></label>
    <Menu menuClicked={() => (menuOpen = false)} />
  </div>
</div>

<Modals>
  {#snippet backdrop()}
    <button
      type="button"
      aria-label="Close modal"
      class="fixed inset-0 z-40 max-h-full max-w-full bg-black/20 backdrop-blur-sm"
      transition:fade
      onclick={modals.closeAll}
    ></button>
  {/snippet}
</Modals>

<Toast />
