<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { derived } from 'svelte/store'
  import { page } from '$app/state'
  import { Modals, modals } from 'svelte-modals'
  import Toast from '$lib/components/toasts/Toast.svelte'
  import { fade } from 'svelte/transition'
  import '../app.css'
  import Menu from '../lib/components/menu/Menu.svelte'
  import Statusbar from '../lib/components/statusbar/statusbar.svelte'
  import { canReachRobot, location, outControllerData } from '$lib/stores'
  import {
    animationState,
    gaitState,
    goalState,
    mode,
    modeState,
    reportFailure
  } from '$lib/robot/state'
  import { notifications } from '$lib/components/toasts/notifications'
  import { dataBroker } from '$lib/transport/databroker'
  import { serial } from '$lib/transport/serial-adapter'
  import { ble } from '$lib/transport/ble-adapter'
  import { websocket } from '$lib/transport/websocket-adapter'
  import { ControllerInputData } from '$lib/platform_shared/message'
  import { connectWebsocket, isLinked, link, reachedRobot } from '$lib/stores/link'
  import {
    currentFeatureFlags,
    featureFlagsState,
    followFeatureFlags
  } from '$lib/stores/featureFlags'
  import { MDNSSettings, WifiStatus } from '$lib/platform_shared/api'
  import { watch } from '$lib/robot/watch'
  import { startSavedRobotSync } from '$lib/robot/saved-robot-sync'
  import { followSticks } from '$lib/stick-link'
  import { watchUpdateOutcome } from '$lib/firmware/watch-update'

  interface Props {
    children?: import('svelte').Snippet
  }

  dataBroker.addTransport(serial)
  dataBroker.addTransport(websocket)
  dataBroker.addTransport(ble)

  let stopSendingSticks: (() => void) | undefined
  let stopWatchingUpdate: (() => void) | undefined
  let stopFollowingState: (() => void)[] = []
  let stopFollowingFeatures: (() => void) | undefined
  let stopSavedRobotSync: (() => void) | undefined

  let { children }: Props = $props()

  onMount(() => {
    stopFollowingFeatures = followFeatureFlags(reachedRobot)
    stopSavedRobotSync = startSavedRobotSync({
      transport: derived(link, $link => $link.transport),
      features: currentFeatureFlags,
      featuresState: featureFlagsState,
      mdns: watch(MDNSSettings),
      wifi: watch(WifiStatus),
      disconnect: () => websocket.disconnect(),
      pageHostname: window.location.hostname
    })
    stopWatchingUpdate = watchUpdateOutcome()
    if (canReachRobot(page.url, $location)) connectWebsocket()

    stopSendingSticks = followSticks({
      commands: outControllerData,
      mode,
      linked: isLinked,
      send: input => dataBroker.emitVolatile(ControllerInputData, input)
    })

    // The robot's state is followed for as long as the app runs, so every page finds it current. A firmware from before
    // the snapshots acks no subscribe, and says so here rather than leaving every watch failed in silence.
    stopFollowingState = [
      ...[gaitState, goalState, animationState].map(state => state.subscribe(() => {})),
      reportFailure(modeState, 'live state', message =>
        notifications.error(`Robot state: ${message}`, 10000)
      )
    ]
  })

  onDestroy(() => {
    stopSendingSticks?.()
    stopWatchingUpdate?.()
    stopFollowingState.forEach(stop => stop())
    stopFollowingFeatures?.()
    stopSavedRobotSync?.()
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
