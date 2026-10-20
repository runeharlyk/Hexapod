<script lang="ts">
  import { onMount } from 'svelte'
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import Card from '$lib/components/layout/Card.svelte'
  import LoadFailure from '$lib/components/layout/LoadFailure.svelte'
  import Loading from '$lib/components/layout/Loading.svelte'
  import {
    AnalyticsData,
    SystemCommandData,
    SystemCommand,
    type StaticSystemInformation
  } from '$lib/platform_shared/message'
  import { dataBroker } from '$lib/transport/databroker'
  import { ask } from '$lib/transport/reply'
  import { convertSeconds } from '$lib/utilities'
  import { useFeatureFlags } from '$lib/stores/featureFlags'
  import {
    Cancel,
    Power,
    FactoryReset,
    Sleep,
    Health,
    CPU,
    SDK,
    CPP,
    Speed,
    Heap,
    Pyramid,
    Flash,
    Folder,
    Temperature,
    Stopwatch
  } from '$lib/components/icons'
  import StatusItem from '$lib/components/StatusItem.svelte'

  const features = useFeatureFlags()

  let staticInfo: StaticSystemInformation | undefined = $state()
  let analytics: AnalyticsData | undefined = $state()
  let statusRequest = $state(getSystemStatus())

  async function getSystemStatus() {
    const res = await ask(dataBroker, { systemInformationRequest: {} }, 'the system status')
    const info = res.systemInformationResponse
    staticInfo = info?.staticSystemInformation
    analytics = info?.analyticsData
  }

  const sendCommand = (command: SystemCommand) => dataBroker.emit(SystemCommandData, { command })

  const postFactoryReset = () => sendCommand(SystemCommand.SYS_RESET)

  const postSleep = () => sendCommand(SystemCommand.SYS_SLEEP)

  const handleSystemData = (data: AnalyticsData) => (analytics = data)

  onMount(() => dataBroker.on(AnalyticsData, handleSystemData))

  const postRestart = () => sendCommand(SystemCommand.SYS_RESTART)

  function confirmRestart() {
    modals.open(ConfirmDialog, {
      title: 'Confirm Restart',
      message: 'Are you sure you want to restart the device?',
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Restart', icon: Power }
      },
      onConfirm: () => {
        modals.close()
        postRestart()
      }
    })
  }

  function confirmReset() {
    modals.open(ConfirmDialog, {
      title: 'Confirm Factory Reset',
      message: 'Are you sure you want to reset the device to its factory defaults?',
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Factory Reset', icon: FactoryReset }
      },
      onConfirm: () => {
        modals.close()
        postFactoryReset()
      }
    })
  }

  function confirmSleep() {
    modals.open(ConfirmDialog, {
      title: 'Confirm Going to Sleep',
      message: 'Are you sure you want to put the device into sleep?',
      labels: {
        cancel: { label: 'Abort', icon: Cancel },
        confirm: { label: 'Sleep', icon: Sleep }
      },
      onConfirm: () => {
        modals.close()
        postSleep()
      }
    })
  }
</script>

<Card title="System status" icon={Health}>
  {#await statusRequest}
    <Loading label="Reading the system status" rows={6} />
  {:then}
    <ul class="list bg-base-100 rounded-box">
      <StatusItem
        icon={CPU}
        title="Chip"
        description={`${staticInfo?.cpuType} (${staticInfo?.espPlatform})`}
      />

      <StatusItem
        icon={SDK}
        title="SDK version"
        description={`ESP-IDF ${staticInfo?.sdkVersion}`}
      />

      <StatusItem
        icon={CPP}
        title="Firmware version"
        description={staticInfo?.firmwareVersion ?? ''}
      />

      <StatusItem
        icon={Speed}
        title="CPU frequency"
        description={`${staticInfo?.cpuFreqMhz} MHz ${
          staticInfo?.cpuCores == 2 ? 'Dual Core' : 'Single Core'
        }`}
      />

      <StatusItem
        icon={Heap}
        title="Heap (free / total)"
        description={`${analytics?.freeHeap} / ${analytics?.totalHeap} bytes (max alloc ${analytics?.maxAllocHeap})`}
      />

      <StatusItem
        icon={Pyramid}
        title="PSRAM (free / size)"
        description={`${analytics?.freePsram} / ${analytics?.psramSize} bytes`}
      />

      <StatusItem
        icon={Flash}
        title="Flash chip size"
        description={`${(staticInfo?.flashChipSize ?? 0) / 1000000} MB`}
      />

      <StatusItem
        icon={Folder}
        title="File system (used / total)"
        description={`${(((analytics?.fsUsed ?? 0) / (analytics?.fsTotal || 1)) * 100).toFixed(
          1
        )} % of ${(analytics?.fsTotal ?? 0) / 1000000} MB used (${
          ((analytics?.fsTotal ?? 0) - (analytics?.fsUsed ?? 0)) / 1000000
        } MB free)`}
      />

      <StatusItem
        icon={Temperature}
        title="Core temperature"
        description={`${(analytics?.coreTemp ?? 0).toFixed(2)} °C`}
      />

      <StatusItem
        icon={Stopwatch}
        title="Uptime"
        description={convertSeconds(analytics?.uptime ?? 0)}
      />

      <StatusItem
        icon={Power}
        title="Reset reason"
        description={staticInfo?.cpuResetReason ?? ''}
      />
    </ul>
  {:catch error}
    <LoadFailure
      message="System status could not be loaded: {error.message}"
      retry={() => (statusRequest = getSystemStatus())}
    />
  {/await}

  {#snippet footer()}
    {#if $features.sleep}
      <button class="btn" onclick={confirmSleep}><Sleep class="size-5" />Sleep</button>
    {/if}
    <button class="btn" onclick={confirmRestart}><Power class="size-5" />Restart</button>
    <button class="btn btn-error btn-soft" onclick={confirmReset}>
      <FactoryReset class="size-5" />Factory reset
    </button>
  {/snippet}
</Card>
