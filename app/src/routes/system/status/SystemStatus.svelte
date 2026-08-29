<script lang="ts">
  import { onMount } from 'svelte'
  import { modals } from 'svelte-modals'
  import ConfirmDialog from '$lib/components/ConfirmDialog.svelte'
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import Spinner from '$lib/components/Spinner.svelte'
  import { slide } from 'svelte/transition'
  import { cubicOut } from 'svelte/easing'
  import {
    AnalyticsData,
    SystemCommandData,
    SystemCommand,
    type StaticSystemInformation
  } from '$lib/platform_shared/message'
  import { dataBroker } from '$lib/transport/databroker'
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
  import type { IconComponent } from '$lib/components/icons'
  import StatusItem from '$lib/components/StatusItem.svelte'
  import ActionButton from './ActionButton.svelte'

  const features = useFeatureFlags()

  let staticInfo: StaticSystemInformation | undefined = $state()
  let analytics: AnalyticsData | undefined = $state()

  async function getSystemStatus() {
    const res = await dataBroker.request({ systemInformationRequest: {} })
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

  interface ActionButtonDef {
    icon: IconComponent
    label: string
    onClick: () => void
    type?: string
    condition?: () => boolean
  }

  const actionButtons: ActionButtonDef[] = [
    {
      icon: Sleep,
      label: 'Sleep',
      onClick: confirmSleep,
      condition: () => Boolean($features.sleep)
    },
    {
      icon: Power,
      label: 'Restart',
      onClick: confirmRestart
    },
    {
      icon: FactoryReset,
      label: 'Factory Reset',
      onClick: confirmReset,
      type: 'secondary'
    }
  ]
</script>

<SettingsCard collapsible={false}>
  {#snippet icon()}
    <Health class="lex-shrink-0 mr-2 h-6 w-6 self-end" />
  {/snippet}
  {#snippet title()}
    <span>System Status</span>
  {/snippet}

  <div class="w-full overflow-x-auto">
    {#await getSystemStatus()}
      <Spinner />
    {:then}
      <div
        class="flex w-full flex-col space-y-1"
        transition:slide|local={{ duration: 300, easing: cubicOut }}
      >
        <StatusItem
          icon={CPU}
          title="Chip"
          description={`${staticInfo?.cpuType} (${staticInfo?.espPlatform})`}
        />

        <StatusItem
          icon={SDK}
          title="SDK Version"
          description={`ESP-IDF ${staticInfo?.sdkVersion}`}
        />

        <StatusItem
          icon={CPP}
          title="Firmware Version"
          description={staticInfo?.firmwareVersion ?? ''}
        />

        <StatusItem
          icon={Speed}
          title="CPU Frequency"
          description={`${staticInfo?.cpuFreqMhz} MHz ${
            staticInfo?.cpuCores == 2 ? 'Dual Core' : 'Single Core'
          }`}
        />

        <StatusItem
          icon={Heap}
          title="Heap (Free / Total)"
          description={`${analytics?.freeHeap} / ${analytics?.totalHeap} bytes (max alloc ${analytics?.maxAllocHeap})`}
        />

        <StatusItem
          icon={Pyramid}
          title="PSRAM (Free / Size)"
          description={`${analytics?.freePsram} / ${analytics?.psramSize} bytes`}
        />

        <StatusItem
          icon={Flash}
          title="Flash Chip Size"
          description={`${(staticInfo?.flashChipSize ?? 0) / 1000000} MB`}
        />

        <StatusItem
          icon={Folder}
          title="File System (Used / Total)"
          description={`${(((analytics?.fsUsed ?? 0) / (analytics?.fsTotal || 1)) * 100).toFixed(
            1
          )} % of ${(analytics?.fsTotal ?? 0) / 1000000} MB used (${
            ((analytics?.fsTotal ?? 0) - (analytics?.fsUsed ?? 0)) / 1000000
          } MB free)`}
        />

        <StatusItem
          icon={Temperature}
          title="Core Temperature"
          description={`${(analytics?.coreTemp ?? 0).toFixed(2)} °C`}
        />

        <StatusItem
          icon={Stopwatch}
          title="Uptime"
          description={convertSeconds(analytics?.uptime ?? 0)}
        />

        <StatusItem
          icon={Power}
          title="Reset Reason"
          description={staticInfo?.cpuResetReason ?? ''}
        />
      </div>
    {/await}
  </div>

  <div class="mt-4 flex flex-wrap justify-end gap-2">
    {#each actionButtons as button}
      {#if button.condition === undefined || button.condition()}
        <ActionButton
          onclick={button.onClick}
          icon={button.icon}
          label={button.label}
          type={button.type || 'primary'}
        />
      {/if}
    {/each}
  </div>
</SettingsCard>
