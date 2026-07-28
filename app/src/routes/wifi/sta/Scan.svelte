<script lang="ts">
  import { focusTrap } from 'svelte-focus-trap'
  import { fly } from 'svelte/transition'
  import { onMount, onDestroy } from 'svelte'
  import RssiIndicator from '$lib/components/statusbar/RSSIIndicator.svelte'
  import type { NetworkItem } from '$lib/types/models'
  import { dataBroker } from '$lib/transport/databroker'
  import { AP, Network, Reload, Cancel, WiFi } from '$lib/components/icons'
  import { modals, exitBeforeEnter } from 'svelte-modals'

  interface Props {
    isOpen: boolean
    storeNetwork: (ssid: string) => void
    connect: (network: NetworkItem) => void
  }

  let { isOpen, storeNetwork, connect }: Props = $props()

  const encryptionType = [
    'Open',
    'WEP',
    'WPA PSK',
    'WPA2 PSK',
    'WPA WPA2 PSK',
    'WPA2 Enterprise',
    'WPA3 PSK',
    'WPA2 WPA3 PSK',
    'WAPI PSK'
  ]

  let listOfNetworks: NetworkItem[] = $state([])

  let scanActive = $state(false)
  let scanError = $state('')

  let pollingId: number

  function stopPolling() {
    if (pollingId) {
      clearInterval(pollingId)
      pollingId = 0
    }
  }

  async function scanNetworks() {
    scanActive = true
    scanError = ''
    listOfNetworks = []
    // One wifiNetworksGet both starts the scan and returns results once ready; poll until they come.
    if (!(await pollingResults())) {
      pollingId = setInterval(() => pollingResults(), 1000)
    }
  }

  async function pollingResults() {
    const res = await dataBroker.request({ wifiNetworksGet: {} }).catch(() => null)
    // 503: the radio can't scan while mid-connection to a saved network — stop, don't poll forever.
    if (res?.statusCode === 503) {
      scanActive = false
      scanError = "Can't scan while the robot is connecting to Wi-Fi. Fix or clear the saved network first."
      stopPolling()
      return 1
    }
    const networks = res?.wifiNetworkList?.networks
    if (!networks || networks.length === 0) return 0
    listOfNetworks = networks.map(
      (n): NetworkItem => ({
        rssi: n.rssi,
        ssid: n.ssid,
        bssid: n.bssid,
        channel: n.channel,
        encryption_type: n.encryptionType
      })
    )
    scanActive = false
    if (listOfNetworks.length) {
      clearInterval(pollingId)
      pollingId = 0
    }
    return listOfNetworks.length
  }

  onMount(() => {
    scanNetworks()
  })

  onDestroy(() => {
    if (pollingId) {
      clearInterval(pollingId)
      pollingId = 0
    }
  })
</script>

{#if isOpen}
  <div
    role="dialog"
    class="pointer-events-none fixed inset-0 z-50 flex items-center justify-center"
    transition:fly={{ y: 50 }}
    use:exitBeforeEnter
    use:focusTrap
  >
    <div
      class="bg-base-100 rounded-box pointer-events-auto flex max-h-full min-w-fit max-w-md flex-col justify-between p-4 shadow-lg"
    >
      <h2 class="text-base-content text-start text-2xl font-bold">Scan Networks</h2>
      <div class="divider my-2"></div>
      <div class="overflow-y-auto">
        {#if scanActive}<div class="bg-base-100 flex flex-col items-center justify-center p-6">
            <AP class="text-secondary h-32 w-32 shrink animate-ping stroke-2" />
            <p class="mt-8 text-2xl">Scanning ...</p>
          </div>
        {:else if scanError}
          <div class="bg-base-100 flex flex-col items-center justify-center p-6 text-center">
            <p class="text-warning text-lg">{scanError}</p>
          </div>
        {:else}
          <ul class="menu">
            {#each listOfNetworks as network, i}
              <li>
                <div class="bg-base-200 rounded-btn my-1 flex items-center gap-2 p-0">
                  <!-- Click the row for advanced setup (static IP etc.); the button connects directly. -->
                  <button
                    class="flex grow items-center space-x-3 px-3 py-2 text-left"
                    onclick={() => storeNetwork(network.ssid)}
                  >
                    <div class="mask mask-hexagon bg-primary h-auto w-10 shrink-0">
                      <Network class="text-primary-content h-auto w-full scale-75" />
                    </div>
                    <div>
                      <div class="font-bold">{network.ssid}</div>
                      <div class="text-sm opacity-75">
                        Security: {encryptionType[network.encryption_type]}, Channel: {network.channel}
                      </div>
                    </div>
                    <div class="grow"></div>
                    <RssiIndicator showDBm={true} rssi={network.rssi} />
                  </button>
                  <button
                    class="btn btn-primary btn-sm mr-2 flex-none"
                    onclick={() => connect(network)}
                  >
                    <WiFi class="h-4 w-4" />Connect
                  </button>
                </div>
              </li>
            {/each}
          </ul>
        {/if}
      </div>
      <div class="divider my-2"></div>
      <div class="flex flex-wrap justify-end gap-2">
        <button
          class="btn btn-primary inline-flex flex-none items-center"
          disabled={scanActive}
          onclick={scanNetworks}
        >
          <Reload class="mr-2 h-5 w-5" /><span>Scan again</span>
        </button>

        <div class="grow"></div>
        <button
          class="btn btn-warning text-warning-content inline-flex flex-none items-center"
          onclick={() => modals.close()}
        >
          <Cancel class="mr-2 h-5 w-5" /><span>Cancel</span>
        </button>
      </div>
    </div>
  </div>
{/if}
