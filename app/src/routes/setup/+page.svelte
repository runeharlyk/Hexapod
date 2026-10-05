<script lang="ts">
  import SettingsCard from '$lib/components/SettingsCard.svelte'
  import { Warning } from '$lib/components/icons'
  import { connectSerial, serialAvailable } from '$lib/stores/link'
  import { serial } from '$lib/transport/serial-adapter'
  import Wifi from '../wifi/sta/Wifi.svelte'

  const status = serial.status
</script>

<div class="mx-0 my-1 flex flex-col space-y-4 sm:mx-8 sm:my-8">
  <SettingsCard collapsible={false}>
    {#snippet title()}
      <span>Set up over USB</span>
    {/snippet}
    {#if !serialAvailable()}
      <div role="alert" class="alert alert-warning">
        <Warning class="h-6 w-6 shrink-0" />
        <span>
          This browser has no Web Serial. Use Chrome or Edge on a desktop, or join the robot's
          <code>Hexapod-XXXXXX</code> network and open <code>http://192.168.4.1</code>.
        </span>
      </div>
    {:else if $status === 'disconnected'}
      <p>
        Connect the robot's native USB port to this computer with a data cable, then choose its
        port. A port this page was given before opens without asking.
      </p>
      <button class="btn btn-primary self-start" onclick={connectSerial}>Connect over USB</button>
    {:else if $status === 'connecting'}
      <p class="flex items-center gap-2">
        <span class="loading loading-spinner loading-sm"></span>
        Waiting for the robot to answer; it may still be starting.
      </p>
    {:else}
      <p>Connected over USB. Add your home network below; the robot joins it right away.</p>
      <button class="btn btn-sm self-start" onclick={() => serial.disconnect()}>Disconnect</button>
    {/if}
  </SettingsCard>

  {#if $status === 'connected'}
    <Wifi />
  {/if}
</div>
