<script lang="ts">
  import PageShell from '$lib/components/layout/PageShell.svelte'
  import Section from '$lib/components/layout/Section.svelte'
  import { Connection as Usb, Warning } from '$lib/components/icons'
  import { connectSerial, serialAvailable } from '$lib/stores/link'
  import { serial } from '$lib/transport/serial-adapter'
  import Wifi from '../wifi/sta/Wifi.svelte'

  const status = serial.status
</script>

<PageShell>
  <Section title="Set up over USB" icon={Usb}>
    {#if !serialAvailable()}
      <div role="alert" class="alert alert-warning">
        <Warning class="size-6 shrink-0" />
        <span>
          This browser has no Web Serial. Use Chrome or Edge on a desktop, or join the robot's
          <code>Hexapod-XXXXXX</code> network and open <code>http://192.168.4.1</code>.
        </span>
      </div>
    {:else if $status === 'disconnected'}
      <p class="text-sm">
        Connect the robot's native USB port to this computer with a data cable, then choose its
        port. A port this page was given before opens without asking.
      </p>
      <button class="btn btn-primary self-start" onclick={connectSerial}>Connect over USB</button>
    {:else if $status === 'connecting'}
      <p class="flex items-center gap-2 text-sm">
        <span class="loading loading-spinner loading-sm"></span>
        Waiting for the robot to answer; it may still be starting.
      </p>
    {:else}
      <p class="text-sm">
        Connected over USB. Add your home network below; the robot joins it right away.
      </p>
      <button class="btn btn-sm self-start" onclick={() => serial.disconnect()}>Disconnect</button>
    {/if}
  </Section>

  {#if $status === 'connected'}
    <Wifi />
  {/if}
</PageShell>
