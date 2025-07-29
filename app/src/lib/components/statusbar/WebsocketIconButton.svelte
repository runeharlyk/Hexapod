<script lang="ts">
  import { Connection } from '../icons'
  import { websocket } from '$lib/transport/websocket-adapter'

  let isConnected = websocket.connected
</script>

<button
  class="btn btn-ghost btn-circle btn-sm"
  onclick={() => ($isConnected ? websocket.disconnect() : websocket.connect())}
  title="Disconnect Bluetooth">
  <Connection class="h-6 w-auto {$isConnected ? 'text-success' : 'text-error'}" />
</button>
