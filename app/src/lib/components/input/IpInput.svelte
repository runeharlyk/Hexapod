<script lang="ts">
  import { untrack } from 'svelte'
  import { IPV4_PATTERN } from '$lib/components/layout/validators'
  import { ipToString, ipToU32 } from '$lib/proto-api'

  interface Props {
    /**
     * The address as the firmware packs it (proto-api's `ipToU32`). It follows the text once that is an address, and is
     * `NOT_AN_ADDRESS` while it is not: the draft then differs from the robot's address, so the form stays savable and
     * its validity check, which the text fails, names this field instead of sending anything. The marker is one past
     * the largest address, which `ipToString` would show as 0.0.0.0: `isPacked` tells it from every address, so a
     * half-typed address keeps its text through each edit rather than turning into 0.0.0.0.
     */
    value: number
    id?: string
    name?: string
  }

  let { value = $bindable(), id, name }: Props = $props()

  const NOT_AN_ADDRESS = 2 ** 32
  const ADDRESS = new RegExp(`^${IPV4_PATTERN}$`)
  const isPacked = (packed: unknown) =>
    Number.isInteger(packed) && (packed as number) >= 0 && (packed as number) < NOT_AN_ADDRESS

  // The text is the user's while it is not an address; a value changed from outside replaces it.
  let text = $state(untrack(() => ipToString(value)))

  $effect(() => {
    const held = value
    untrack(() => {
      if (isPacked(held) && (!ADDRESS.test(text) || ipToU32(text) !== held)) text = ipToString(held)
    })
  })

  const edited = () => {
    value = ADDRESS.test(text) ? ipToU32(text) : NOT_AN_ADDRESS
  }
</script>

<input
  {id}
  {name}
  class="input validator w-full"
  type="text"
  required
  pattern={IPV4_PATTERN}
  bind:value={text}
  oninput={edited}
/>
