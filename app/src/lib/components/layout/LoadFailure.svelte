<script lang="ts">
  // How every page says that a read failed: the sentence, and a retry when reading again can help.
  interface Props {
    message: string
    retry?: () => void
  }

  const { message, retry }: Props = $props()
</script>

<div role="alert" class="alert alert-error alert-soft alert-vertical sm:alert-horizontal">
  <span>{message}</span>
  {#if retry}
    <button type="button" class="btn btn-sm" onclick={retry}>Retry</button>
  {/if}
</div>
