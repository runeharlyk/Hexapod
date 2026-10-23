<script lang="ts">
  // What the robot holds for a field the user changed that another client changed as well (settings.svelte.ts `notes`).
  // Saving still sends the user's value, so the note is the one place the other client's is seen.
  interface Props {
    /** The form's notes, by field (`fieldOf`). */
    notes: Record<string, unknown>
    field: string
    /** The robot's value as the page shows values; its text by default. */
    format?: (value: unknown) => string
  }

  const { notes, field, format = value => String(value) }: Props = $props()
</script>

{#if field in notes}
  <p class="text-warning mt-1 text-xs break-words">
    changed on the robot: {format(notes[field])}
  </p>
{/if}
