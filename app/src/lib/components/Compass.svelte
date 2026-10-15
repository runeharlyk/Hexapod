<script lang="ts">
  import { cardinalDirection } from '$lib/compass'

  interface Props {
    heading?: number
  }

  let { heading = 0 }: Props = $props()

  const ticks = Array.from({ length: 12 }, (_, i) => i * 30)
  const at = (deg: number, r: number) => ({
    x: 100 + r * Math.sin((deg * Math.PI) / 180),
    y: 100 - r * Math.cos((deg * Math.PI) / 180)
  })
</script>

<div class="flex flex-col items-center">
  <svg viewBox="0 0 200 200" class="h-40 w-40" role="img" aria-label="Compass heading">
    <circle
      cx="100"
      cy="100"
      r="90"
      fill="none"
      stroke="currentColor"
      stroke-width="2"
      class="opacity-30"
    />
    {#each ticks as tick (tick)}
      <line
        x1={at(tick, 85).x}
        y1={at(tick, 85).y}
        x2={at(tick, 77).x}
        y2={at(tick, 77).y}
        stroke="currentColor"
        stroke-width={tick % 90 === 0 ? 2 : 1}
        class="opacity-50"
      />
    {/each}
    {#each ['N', 'E', 'S', 'W'] as label, i (label)}
      <text
        x={at(i * 90, 62).x}
        y={at(i * 90, 62).y + 5}
        text-anchor="middle"
        class="fill-current text-sm font-bold">{label}</text
      >
    {/each}
    <g transform="rotate({heading}, 100, 100)">
      <polygon points="100,25 93,100 100,90 107,100" class="fill-error" />
      <polygon points="100,175 93,100 100,110 107,100" class="fill-base-300" />
    </g>
    <circle cx="100" cy="100" r="6" class="fill-base-content" />
  </svg>
  <div class="font-mono text-xl font-bold">{heading.toFixed(1)} deg</div>
  <div class="text-sm opacity-70">{cardinalDirection(heading)}</div>
</div>
