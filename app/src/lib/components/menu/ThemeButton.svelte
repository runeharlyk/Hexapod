<script lang="ts">
  import { MdiWeatherSunny, MdiMoonAndStars } from '../icons'
</script>

<label class="btn btn-ghost btn-sm gap-2" aria-label="Toggle light and dark theme">
  <span class="swap swap-rotate">
    <input type="checkbox" value="light" class="theme-controller" />
    <MdiWeatherSunny class="swap-off h-5 w-5" />
    <MdiMoonAndStars class="swap-on h-5 w-5" />
  </span>
  <span class="text-sm font-normal">Theme</span>
</label>
