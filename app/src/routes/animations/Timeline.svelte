<script lang="ts">
  import { onDestroy } from 'svelte'
  import { get } from 'svelte/store'
  import { Ease, type Overlay } from '$lib/platform_shared/animation'
  import { outControllerData } from '$lib/stores'
  import { clonePose, duration, froundAnimation, stancePose } from '$lib/animation/model'
  import { Player, State } from '$lib/animation/player'
  import {
    LEG_NAMES,
    editor,
    kinematics,
    rideBase,
    shown,
    stanceFor
  } from '$lib/stores/animation-editor'

  const FOOT_CHANNELS = Array.from({ length: 18 }, (_, j) => j)

  const EASES = [Ease.LINEAR, Ease.EASE_IN, Ease.EASE_OUT, Ease.EASE_IN_OUT]
  const BODY_CHANNELS = ['roll', 'pitch', 'yaw', 'x', 'y', 'z']
  const OVERLAY_FIELDS = ['amplitude', 'frequency', 'phase', 'start', 'end'] as const
  // A backgrounded tab resumes with one long frame; cap it so the preview does not skip ahead.
  const MAX_FRAME_S = 0.1
  const MIN_GAP_S = 0.01
  const DRAG_THRESHOLD_PX = 3

  let loop = $state(false)
  let speed = $state(1)
  let stopping = $state(false)

  let player: Player | null = null
  let baseZ = 0
  let frameId = 0
  let last = 0

  let keyframes = $derived($editor.document.keyframes)
  let total = $derived(duration($editor.document))
  let selected = $derived($editor.selected)

  const start = () => {
    const s = get(editor)
    const doc = froundAnimation(s.document)
    const controller = get(outControllerData)
    // The preview holds one base throughout, so it skips the firmware's base blend between the
    // height slider and a fixed ride height during Entry and Exit.
    baseZ = rideBase(doc, controller[4])
    player = new Player(kinematics, stanceFor(controller[7]))
    player.play(doc, s.values, stancePose(), baseZ)
  }

  const step = (now: number) => {
    if (!player) return
    const dt = (Math.min(now - last, MAX_FRAME_S * 1000) / 1000) * speed
    last = now
    const pose = player.update(dt, baseZ)
    if (player.state === State.PLAYING) editor.setScrub(player.t)
    if (player.state === State.IDLE && loop && !stopping) start()
    else if (player.state === State.IDLE) return finish()
    editor.setPlayback(clonePose(pose))
    frameId = requestAnimationFrame(step)
  }

  const finish = () => {
    cancelAnimationFrame(frameId)
    player = null
    stopping = false
    editor.setPlayback(null)
    editor.setScrub(0)
  }

  const play = () => {
    start()
    stopping = false
    last = performance.now()
    editor.setPlayback(clonePose(player!.lastPose))
    frameId = requestAnimationFrame(step)
  }

  // Pausing runs the player's Exit, as a stop on the robot does, and the preview ends in stance.
  const pause = () => {
    stopping = true
    player?.stop(baseZ)
  }

  onDestroy(() => {
    cancelAnimationFrame(frameId)
    if (player) editor.setPlayback(null)
  })

  let axis: HTMLDivElement | undefined = $state()
  let drag: { index: number; scale: number; x: number; time: number } | null = null

  const grab = (e: PointerEvent, index: number) => {
    editor.selectKeyframe(index)
    if (index === 0 || !axis) return
    ;(e.currentTarget as Element).setPointerCapture(e.pointerId)
    // Relative to the press, on a scale frozen for the drag: retiming the last keyframe changes
    // the duration, and a click must not move the keyframe it selects.
    drag = {
      index,
      scale: (total || 1) / axis.getBoundingClientRect().width,
      x: e.clientX,
      time: keyframes[index].time
    }
  }

  const move = (e: PointerEvent) => {
    if (!drag || Math.abs(e.clientX - drag.x) < DRAG_THRESHOLD_PX) return
    const t = drag.time + (e.clientX - drag.x) * drag.scale
    const prev = keyframes[drag.index - 1].time + MIN_GAP_S
    const next = keyframes[drag.index + 1]?.time ?? Infinity
    const clamped = Math.min(Math.max(Math.round(t * 100) / 100, prev), next - MIN_GAP_S)
    if (clamped === keyframes[drag.index].time) return
    editor.setKeyframeTime(drag.index, clamped)
    editor.selectKeyframe(drag.index)
  }

  const release = () => (drag = null)

  const position = (t: number) => (total > 0 ? (t / total) * 100 : 0)

  const channelOf = (o: Overlay) =>
    o.bodyAxis !== undefined ? `b${o.bodyAxis}` : `f${o.footChannel}`

  const setChannel = (i: number, value: string) => {
    const n = Number(value.slice(1))
    editor.updateOverlay(
      i,
      value[0] === 'b' ?
        { bodyAxis: n, footChannel: undefined }
      : { bodyAxis: undefined, footChannel: n }
    )
  }

  const setNumber = (apply: (v: number) => void) => (e: Event) => {
    const v = (e.currentTarget as HTMLInputElement).valueAsNumber
    if (Number.isFinite(v)) apply(v)
  }
</script>

<section class="card bg-base-200">
  <div class="card-body gap-3 p-4">
    <h2 class="card-title text-base">Timeline</h2>

    <div class="px-2">
      <div
        bind:this={axis}
        class="bg-base-300 relative h-8 rounded"
        role="presentation"
        onpointermove={move}
        onpointerup={release}
        onpointercancel={release}
      >
        {#each keyframes as k, i (i)}
          <button
            class="absolute top-1 h-6 w-3 -translate-x-1/2 rounded-sm {i === selected ? 'bg-primary'
            : 'bg-base-content/50'} {i === 0 ? 'cursor-pointer' : 'cursor-ew-resize'}"
            style="left: {position(k.time)}%"
            aria-label="Keyframe {i} at {shown(k.time)} s"
            onpointerdown={e => grab(e, i)}
          ></button>
        {/each}
        <div
          class="bg-error pointer-events-none absolute top-0 h-8 w-0.5"
          style="left: {position($editor.scrub)}%"
        ></div>
      </div>
      <input
        type="range"
        class="range range-xs mt-1"
        aria-label="Scrub"
        min="0"
        max={total}
        step="0.01"
        value={$editor.scrub}
        disabled={$editor.playing || total === 0}
        oninput={e => editor.setScrub(e.currentTarget.valueAsNumber)}
      />
      <div class="flex justify-between text-xs opacity-60">
        <span>0 s</span>
        <span class="font-mono">{$editor.scrub.toFixed(2)} s</span>
        <span>{shown(total)} s</span>
      </div>
    </div>

    <div class="flex flex-wrap items-center gap-2">
      {#if $editor.playing}
        <button class="btn btn-sm" disabled={stopping} onclick={pause}>Pause</button>
      {:else}
        <button class="btn btn-sm btn-primary" onclick={play}>Play</button>
      {/if}
      <label class="flex items-center gap-1 text-sm">
        <input type="checkbox" class="toggle toggle-sm" bind:checked={loop} /> Loop
      </label>
      <label class="flex items-center gap-2 text-sm">
        Speed
        <input
          type="range"
          class="range range-xs w-28"
          min="0.25"
          max="2"
          step="0.05"
          bind:value={speed}
        />
        <span class="font-mono">{speed.toFixed(2)}x</span>
      </label>
    </div>

    <div class="flex flex-wrap items-center gap-2 text-sm">
      <span class="font-medium">Keyframe {selected}</span>
      <label class="flex items-center gap-1">
        time
        <input
          type="number"
          class="input input-sm input-bordered w-24"
          step="0.05"
          min="0"
          value={shown(keyframes[selected].time)}
          disabled={selected === 0}
          onchange={setNumber(v => editor.setKeyframeTime(selected, v))}
        />
      </label>
      <label class="flex items-center gap-1">
        ease
        <select
          class="select select-sm w-auto"
          value={keyframes[selected].ease}
          disabled={selected === 0}
          onchange={e => editor.setEase(selected, Number(e.currentTarget.value) as Ease)}
        >
          {#each EASES as ease (ease)}
            <option value={ease}>{Ease[ease]}</option>
          {/each}
        </select>
      </label>
      <button class="btn btn-sm" onclick={() => editor.addKeyframe(selected)}>Add after</button>
      <button class="btn btn-sm" onclick={() => editor.duplicateKeyframe(selected)}
        >Duplicate</button
      >
      <button
        class="btn btn-sm btn-error"
        disabled={selected === 0}
        onclick={() => editor.deleteKeyframe(selected)}>Delete</button
      >
    </div>

    <div class="flex flex-col gap-2">
      <div class="flex items-center justify-between">
        <h3 class="font-medium">Overlays</h3>
        <button class="btn btn-xs" onclick={editor.addOverlay}>Add overlay</button>
      </div>
      {#each $editor.document.overlays as o, i (i)}
        <div class="flex flex-wrap items-center gap-2 text-xs">
          <select
            class="select select-xs w-auto"
            aria-label="Overlay {i} channel"
            value={channelOf(o)}
            onchange={e => setChannel(i, e.currentTarget.value)}
          >
            <optgroup label="Body">
              {#each BODY_CHANNELS as name, axis (axis)}
                <option value="b{axis}">body {name}</option>
              {/each}
            </optgroup>
            <optgroup label="Foot">
              {#each FOOT_CHANNELS as ch (ch)}
                <option value="f{ch}">{LEG_NAMES[Math.floor(ch / 3)]} foot {'xyz'[ch % 3]}</option>
              {/each}
            </optgroup>
          </select>
          {#each OVERLAY_FIELDS as field (field)}
            <label class="flex items-center gap-1">
              {field}
              <input
                type="number"
                class="input input-xs input-bordered w-16"
                step="0.01"
                value={shown(o[field])}
                onchange={setNumber(v => editor.updateOverlay(i, { [field]: v }))}
              />
            </label>
          {/each}
          <button class="btn btn-xs btn-ghost" onclick={() => editor.removeOverlay(i)}
            >Remove</button
          >
        </div>
      {:else}
        <p class="text-xs opacity-60">No overlays.</p>
      {/each}
    </div>
  </div>
</section>
