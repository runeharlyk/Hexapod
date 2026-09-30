<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { get } from 'svelte/store'
  import Visualization from '$lib/components/Visualization.svelte'
  import { notifications } from '$lib/components/toasts/notifications'
  import { Animation } from '$lib/platform_shared/animation'
  import { MotionModes } from '$lib/motion'
  import { outControllerData } from '$lib/stores'
  import { isLinked } from '$lib/stores/link'
  import { animationPreview, animationStatus } from '$lib/stores/animation'
  import { clampedJoints, editor, sliderBase } from '$lib/stores/animation-editor'
  import {
    PoseSender,
    inactiveModeWarning,
    playAnimation,
    requestMode,
    stopAnimation
  } from '$lib/control'
  import { dataBroker } from '$lib/transport/databroker'
  import { AnimationState, ModeData } from '$lib/platform_shared/message'
  import {
    clonePose,
    froundAnimation,
    loadAnimationJson,
    serializeAnimation,
    type Vec3
  } from '$lib/animation/model'
  import { downloadAnimation, uploadAnimation } from '$lib/animation/transfer'
  import {
    builtIn,
    draftAnimations,
    loadDraft,
    refreshRobotList,
    robotAnimations,
    saveDraft
  } from '$lib/animation/library'
  import PosePanel from './PosePanel.svelte'
  import Timeline from './Timeline.svelte'
  import AnimationPanel from './AnimationPanel.svelte'

  interface Props {
    onOpen: (a: Animation | null) => void
  }

  let { onOpen }: Props = $props()

  const { validation, frame, pose } = editor

  let sender: PoseSender | null = null
  let unsubscribers: (() => void)[] = []

  onMount(() => {
    unsubscribers = [
      frame.subscribe(f => animationPreview.set(f.preview)),
      isLinked.subscribe(linked => {
        if (linked) refreshRobotList()
        else editor.setShowOnRobot(false)
      }),
      editor.subscribe(s => {
        if (s.showOnRobot && !sender) {
          requestMode(MotionModes.ANIMATE)
          sender = new PoseSender()
          sender.send(clonePose(get(pose)))
        } else if (!s.showOnRobot && sender) {
          sender.cancel()
          sender = null
        }
      }),
      // The player returns its pose by reference, so the sender gets a copy.
      pose.subscribe(p => sender?.send(clonePose(p))),
      // The robot drops a pose sent before it applied ANIMATE, and after a play it rests at stance;
      // both leave it off the preview until the pose changes, so the pose goes out again.
      dataBroker.on(ModeData, data => {
        if (Object.values(MotionModes)[data.mode] === MotionModes.ANIMATE) sender?.resend()
      }),
      animationStatus.subscribe(status => {
        const state = status?.state ?? AnimationState.ANIM_IDLE
        if (state === AnimationState.ANIM_IDLE && lastState !== AnimationState.ANIM_IDLE)
          sender?.resend()
        lastState = state
      })
    ]
  })

  let lastState = AnimationState.ANIM_IDLE

  onDestroy(() => {
    unsubscribers.forEach(unsubscribe => unsubscribe())
    editor.setShowOnRobot(false)
    sender?.cancel()
    animationPreview.set(null)
  })

  // The handles are drawn on the selected keyframe's own feet in body-frame mm and only line up
  // with the rendered robot for a zero body rotation and translation.
  const handles = {
    onDrag: (leg: number, offset: Vec3) => editor.dragFoot(leg, offset),
    onDragEnd: () => {}
  }

  const rounded = () => froundAnimation(get(editor).document)

  const openFile = async (input: HTMLInputElement) => {
    const file = input.files?.[0]
    input.value = ''
    if (!file) return
    const loaded = loadAnimationJson(await file.text())
    if ('error' in loaded) notifications.error(`${file.name}: ${loaded.error}`, 5000)
    else onOpen(loaded.animation)
  }

  const openFrom = async (select: HTMLSelectElement) => {
    const [source, name] = select.value.split(':')
    select.value = ''
    if (source === 'builtin') {
      const a = builtIn.find(b => b.name === name)
      if (a) onOpen(a)
    } else if (source === 'draft') {
      const loaded = loadDraft(name)
      if ('error' in loaded) notifications.error(loaded.error, 5000)
      else onOpen(loaded.animation)
    } else if (source === 'robot') {
      try {
        onOpen(await downloadAnimation(name))
      } catch (e) {
        notifications.error(`Downloading ${name} failed: ${e}`, 5000)
      }
    }
  }

  const saveJson = () => {
    const doc = rounded()
    const blob = new Blob([serializeAnimation(doc)], { type: 'application/json' })
    const link = document.createElement('a')
    link.href = URL.createObjectURL(blob)
    link.download = `${doc.name}.json`
    link.click()
    setTimeout(() => URL.revokeObjectURL(link.href))
    editor.markSaved()
  }

  const saveToDraft = () => {
    const doc = rounded()
    if (!saveDraft(doc)) return notifications.error('This browser refused to store the draft', 5000)
    editor.markSaved()
    notifications.success(`Saved draft ${doc.name}`, 3000)
  }

  const upload = async (): Promise<boolean> => {
    const doc = rounded()
    try {
      const result = await uploadAnimation(doc)
      if (!result.ok) {
        notifications.error(`${doc.name} was refused: ${result.error}`, 6000)
        return false
      }
      const clamped = clampedJoints(result.report.clampedMask)
      if (clamped.length)
        notifications.warning(`${doc.name} uploaded; clamped joints: ${clamped.join(', ')}`, 6000)
      else notifications.success(`${doc.name} uploaded and validated`, 3000)
      refreshRobotList()
      return true
    } catch (e) {
      notifications.error(`Uploading ${doc.name} failed: ${e}`, 6000)
      return false
    }
  }

  const playOnRobot = async () => {
    if (!(await upload())) return
    const warning = inactiveModeWarning()
    if (warning) notifications.warning(warning, 6000)
    playAnimation(rounded().name, get(editor).values)
  }

  // Shown while mirroring a file whose fixed ride height differs from the slider's.
  let mirrorHint = $derived.by(() => {
    const fixed = $editor.document.rideHeight
    const slider = sliderBase($outControllerData[4])
    return $editor.showOnRobot && fixed !== undefined && fixed !== slider ?
        `Mirrors at the slider height (${slider} mm); this file plays at ${fixed} mm`
      : null
  })
</script>

<div class="flex flex-wrap items-center gap-2">
  <button class="btn btn-sm" onclick={() => onOpen(null)}>New</button>
  <label class="btn btn-sm">
    Open file
    <input
      type="file"
      accept=".json,application/json"
      class="hidden"
      onchange={e => openFile(e.currentTarget)}
    />
  </label>
  <select
    class="select select-sm w-auto"
    aria-label="Open from library, drafts or robot"
    onchange={e => openFrom(e.currentTarget)}
  >
    <option value="" selected>Open from...</option>
    <optgroup label="Built-in">
      {#each builtIn as a (a.name)}
        <option value="builtin:{a.name}">{a.name}</option>
      {/each}
    </optgroup>
    <optgroup label="Drafts">
      {#each $draftAnimations as a (a.name)}
        <option value="draft:{a.name}">{a.name}</option>
      {/each}
    </optgroup>
    {#if $isLinked && $robotAnimations}
      <optgroup label="Robot">
        {#each $robotAnimations as entry (entry.name)}
          <option value="robot:{entry.name}">{entry.name}</option>
        {/each}
      </optgroup>
    {/if}
  </select>
  <button
    class="btn btn-sm"
    disabled={$validation !== null}
    title={$validation ?? 'Download the animation as JSON'}
    onclick={saveJson}>Save JSON</button
  >
  <button
    class="btn btn-sm"
    disabled={$validation !== null}
    title={$validation ?? 'Keep a copy in this browser'}
    onclick={saveToDraft}>Save draft</button
  >
  <button class="btn btn-sm" disabled={!$isLinked || $validation !== null} onclick={upload}
    >Upload to robot</button
  >
  <button
    class="btn btn-sm btn-primary"
    disabled={!$isLinked || $validation !== null}
    onclick={playOnRobot}>Play on robot</button
  >
  <button class="btn btn-sm" disabled={!$isLinked} onclick={stopAnimation}>Stop</button>
  <label class="label cursor-pointer gap-2 text-sm">
    <input
      type="checkbox"
      class="toggle toggle-sm"
      disabled={!$isLinked}
      checked={$editor.showOnRobot}
      onchange={e => editor.setShowOnRobot(e.currentTarget.checked)}
    />
    Show on robot
  </label>
  {#if mirrorHint}<span class="text-xs opacity-70">{mirrorHint}</span>{/if}
  {#if $editor.dirty}<span class="text-xs opacity-60">unsaved</span>{/if}
</div>

{#if $editor.error}
  <div role="alert" class="alert alert-warning alert-soft py-2 text-sm">{$editor.error}</div>
{/if}

<div class="grid grid-cols-1 gap-4 lg:grid-cols-2">
  <div class="rounded-box bg-base-200 h-80 overflow-hidden lg:sticky lg:top-4 lg:h-[36rem]">
    <Visualization sky={false} panel={false} previewMode={MotionModes.STAND} {handles} />
  </div>
  <div class="flex flex-col gap-4">
    <PosePanel />
    <Timeline />
    <AnimationPanel />
  </div>
</div>
