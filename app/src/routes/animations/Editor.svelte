<script lang="ts">
  import { onDestroy, onMount } from 'svelte'
  import { get } from 'svelte/store'
  import Visualization from '$lib/components/Visualization.svelte'
  import { notifications } from '$lib/components/toasts/notifications'
  import { Animation } from '$lib/platform_shared/animation'
  import { MotionModes } from '$lib/motion'
  import { outControllerData } from '$lib/stores'
  import { isLinked } from '$lib/stores/link'
  import { animationPreview } from '$lib/stores/animation'
  import { clampedJoints, editor, stanceFor } from '$lib/stores/animation-editor'
  import { PoseSender, playAnimation, requestMode, stopAnimation } from '$lib/control'
  import { DEFAULT_FEET } from '$lib/animation/evaluator'
  import {
    clonePose,
    froundAnimation,
    legTarget,
    loadAnimationJson,
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
      pose.subscribe(p => sender?.send(clonePose(p)))
    ]
  })

  onDestroy(() => {
    unsubscribers.forEach(unsubscribe => unsubscribe())
    editor.setShowOnRobot(false)
    sender?.cancel()
    animationPreview.set(null)
  })

  // The handles report body-frame mm from DEFAULT_FEET and are only exact for a zero body rotation
  // and translation; with a keyframe body offset the stored foot differs from the dragged position
  // by that offset. The offset is rebased onto the editor's feet-distance stance.
  const handles = {
    onDrag: (leg: number, offset: Vec3) => {
      const s = get(editor)
      if (s.playing || legTarget(s.document.keyframes[s.selected], leg).joints) return
      const stance = stanceFor(get(outControllerData)[7])
      editor.setLeg(s.selected, leg, {
        joints: false,
        v: [
          offset[0] + DEFAULT_FEET[leg][0] - stance[leg][0],
          offset[1] + DEFAULT_FEET[leg][1] - stance[leg][1],
          offset[2]
        ]
      })
    },
    onDragEnd: () => {}
  }

  const document = () => froundAnimation(get(editor).document)

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
    const doc = document()
    const blob = new Blob([JSON.stringify(Animation.toJSON(doc), null, 2)], {
      type: 'application/json'
    })
    const link = window.document.createElement('a')
    link.href = URL.createObjectURL(blob)
    link.download = `${doc.name}.json`
    link.click()
    URL.revokeObjectURL(link.href)
    editor.markSaved()
  }

  const saveToDraft = () => {
    const doc = document()
    if (!saveDraft(doc)) return notifications.error('This browser refused to store the draft', 5000)
    editor.markSaved()
    notifications.success(`Saved draft ${doc.name}`, 3000)
  }

  const upload = async (): Promise<boolean> => {
    const doc = document()
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
    if (await upload()) playAnimation(document().name, get(editor).values)
  }
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
  <button class="btn btn-sm" onclick={saveJson}>Save JSON</button>
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
