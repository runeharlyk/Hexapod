<script lang="ts">
  import { BodyAxis, bodyOf, legTarget, type Vec3 } from '$lib/animation/model'
  import { legMask } from '$lib/animation/handles'
  import { JOINT_NAMES, LEG_NAMES, editor, shown } from '$lib/stores/animation-editor'

  const { mask } = editor

  const BODY_SLIDERS = [
    { axis: BodyAxis.ROLL, label: 'roll', min: -0.6, max: 0.6, step: 0.005 },
    { axis: BodyAxis.PITCH, label: 'pitch', min: -0.6, max: 0.6, step: 0.005 },
    { axis: BodyAxis.YAW, label: 'yaw', min: -0.6, max: 0.6, step: 0.005 },
    { axis: BodyAxis.X, label: 'x', min: -60, max: 60, step: 0.5 },
    { axis: BodyAxis.Y, label: 'y', min: -60, max: 60, step: 0.5 },
    { axis: BodyAxis.Z, label: 'z', min: -80, max: 80, step: 0.5 }
  ] as const
  const FOOT_AXES = ['x', 'y', 'z'] as const

  let selected = $derived($editor.selected)
  let keyframe = $derived($editor.document.keyframes[selected])
  let body = $derived(bodyOf(keyframe))
  let legs = $derived(Array.from({ length: 6 }, (_, i) => legTarget(keyframe, i)))

  const isAngle = (axis: BodyAxis) => axis <= BodyAxis.YAW

  const setComponent = (leg: number, axis: number, value: number) => {
    if (!Number.isFinite(value)) return
    const v = [...legs[leg].v] as Vec3
    v[axis] = value
    editor.setLeg(selected, leg, { joints: legs[leg].joints, v })
  }

  const clampedNames = (leg: number) =>
    JOINT_NAMES.filter((_, joint) => (legMask($mask, leg) >>> joint) & 1)
</script>

<section class="card bg-base-200">
  <div class="card-body gap-3 p-4">
    <h2 class="card-title text-base">Pose of keyframe {selected} at {shown(keyframe.time)} s</h2>
    <div class="grid grid-cols-1 gap-x-4 gap-y-1 sm:grid-cols-2">
      {#each BODY_SLIDERS as s (s.axis)}
        <label class="flex flex-col text-xs">
          <span class="flex justify-between">
            <span>{s.label}</span>
            <span class="font-mono">
              {isAngle(s.axis) ?
                `${body[s.axis].toFixed(3)} rad (${((body[s.axis] * 180) / Math.PI).toFixed(
                  1
                )} deg)`
              : `${body[s.axis].toFixed(1)} mm`}
            </span>
          </span>
          <input
            type="range"
            class="range range-xs"
            min={s.min}
            max={s.max}
            step={s.step}
            value={body[s.axis]}
            oninput={e => editor.setBody(selected, s.axis, e.currentTarget.valueAsNumber)}
          />
        </label>
      {/each}
    </div>
    <div class="flex flex-col gap-2">
      {#each legs as leg, i (i)}
        {@const clamped = clampedNames(i)}
        <div class="flex flex-wrap items-center gap-2 text-sm">
          <span class="w-8 font-medium" class:text-error={clamped.length > 0}>{LEG_NAMES[i]}</span>
          <label class="flex items-center gap-1 text-xs">
            foot
            <input
              type="checkbox"
              class="toggle toggle-xs"
              checked={leg.joints}
              onchange={e =>
                editor.setLeg(selected, i, { joints: e.currentTarget.checked, v: leg.v })}
            />
            joints
          </label>
          {#each leg.v as value, axis (axis)}
            <label class="flex items-center gap-1 text-xs">
              {leg.joints ? JOINT_NAMES[axis] : FOOT_AXES[axis]}
              <input
                type="number"
                class="input input-xs input-bordered w-20"
                step={leg.joints ? 1 : 0.5}
                value={shown(value)}
                onchange={e => setComponent(i, axis, e.currentTarget.valueAsNumber)}
              />
            </label>
          {/each}
          <span class="text-xs opacity-60">{leg.joints ? 'deg' : 'mm'}</span>
          {#if clamped.length}
            <span class="text-error text-xs">{clamped.join(', ')} clamped</span>
          {/if}
        </div>
      {/each}
    </div>
  </div>
</section>
