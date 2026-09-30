<script lang="ts">
  import { ParamId } from '$lib/platform_shared/animation'
  import { DEFAULT_ENTRY_S, DEFAULT_EXIT_S, PARAM_COUNT } from '$lib/animation/model'
  import { editor, shown } from '$lib/stores/animation-editor'

  const { validation } = editor
  const SPEC_FIELDS = ['min', 'defaultValue', 'max'] as const

  let doc = $derived($editor.document)
  let unused = $derived(
    Array.from({ length: PARAM_COUNT }, (_, id) => id as ParamId).filter(
      id => !doc.params.some(p => p.id === id)
    )
  )
  let adding = $state<ParamId | ''>('')

  const numberFrom = (e: Event, apply: (v: number) => void) => {
    const v = (e.currentTarget as HTMLInputElement).valueAsNumber
    if (Number.isFinite(v)) apply(v)
  }

  const valueOf = (id: ParamId, fallback: number) => $editor.values.get(id) ?? fallback
</script>

<section class="card bg-base-200">
  <div class="card-body gap-3 p-4">
    <h2 class="card-title text-base">Animation</h2>
    <div class="grid grid-cols-1 gap-2 sm:grid-cols-2">
      <label class="flex flex-col gap-1 text-sm">
        name
        <input
          class="input input-sm input-bordered w-full"
          value={doc.name}
          maxlength="32"
          oninput={e => editor.setMeta({ name: e.currentTarget.value })}
        />
      </label>
      <label class="flex flex-col gap-1 text-sm">
        description
        <input
          class="input input-sm input-bordered w-full"
          value={doc.description}
          oninput={e => editor.setMeta({ description: e.currentTarget.value })}
        />
      </label>
    </div>
    <div class="flex flex-wrap items-center gap-4 text-sm">
      <!-- loop and hold_end are exclusive in the schema, so setting one clears the other. -->
      <label class="flex items-center gap-2">
        <input
          type="checkbox"
          class="toggle toggle-sm"
          checked={doc.loop}
          onchange={e =>
            editor.setMeta({
              loop: e.currentTarget.checked,
              holdEnd: e.currentTarget.checked ? false : doc.holdEnd
            })}
        />
        loop
      </label>
      <label class="flex items-center gap-2">
        <input
          type="checkbox"
          class="toggle toggle-sm"
          checked={doc.holdEnd}
          onchange={e =>
            editor.setMeta({
              holdEnd: e.currentTarget.checked,
              loop: e.currentTarget.checked ? false : doc.loop
            })}
        />
        hold at end
      </label>
      <label class="flex items-center gap-1">
        entry
        <input
          type="number"
          class="input input-sm input-bordered w-20"
          min="0"
          step="0.05"
          value={shown(doc.entryTime)}
          title="0 means the default {DEFAULT_ENTRY_S} s"
          onchange={e => numberFrom(e, v => editor.setMeta({ entryTime: v }))}
        />
        s
      </label>
      <label class="flex items-center gap-1">
        exit
        <input
          type="number"
          class="input input-sm input-bordered w-20"
          min="0"
          step="0.05"
          value={shown(doc.exitTime)}
          title="0 means the default {DEFAULT_EXIT_S} s"
          onchange={e => numberFrom(e, v => editor.setMeta({ exitTime: v }))}
        />
        s
      </label>
      <label class="flex items-center gap-2">
        <input
          type="checkbox"
          class="checkbox checkbox-sm"
          checked={doc.rideHeight !== undefined}
          onchange={e => editor.setMeta({ rideHeight: e.currentTarget.checked ? 0 : undefined })}
        />
        fixed ride height
      </label>
      {#if doc.rideHeight !== undefined}
        <label class="flex items-center gap-1">
          <input
            type="number"
            class="input input-sm input-bordered w-20"
            step="1"
            value={shown(doc.rideHeight)}
            onchange={e => numberFrom(e, v => editor.setMeta({ rideHeight: v }))}
          />
          mm
        </label>
      {/if}
    </div>

    <div class="flex flex-col gap-2">
      <h3 class="font-medium">Parameters</h3>
      {#each doc.params as spec, i (spec.id)}
        {@const value = valueOf(spec.id, spec.defaultValue)}
        <div class="flex flex-wrap items-center gap-2 text-xs">
          <span class="w-36 font-medium">{ParamId[spec.id]}</span>
          {#each SPEC_FIELDS as field (field)}
            <label class="flex items-center gap-1">
              {field === 'defaultValue' ? 'default' : field}
              <input
                type="number"
                class="input input-xs input-bordered w-16"
                step="0.05"
                value={shown(spec[field])}
                onchange={e => numberFrom(e, v => editor.setParam(i, { [field]: v }))}
              />
            </label>
          {/each}
          <label class="flex items-center gap-1" title="Preview value">
            <input
              type="range"
              class="range range-xs w-24"
              min={spec.min}
              max={spec.max}
              step={(spec.max - spec.min) / 100 || 0.01}
              {value}
              oninput={e => editor.setValue(spec.id, e.currentTarget.valueAsNumber)}
            />
            <span class="font-mono">{value.toFixed(2)}</span>
          </label>
          <button class="btn btn-xs btn-ghost" onclick={() => editor.removeParam(i)}>Remove</button>
        </div>
      {/each}
      {#if unused.length}
        <div class="flex items-center gap-2">
          <select
            class="select select-xs w-auto"
            aria-label="Parameter to expose"
            bind:value={adding}
          >
            <option value="">Expose a parameter...</option>
            {#each unused as id (id)}
              <option value={id}>{ParamId[id]}</option>
            {/each}
          </select>
          <button
            class="btn btn-xs"
            disabled={adding === ''}
            onclick={() => {
              if (adding !== '') editor.addParam(adding)
              adding = ''
            }}>Add</button
          >
        </div>
      {/if}
    </div>

    {#if $validation}
      <p class="text-error text-sm">Invalid: {$validation}</p>
    {:else}
      <p class="text-success text-sm">Valid</p>
    {/if}
  </div>
</section>
