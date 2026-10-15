<script lang="ts">
  import type { Servo } from '$lib/platform_shared/api'
  import { config } from '$lib/components/config'
  import {
    JOINT_NAMES,
    LEG_NAMES,
    calibrationLeg,
    kneeAngle,
    legPlanePoints,
    legPoints,
    outwardSide,
    referenceLeg,
    servoJoint,
    type LegAngles
  } from '$lib/calibration/leg-pose'

  interface Props {
    servos: Servo[]
    servoId: number
    pwm: number
  }

  let { servos, servoId, pwm }: Props = $props()

  // The faint leg shows where a slightly higher PWM puts the joint: the way the real joint must move as the slider rises.
  const PWM_STEP = 20

  const selected = $derived(servoJoint(servoId))
  const now = $derived(calibrationLeg(servos, servoId, pwm))
  const ahead = $derived(calibrationLeg(servos, servoId, pwm + PWM_STEP))
  const reference = $derived(referenceLeg(servos, selected.leg))

  const reach =
    config.legRootToJoint1 +
    config.legJoint1ToJoint2 +
    config.legJoint2ToJoint3 +
    config.legJoint3ToTip
  const bodyReach = Math.max(...config.legMountX.map((x, i) => Math.hypot(x, config.legMountY[i])))
  // Wide enough for a leg reaching out level to its knee; only an opened knee can draw past it.
  const topSpan =
    1.1 * (bodyReach + config.legRootToJoint1 + config.legJoint1ToJoint2 + config.legJoint2ToJoint3)
  const topViewBox = `${-topSpan} ${-topSpan} ${2 * topSpan} ${2 * topSpan}`
  const sideReach = config.legJoint2ToJoint3 + config.legJoint3ToTip
  // Both views draw at about the same width on the page, so equal pixel strokes are a fixed share of each view.
  const topStroke = (2 * topSpan) / 120
  const sideStroke = (1.3 * reach) / 120
  // From a knee raised as high as the femur reaches down to a foot straight below it.
  const sideTop = -1.1 * config.legJoint2ToJoint3
  const sideHeight = 1.05 * sideReach - sideTop

  const side = $derived(outwardSide(config, selected.leg))
  const sideViewBox = $derived(
    side > 0 ?
      `${-0.25 * reach} ${sideTop} ${1.3 * reach} ${sideHeight}`
    : `${-1.05 * reach} ${sideTop} ${1.3 * reach} ${sideHeight}`
  )

  const polyline = (points: number[][]) => points.map(([x, y]) => `${x},${y}`).join(' ')

  // Seen from above with the front at the top: body x to the right, body y up the page.
  const top = ([x, y]: number[]) => [x, -y]
  const topLeg = (leg: number, angles: LegAngles) => {
    const { coxaAxis, femurAxis, knee, foot } = legPoints(config, leg, angles)
    return {
      whole: polyline([coxaAxis, femurAxis, knee, foot].map(top)),
      joints: [coxaAxis, femurAxis].map(top)
    }
  }

  // The leg's own plane seen from behind the robot: outwards is to the right for a right leg, to the left for a left one.
  const plane = ([r, z]: number[]) => [side * r, -z]
  const sideLeg = (angles: LegAngles) => {
    const { coxaAxis, femurAxis, knee, foot } = legPlanePoints(config, angles)
    return {
      coxa: polyline([coxaAxis, femurAxis].map(plane)),
      femur: polyline([femurAxis, knee].map(plane)),
      tibia: polyline([knee, foot].map(plane)),
      whole: polyline([coxaAxis, femurAxis, knee, foot].map(plane)),
      joints: [coxaAxis, femurAxis, knee].map(plane)
    }
  }

  const body = polyline(
    [0, 1, 2, 5, 4, 3].map(i => top([config.legMountX[i], config.legMountY[i]]))
  )

  const topNow = $derived(topLeg(selected.leg, now))
  const topAhead = $derived(topLeg(selected.leg, ahead))
  const topReference = $derived(topLeg(selected.leg, reference))
  const sideNow = $derived(sideLeg(now))
  const sideAhead = $derived(sideLeg(ahead))
  const sideReference = $derived(sideLeg(reference))

  const linkClass = (joint: number) =>
    selected.joint === joint ? 'stroke-primary' : 'stroke-current opacity-60'

  const readouts = $derived([
    {
      label: 'Coxa',
      unit: 'from straight out, + is counter-clockwise from above',
      now: now[0],
      reference: reference[0]
    },
    { label: 'Femur', unit: 'above level', now: now[1], reference: reference[1] },
    {
      label: 'Tibia',
      unit: 'knee angle, 90 is square to the femur',
      now: kneeAngle(now[2]),
      reference: kneeAngle(reference[2])
    }
  ])

  const degrees = (angle: number) => `${(Math.abs(angle) < 0.05 ? 0 : angle).toFixed(1)}°`
</script>

<div class="bg-base-200 rounded-box flex w-full max-w-2xl flex-col gap-3 self-center p-4">
  <div class="flex flex-wrap items-baseline justify-between gap-2">
    <h2 class="text-lg font-semibold">Reference pose</h2>
    <span class="text-sm">
      Servo {servoId}:
      <span class="font-semibold">{LEG_NAMES[selected.leg]} {JOINT_NAMES[selected.joint]}</span>
    </span>
  </div>
  <p class="text-xs opacity-70">
    Green dashes: the leg at every servo's centre PWM. Solid: the selected joint at the slider's
    PWM, the rest of the leg drawn at its centre. Faint: {PWM_STEP} PWM higher, the way the joint must
    turn as the slider rises. Line the real joint up with the green leg, then set the centre. A joint
    that turns the other way needs its direction toggled.
  </p>

  <div class="grid grid-cols-1 gap-3 sm:grid-cols-2">
    <figure class="rounded-box p-2 {selected.joint === 0 ? 'bg-base-300' : ''}">
      <svg
        viewBox={topViewBox}
        class="text-base-content mx-auto w-full max-w-72"
        role="img"
        aria-label="{LEG_NAMES[selected.leg]} leg seen from above"
      >
        <polygon
          points={body}
          class="fill-base-300 stroke-current opacity-50"
          stroke-width={topStroke / 2}
        />
        <path
          d="M 0 {-0.75 * bodyReach} l {-bodyReach / 6} {bodyReach / 4} h {bodyReach / 3} z"
          class="fill-current opacity-40"
        />
        {#each LEG_NAMES as name, leg (name)}
          {#if leg !== selected.leg}
            {@const other = topLeg(leg, referenceLeg(servos, leg))}
            <polyline
              points={other.whole}
              fill="none"
              class="stroke-current opacity-20"
              stroke-width={topStroke}
              stroke-linecap="round"
            />
          {/if}
        {/each}
        <polyline
          points={topReference.whole}
          fill="none"
          class="stroke-success"
          stroke-width={topStroke}
          stroke-dasharray="{3 * topStroke} {1.5 * topStroke}"
        />
        <polyline
          points={topAhead.whole}
          fill="none"
          class="stroke-primary opacity-40"
          stroke-width={topStroke}
          stroke-dasharray="{2 * topStroke} {2 * topStroke}"
        />
        <!-- The whole leg turns with the coxa, so its whole length shows the coxa's angle. -->
        <polyline
          points={topNow.whole}
          fill="none"
          class={linkClass(0)}
          stroke-width={2 * topStroke}
          stroke-linecap="round"
          stroke-linejoin="round"
        />
        {#each topNow.joints as [x, y], i (i)}
          <circle
            cx={x}
            cy={y}
            r={1.5 * topStroke}
            class="fill-base-100 stroke-current"
            stroke-width={topStroke / 2}
          />
        {/each}
      </svg>
      <figcaption class="text-center text-xs opacity-70">From above, front at the top</figcaption>
    </figure>

    <figure class="rounded-box p-2 {selected.joint !== 0 ? 'bg-base-300' : ''}">
      <svg
        viewBox={sideViewBox}
        class="text-base-content mx-auto w-full max-w-72"
        role="img"
        aria-label="{LEG_NAMES[selected.leg]} leg in its own plane, seen from behind"
      >
        <rect
          x={side > 0 ? -0.2 * reach : 0}
          y={-0.1 * reach}
          width={0.2 * reach}
          height={0.2 * reach}
          class="fill-base-300 stroke-current opacity-50"
          stroke-width={sideStroke / 2}
        />
        <line
          x1="0"
          y1="0"
          x2={side * reach}
          y2="0"
          class="stroke-current opacity-25"
          stroke-width={sideStroke / 2}
          stroke-dasharray="{sideStroke} {sideStroke}"
        />
        <polyline
          points={sideReference.whole}
          fill="none"
          class="stroke-success"
          stroke-width={sideStroke}
          stroke-dasharray="{3 * sideStroke} {1.5 * sideStroke}"
          stroke-linejoin="round"
        />
        <polyline
          points={sideAhead.whole}
          fill="none"
          class="stroke-primary opacity-40"
          stroke-width={sideStroke}
          stroke-dasharray="{2 * sideStroke} {2 * sideStroke}"
          stroke-linejoin="round"
        />
        <polyline
          points={sideNow.coxa}
          fill="none"
          class="stroke-current opacity-60"
          stroke-width={1.5 * sideStroke}
          stroke-linecap="round"
        />
        <polyline
          points={sideNow.femur}
          fill="none"
          class={linkClass(1)}
          stroke-width={2 * sideStroke}
          stroke-linecap="round"
        />
        <polyline
          points={sideNow.tibia}
          fill="none"
          class={linkClass(2)}
          stroke-width={2 * sideStroke}
          stroke-linecap="round"
        />
        {#each sideNow.joints as [x, y], i (i)}
          <circle
            cx={x}
            cy={y}
            r={1.5 * sideStroke}
            class="fill-base-100 stroke-current"
            stroke-width={sideStroke / 2}
          />
        {/each}
      </svg>
      <figcaption class="text-center text-xs opacity-70">
        Its own plane from behind, body on the {side > 0 ? 'left' : 'right'}
      </figcaption>
    </figure>
  </div>

  <table class="table-xs table">
    <thead>
      <tr>
        <th>Joint</th>
        <th class="text-right">At PWM {pwm}</th>
        <th class="text-right">
          <span class="border-success mr-1 inline-block w-4 border-t-2 border-dashed align-middle"
          ></span>Reference
        </th>
      </tr>
    </thead>
    <tbody>
      {#each readouts as readout, joint (readout.label)}
        <tr class={selected.joint === joint ? 'bg-base-300 font-semibold' : ''}>
          <td>
            {readout.label}
            <span class="block text-[10px] font-normal opacity-60">{readout.unit}</span>
          </td>
          <td class="text-right font-mono {selected.joint === joint ? 'text-primary' : ''}">
            {degrees(readout.now)}
          </td>
          <td class="text-right font-mono">{degrees(readout.reference)}</td>
        </tr>
      {/each}
    </tbody>
  </table>
</div>
