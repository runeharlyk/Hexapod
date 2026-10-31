import { StanceEnum } from '$lib/platform_shared/message'
import { GaitType } from '$lib/gait'
import { MotionModes } from '$lib/motion'

/**
 * The feet the classic gaits stand and walk on (platform_shared/message.proto StanceEnum): DEFAULT splays the legs about
 * the body, CRAB points every leg straight out to its side, to walk sideways.
 */
export enum Stance {
  DEFAULT = 'default',
  CRAB = 'crab'
}

const WIRE: Record<Stance, StanceEnum> = {
  [Stance.DEFAULT]: StanceEnum.STANCE_DEFAULT,
  [Stance.CRAB]: StanceEnum.STANCE_CRAB
}

/** The value a stance is sent and reported as. */
export const stanceToWire = (stance: Stance): StanceEnum => WIRE[stance]

/** The stance a reported value is, or undefined for one this app knows no stance for. */
export const stanceFromWire = (value: number): Stance | undefined =>
  Object.values(Stance).find(stance => WIRE[stance] === value)

// AUTO and TUNED walk the default stance alone (firmware message_types.h defaultStanceOnly), and WALK_NN's policy was
// trained on it: the robot refuses each of them in the crab.
const DEFAULT_STANCE_GAITS = [GaitType.AUTO, GaitType.TUNED]
const DEFAULT_STANCE_MODES = [MotionModes.WALK_NN]

/** The gaits the gait selector offers in a stance: those the robot walks it with. */
export const gaitsOffered = (stance: Stance): GaitType[] =>
  Object.values(GaitType).filter(
    gait => stance === Stance.DEFAULT || !DEFAULT_STANCE_GAITS.includes(gait)
  )

/** The modes the mode selector offers in a stance: those the robot takes in it. */
export const modesOffered = (stance: Stance): MotionModes[] =>
  Object.values(MotionModes).filter(
    mode => stance === Stance.DEFAULT || !DEFAULT_STANCE_MODES.includes(mode)
  )
