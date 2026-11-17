import { SystemCommand } from '$lib/platform_shared/message'
import { dataBroker } from '$lib/transport/databroker'
import { ask, type Reply, type RequestLink } from '$lib/transport/reply'

export type SystemCommandResult = { ok: true; message: string } | { ok: false; error: string }

type Command = SystemCommand.SYS_RESTART | SystemCommand.SYS_RESET | SystemCommand.SYS_SLEEP

const WORDING: Record<Command, { feature: string; done: string }> = {
  [SystemCommand.SYS_RESTART]: { feature: 'a restart', done: 'The robot is restarting' },
  [SystemCommand.SYS_RESET]: {
    feature: 'a factory reset',
    done: 'The robot is reset to its factory defaults and restarting'
  },
  [SystemCommand.SYS_SLEEP]: { feature: 'sleep', done: 'The robot is going to sleep' }
}

/**
 * Restarts, resets or puts the robot to sleep. The robot answers before it acts, so the reply says whether it will; the
 * link then drops. A reset whose erase failed is answered as a failure, though the robot restarts all the same
 * (firmware/include/system_command.h).
 */
export const runSystemCommand = async (
  command: Command,
  link: RequestLink<{ systemCommand: { command: SystemCommand } }, Reply> = dataBroker
): Promise<SystemCommandResult> => {
  const { feature, done } = WORDING[command]
  try {
    await ask(link, { systemCommand: { command } }, feature)
    return { ok: true, message: done }
  } catch (error) {
    return { ok: false, error: error instanceof Error ? error.message : String(error) }
  }
}
