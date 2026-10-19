import { dataBroker } from '$lib/transport/databroker'
import { ask, RequestFailed, type Reply, type RequestLink } from '$lib/transport/reply'

export type CalibrationResult = { ok: true } | { ok: false; error: string }

// The refusals of the imu_calibrate request the firmware gives no reason for (firmware/src/main.cpp).
const REFUSALS: Record<number, string> = {
  409: 'Deactivate the robot first, so it lies on its belly.',
  422: 'The robot moved or lies tilted. Put it on a level floor and keep it still.',
  503: 'The robot has no working IMU.'
}

const explain = (error: unknown) => {
  if (error instanceof RequestFailed) {
    const { failure } = error
    if (failure.kind === 'refused' && !failure.message && REFUSALS[failure.status])
      return REFUSALS[failure.status]
  }
  return error instanceof Error ? error.message : String(error)
}

export const calibrateImu = async (
  link: RequestLink<{ imuCalibrate: object }, Reply> = dataBroker
): Promise<CalibrationResult> => {
  try {
    await ask(link, { imuCalibrate: {} }, 'IMU calibration')
    return { ok: true }
  } catch (error) {
    return { ok: false, error: explain(error) }
  }
}
