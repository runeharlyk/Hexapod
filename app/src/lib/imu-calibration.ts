import { dataBroker } from '$lib/transport/databroker'

export type CalibrationResult = { ok: true } | { ok: false; error: string }

// The answers of the imu_calibrate request (platform_shared/message.proto, ImuCalibrateRequest).
export const describeCalibration = (statusCode: number): CalibrationResult => {
  switch (statusCode) {
    case 200:
      return { ok: true }
    case 409:
      return { ok: false, error: 'Deactivate the robot first, so it lies on its belly.' }
    case 422:
      return {
        ok: false,
        error: 'The robot moved or lies tilted. Put it on a level floor and keep it still.'
      }
    case 503:
      return { ok: false, error: 'The robot has no working IMU.' }
    default:
      return { ok: false, error: `Calibration failed with status ${statusCode}.` }
  }
}

export const calibrateImu = async (): Promise<CalibrationResult> => {
  const res = await dataBroker.request({ imuCalibrate: {} })
  return describeCalibration(res.statusCode)
}
