#pragma once

#include <string>

#include <event_bus.h>
#include <filesystem.h>
#include <template/stateful_service.h>
#include <template/stateful_persistence.h>
#include <platform_shared/message.pb.h>

using ImuCalibration = socket_message_ImuCalibration;

// One shallow slot: the offsets change only when the calibrate button is pressed.
using ImuCalibrationBus = EventBus<ImuCalibration, 2, 2, 1>;

inline ImuCalibration ImuCalibration_defaults() { return socket_message_ImuCalibration_init_zero; }

inline void ImuCalibration_read(const ImuCalibration &settings, ImuCalibration &proto) { proto = settings; }

inline StateUpdateResult ImuCalibration_update(const ImuCalibration &proto, ImuCalibration &settings) {
    // Either a full set of offsets from a calibration, or none: the chip's factory offsets stay.
    const bool none = proto.accel_offsets_count == 0 && proto.gyro_offsets_count == 0;
    const bool full = proto.accel_offsets_count == 3 && proto.gyro_offsets_count == 3;
    if (!none && !full) return StateUpdateResult::ERROR;
    settings = proto;
    return StateUpdateResult::CHANGED;
}

// Persists the MPU6050 offsets the calibration found; the chip forgets them at every power cycle.
class ImuCalibrationService : public StatefulService<ImuCalibration> {
  public:
    ImuCalibrationService()
        : _persistence(ImuCalibration_read, ImuCalibration_update, this, IMU_CALIBRATION_FILE,
                       socket_message_ImuCalibration_fields, socket_message_ImuCalibration_size,
                       ImuCalibration_defaults()) {}

    void begin() {
        _persistence.readFromFS();
        ImuCalibrationBus::publish(state());
    }

  private:
    FSPersistencePB<ImuCalibration> _persistence;
};
