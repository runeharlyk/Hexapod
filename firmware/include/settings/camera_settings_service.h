#pragma once

#include <string>

#include <feature_flags.h>
#include <filesystem.h>
#include <template/stateful_service.h>
#include <template/stateful_persistence.h>
#include <settings/camera_settings.h>
#if USE_CAMERA
#include <camera_service.h>
#endif

// Persists the camera's settings, which the sensor itself forgets at every reset. Every build stores them, so the topic
// table is one; a build with a camera applies them at boot (camera_service::init) and after every stored change.
class CameraSettingsService : public StatefulService<CameraSettings> {
  public:
    CameraSettingsService()
        : _persistence(CameraSettings_read, CameraSettings_update, this, CAMERA_SETTINGS_FILE,
                       api_CameraSettings_fields, api_CameraSettings_size, CameraSettings_defaults()) {
#if USE_CAMERA
        // An update handler runs after the update has released the service lock, which apply takes again after the
        // sensor's (camera_service::sensorMutex).
        addUpdateHandler([this](const std::string &) { camera_service::apply(*this); }, false);
#endif
    }

    void begin() { _persistence.readFromFS(); }

  private:
    FSPersistencePB<CameraSettings> _persistence;
};
