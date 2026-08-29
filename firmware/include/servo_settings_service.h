#pragma once

#include <string>

#include <event_bus.h>
#include <filesystem.h>
#include <template/stateful_service.h>
#include <template/stateful_persistence.h>
#include <settings/servo_settings.h>

class ServoSettingsService : public StatefulService<ServoSettings> {
  public:
    ServoSettingsService()
        : _persistence(ServoSettings_read, ServoSettings_update, this, SERVO_SETTINGS_FILE, api_ServoSettings_fields,
                       api_ServoSettings_size, ServoSettings_defaults()) {
        addUpdateHandler([&](const std::string &) { ServoSettingsBus::publish(state()); }, false);
    }

    void begin() {
        _persistence.readFromFS();
        ServoSettingsBus::publish(state());
    }

  private:
    FSPersistencePB<ServoSettings> _persistence;
};
