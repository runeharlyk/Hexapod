#pragma once

#include <string>

#include <event_bus.h>
#include <filesystem.h>
#include <template/stateful_service.h>
#include <template/stateful_persistence.h>
#include <settings/peripherals_settings.h>

// One shallow slot: the I2C pin/frequency config and the compass offset are a rare snapshot, not a stream.
using PeripheralSettingsBus = EventBus<PeripheralSettings, 2, 2, 1>;

// Persists the I2C bus configuration and the compass's hard-iron offset. The bus is opened once at boot
// with whatever is stored here; changing pins on a live bus would invalidate every device handle the
// drivers hold, so new pins take effect on the next restart. The offset applies live: the compass read
// takes it from the bus on every poll.
class PeripheralSettingsService : public StatefulService<PeripheralSettings> {
  public:
    PeripheralSettingsService()
        : _persistence(PeripheralSettings_read, PeripheralSettings_update, this, PERIPHERAL_SETTINGS_FILE,
                       api_PeripheralSettings_fields, api_PeripheralSettings_size, PeripheralSettings_defaults()) {
        addUpdateHandler([&](const std::string &) { PeripheralSettingsBus::publish(state()); }, false);
    }

    void begin() {
        _persistence.readFromFS();
        PeripheralSettingsBus::publish(state());
    }

  private:
    FSPersistencePB<PeripheralSettings> _persistence;
};
