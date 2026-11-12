#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_heap_caps.h>
#include <esp_log.h>

#include <gesture.h>
#include <hexapod.h>
#include <ota_service.h>
#include <peripherals/drivers/paj7620.h>
#include <utils/timing.h>

// Polls the PAJ7620U2 every POLL_MS from a task of its own and starts the move each confirmed gesture asks for
// (gesture.h). It polls rather than waits on the chip's INT pin, so the sensor needs only the I2C bus.
namespace gesture_service {

constexpr const char *TAG = "Gesture";
constexpr uint32_t POLL_MS = 50;

inline Hexapod *robot = nullptr;
inline PAJ7620Driver sensor;

inline const char *nameOf(gesture::Hand hand) {
    using gesture::Hand;
    switch (hand) {
        case Hand::UP: return "up";
        case Hand::RIGHT: return "right";
        case Hand::DOWN: return "down";
        case Hand::LEFT: return "left";
        case Hand::CLOCKWISE: return "clockwise";
        case Hand::COUNTER_CLOCKWISE: return "counter-clockwise";
        default: return "other";
    }
}

inline void poll(void *) {
    gesture::Filter filter;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        const std::optional<uint16_t> flags = sensor.readFlags();
        const std::optional<gesture::Hand> hand =
            filter.update(flags ? gesture::decode(*flags) : gesture::Hand::NONE, millis());
        // The robot stays deactivated while a new firmware is written, as the goal requests' handler has it.
        if (!hand || ota_service::inProgress) continue;
        ESP_LOGI(TAG, "Gesture %s: %d", nameOf(*hand), gesture::act(*hand, *robot));
    }
}

/*
 * Control task, once the bus is up: sets the sensor up and starts the poll, or logs that there is no sensor and
 * starts nothing. The poll runs on core 1, as every transfer on the bus does (docs/firmware.md, I2C scan), below the
 * service task's priority, on a PSRAM stack, as it never touches flash.
 */
inline void start(Hexapod &hexapod) {
    if (!sensor.begin()) {
        ESP_LOGI(TAG, "No gesture sensor at 0x%02x; gestures are off", PAJ7620Driver::ADDR);
        return;
    }
    robot = &hexapod;
    if (xTaskCreatePinnedToCoreWithCaps(poll, "gesture", 3072, nullptr, 1, nullptr, 1, MALLOC_CAP_SPIRAM) != pdPASS)
        ESP_LOGE(TAG, "gesture poll not started");
}

} // namespace gesture_service
