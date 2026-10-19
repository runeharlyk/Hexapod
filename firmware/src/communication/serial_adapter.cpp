#include <communication/serial_adapter.h>

#if FT_ENABLED(USE_SERIAL_LINK)

#include <driver/usb_serial_jtag.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>

static const char* TAG = "SerialLink";

void SerialAdapter::begin() {
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = RX_BUFFER;
    cfg.tx_buffer_size = RX_BUFFER;
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "usb_serial_jtag_driver_install failed; serial link disabled");
        return;
    }
    // The correlation handlers run on this task: the response union alone is ~2.2 KB, plus settings copies
    // and LittleFS writes on the update paths.
    xTaskCreate(rxTask, "serial_rx", 8192, this, 4, nullptr);
    ESP_LOGI(TAG, "protobuf link up on the native USB port");
}

void SerialAdapter::rxTask(void* arg) { static_cast<SerialAdapter*>(arg)->readLoop(); }

void SerialAdapter::readLoop() {
    uint8_t buf[256];
    for (;;) {
        // A host that closes the port leaves its tag subscriptions behind, and every later emit
        // would encode a message for nobody. Track presence and forget the client when it goes.
        const bool present = usb_serial_jtag_is_connected();
        if (present != hostPresent_) {
            hostPresent_ = present;
            if (!present) {
                decoder_.reset();
                removeClient(CLIENT_ID);
                ESP_LOGI(TAG, "host disconnected");
            } else {
                ESP_LOGI(TAG, "host connected");
            }
        }

        const int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n > 0) decoder_.feed(buf, (size_t)n);
    }
}

bool SerialAdapter::send(const uint8_t* data, size_t len, int cid) {
    (void)cid;
    if (!hostPresent_ || len == 0) return false;

    const std::vector<uint8_t> frame = serial_frame::encode(data, len);
    if (frame.empty()) return false;
    // Short timeout: a host that stops reading must not stall the event-bus worker behind it. A frame
    // cut short is dropped by the app's CRC, and the delimiters let it find the next one.
    for (size_t off = 0; off < frame.size(); off += TX_CHUNK) {
        const size_t take = frame.size() - off < TX_CHUNK ? frame.size() - off : TX_CHUNK;
        if (usb_serial_jtag_write_bytes(frame.data() + off, take, pdMS_TO_TICKS(20)) != (int)take) return false;
    }
    return true;
}

#endif  // FT_ENABLED(USE_SERIAL_LINK)
