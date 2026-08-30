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
    xTaskCreate(rxTask, "serial_rx", 4096, this, 4, nullptr);
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
                rx_.clear();
                removeClient(CLIENT_ID);
                ESP_LOGI(TAG, "host disconnected");
            } else {
                ESP_LOGI(TAG, "host connected");
            }
        }

        const int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        if (n > 0) feed(buf, (size_t)n);
    }
}

void SerialAdapter::feed(const uint8_t* data, size_t len) {
    if (rx_.size() + len > 2 * PROTO_BUFFER_SIZE) {
        ESP_LOGW(TAG, "rx buffer overrun, dropping stream to resync");
        rx_.clear();
        return;
    }
    rx_.insert(rx_.end(), data, data + len);

    size_t pos = 0;
    while (rx_.size() - pos >= 2) {
        const size_t msgLen = rx_[pos] | (rx_[pos + 1] << 8);
        if (msgLen == 0 || msgLen > PROTO_BUFFER_SIZE) {
            ESP_LOGW(TAG, "bogus frame length %u, dropping stream to resync", (unsigned)msgLen);
            rx_.clear();
            return;
        }
        if (rx_.size() - pos < 2 + msgLen) break;
        handleIncoming(rx_.data() + pos + 2, msgLen, CLIENT_ID);
        pos += 2 + msgLen;
    }
    if (pos > 0) rx_.erase(rx_.begin(), rx_.begin() + pos);
}

void SerialAdapter::send(const uint8_t* data, size_t len, int cid) {
    (void)cid;
    if (!hostPresent_ || len == 0 || len > PROTO_BUFFER_SIZE) return;

    const uint8_t header[2] = {(uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
    // Short timeout: a host that stops reading must not stall the event-bus worker behind it.
    if (usb_serial_jtag_write_bytes(header, sizeof(header), pdMS_TO_TICKS(20)) != sizeof(header)) return;
    for (size_t off = 0; off < len; off += TX_CHUNK) {
        const size_t take = len - off < TX_CHUNK ? len - off : TX_CHUNK;
        if (usb_serial_jtag_write_bytes(data + off, take, pdMS_TO_TICKS(20)) != (int)take) return;
    }
}

#endif  // FT_ENABLED(USE_SERIAL_LINK)
