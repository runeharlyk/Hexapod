#pragma once

#include <feature_flags.h>

#if FT_ENABLED(USE_SERIAL_LINK)

#include <communication/comm_base.hpp>

/*
 * Protobuf transport over the ESP32-S3's native USB Serial/JTAG port.
 *
 * The console stays on UART0 (CONFIG_ESP_CONSOLE_UART_DEFAULT), so this peripheral is free and the
 * two never interleave -- you can watch the log while the link is carrying frames.
 *
 * It exists mainly to make WiFi provisioning workable from the https build. Web Bluetooth is the
 * only transport an https page can otherwise reach the robot with, and it needs pairing; Web Serial
 * is also a secure-context API, so the same page can just open the USB port instead. See
 * docs/connectivity.md for why the https origin cannot use ws:// or the camera.
 *
 * Framing is [uint16 LE length][payload], the same as the BLE adapter, so the app reassembles both
 * with one code path.
 */
class SerialAdapter : public CommAdapterBase {
  public:
    void begin() override;
    bool hasClient() const override { return hostPresent_; }

  private:
    static constexpr size_t RX_BUFFER = 1024;
    static constexpr size_t TX_CHUNK = 256;
    // One host, so the client id is fixed. It still has to be tracked, because CommAdapterBase keys
    // tag subscriptions by client and they must be dropped when the host goes away.
    static constexpr int CLIENT_ID = 0;

    void send(const uint8_t* data, size_t len, int cid = -1) override;

    static void rxTask(void* arg);
    void readLoop();
    void feed(const uint8_t* data, size_t len);

    std::vector<uint8_t> rx_;
    bool hostPresent_ = false;
};

#endif  // FT_ENABLED(USE_SERIAL_LINK)
