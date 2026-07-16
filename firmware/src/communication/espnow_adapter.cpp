#include <communication/espnow_adapter.h>

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <atomic>

#include <event_bus.h>
#include <message_types.h>
#include <communication/controller_packet.h>
#include <features.h>

static const char* TAG = "espnow";

static std::atomic<uint32_t> s_lastRxMs {0};

static std::atomic<MOTION_STATE> s_mode {MOTION_STATE::DEACTIVATED};
static std::atomic<GaitType> s_gait {GaitType::TRI_GATE};
static float s_height = 0.0f;

static constexpr MOTION_STATE MODE_CYCLE[] = {MOTION_STATE::DEACTIVATED, MOTION_STATE::IDLE,
                                              MOTION_STATE::STAND, MOTION_STATE::WALK};
static constexpr size_t MODE_CYCLE_N = sizeof(MODE_CYCLE) / sizeof(MODE_CYCLE[0]);

#ifndef ESPNOW_WIFI_CHANNEL
#define ESPNOW_WIFI_CHANNEL 1
#endif

static void handlePacket(const uint8_t* data, int len) {
    if (len != (int)sizeof(controller_packet_t)) return;

    controller_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.version != CONTROLLER_PACKET_VERSION) return;

    s_lastRxMs.store(millis(), std::memory_order_relaxed);  // mark controller "active"

    const bool rightHeld = pkt.buttons & BTN_RIGHT;
    const bool inStand = s_mode.load(std::memory_order_relaxed) == MOTION_STATE::STAND;

    constexpr float k = 1.0f / AXIS_FULL_SCALE;
    CommandMsg cmd {};
    cmd.lx = pkt.left_x * k;
    cmd.ly = pkt.left_y * k;
    cmd.rx = pkt.right_x * k;
    cmd.ry = pkt.right_y * k;

    if (inStand && rightHeld) {
        s_height = cmd.ry;
        cmd.ry = 0.0f;
    }
    cmd.h = s_height;
    cmd.s = cmd.s1 = cmd.fd = 0.0f;
    EventBus<CommandMsg>::publish(cmd);

    static uint8_t prev = 0;
    uint8_t rising = pkt.buttons & ~prev;
    prev = pkt.buttons;

    if ((pkt.buttons & (BTN_LEFT | BTN_RIGHT)) == (BTN_LEFT | BTN_RIGHT)) {
        if (rising) {  // both held = emergency stop
            s_mode.store(MOTION_STATE::DEACTIVATED, std::memory_order_relaxed);
            EventBus<ModeMsg>::publish({MOTION_STATE::DEACTIVATED});
        }
        return;
    }

    if (rising & BTN_LEFT) {
        MOTION_STATE cur = s_mode.load(std::memory_order_relaxed);
        size_t idx = 0;
        for (size_t i = 0; i < MODE_CYCLE_N; ++i)
            if (MODE_CYCLE[i] == cur) {
                idx = i;
                break;
            }
        MOTION_STATE next = MODE_CYCLE[(idx + 1) % MODE_CYCLE_N];
        s_mode.store(next, std::memory_order_relaxed);
        EventBus<ModeMsg>::publish({next});
    }

    if ((rising & BTN_RIGHT) && !inStand) {
        GaitType g =
            s_gait.load(std::memory_order_relaxed) == GaitType::TRI_GATE ? GaitType::BI_GATE : GaitType::TRI_GATE;
        s_gait.store(g, std::memory_order_relaxed);
        EventBus<GaitMsg>::publish({g});
    }
}

#if ESP_IDF_VERSION_MAJOR >= 5
void EspNowAdapter::onRecv(const esp_now_recv_info_t*, const uint8_t* data, int len) { handlePacket(data, len); }
#else
void EspNowAdapter::onRecv(const uint8_t*, const uint8_t* data, int len) { handlePacket(data, len); }
#endif

bool EspNowAdapter::controllerActive(uint32_t windowMs) {
    uint32_t last = s_lastRxMs.load(std::memory_order_relaxed);
    return last != 0 && (millis() - last) < windowMs;
}

void EspNowAdapter::begin() {
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed");
        return;
    }
    esp_now_register_recv_cb(onRecv);

    EventBus<ModeMsg>::consume([](const ModeMsg& m) { s_mode.store(m.mode, std::memory_order_relaxed); });
    EventBus<GaitMsg>::consume([](const GaitMsg& g) { s_gait.store(g.gait, std::memory_order_relaxed); });

    uint8_t ch = 0;
    wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);

    if (ch != ESPNOW_WIFI_CHANNEL) {
        if (WiFi.status() != WL_CONNECTED) {
            // AP-only / not joined to a router: safe to pin the radio.
            esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
            ESP_LOGW(TAG, "locked radio to channel %d for ESP-NOW", ESPNOW_WIFI_CHANNEL);
        } else {
            ESP_LOGW(TAG,
                     "STA connected on channel %u but controller uses %d — they MUST match; "
                     "put the robot in AP mode on channel %d or the router on channel %d",
                     ch, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_CHANNEL);
        }
    }
    ESP_LOGI(TAG, "ESP-NOW controller receiver ready on channel %d", ESPNOW_WIFI_CHANNEL);
}
