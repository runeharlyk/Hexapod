#include <communication/espnow_adapter.h>

#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <atomic>
#include <cstring>

#include <wifi/wifi_idf.h>

#include <event_bus.h>
#include <message_types.h>
#include <communication/controller_packet.h>
#include <features.h>

static const char* TAG = "espnow";

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
    if (cmd.sanitize()) EventBus<CommandMsg>::publish(cmd);

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

void EspNowAdapter::onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    // One line the first time a frame lands, so "no reaction" can be told apart from "not heard".
    // Silence after boot means the radio left the controller's channel -- see applyChannel().
    static bool announced = false;
    if (!announced && info && info->src_addr) {
        announced = true;
        const uint8_t* m = info->src_addr;
        ESP_LOGI(TAG, "controller heard: %02x:%02x:%02x:%02x:%02x:%02x len=%d ver=%u", m[0], m[1], m[2], m[3],
                 m[4], m[5], len, len > 0 ? data[0] : 0);
    }
    handlePacket(data, len);
}

void EspNowAdapter::begin() {
    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed");
        return;
    }
    esp_now_register_recv_cb(onRecv);

    // Broadcast peer with channel 0 == "whatever the radio is on", so the beacon follows the radio
    // through every WiFi join without needing to be re-registered.
    esp_now_peer_info_t peer {};
    memset(peer.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) ESP_LOGW(TAG, "broadcast peer not added; beacon disabled");

    const esp_timer_create_args_t beaconArgs {
        .callback = &EspNowAdapter::sendBeacon, .arg = nullptr, .dispatch_method = ESP_TIMER_TASK,
        .name = "espnow_beacon", .skip_unhandled_events = true};
    esp_timer_handle_t beacon = nullptr;
    if (esp_timer_create(&beaconArgs, &beacon) == ESP_OK) {
        esp_timer_start_periodic(beacon, HEXAPOD_BEACON_PERIOD_MS * 1000);
    }

    EventBus<ModeMsg>::consume([](const ModeMsg& m) { s_mode.store(m.mode, std::memory_order_relaxed); });
    EventBus<GaitMsg>::consume([](const GaitMsg& g) { s_gait.store(g.gait, std::memory_order_relaxed); });

    // A STA join retunes the radio, so re-check on every connect and disconnect rather than only
    // here -- at boot the join is still seconds away and this would always take the "free to pin"
    // branch, then be overridden without a word.
    WiFi.onEvent([](int32_t, void *) { applyChannel(); }, WIFI_EVENT_STA_CONNECTED);
    WiFi.onEvent([](int32_t, void *) { applyChannel(); }, WIFI_EVENT_STA_DISCONNECTED);
    applyChannel();
}

// Announces the current channel so a hunting controller can lock onto it. The controller cannot be
// told to move -- it would have to already be on our channel to hear that -- so it hunts instead.
void EspNowAdapter::sendBeacon(void*) {
    uint8_t ch = 0;
    wifi_second_chan_t sc;
    if (esp_wifi_get_channel(&ch, &sc) != ESP_OK) return;

    static const uint8_t broadcast[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    hexapod_beacon_t b {HEXAPOD_BEACON_MAGIC, HEXAPOD_BEACON_VERSION, ch, 0};
    esp_now_send(broadcast, reinterpret_cast<const uint8_t*>(&b), sizeof(b));
}

void EspNowAdapter::applyChannel() {
    uint8_t ch = 0;
    wifi_second_chan_t sc;
    esp_wifi_get_channel(&ch, &sc);

    if (ch == ESPNOW_WIFI_CHANNEL) {
        ESP_LOGI(TAG, "listening for the controller on channel %u", ch);
        return;
    }

    // Ask the driver whether the STA is associated rather than WiFi.isConnected(): our own status
    // only flips on GOT_IP, which is a second or more after the join has already retuned the radio.
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        // One radio serves STA, AP and ESP-NOW; the router owns the channel while joined.
        ESP_LOGW(TAG,
                 "DEAF to the controller: STA holds channel %u, controller broadcasts on %d. "
                 "Run AP-only or move the router to channel %d.",
                 ch, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_CHANNEL);
        return;
    }

    esp_err_t err = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        // Usually a scan or join in flight; the next STA event re-runs this.
        ESP_LOGI(TAG, "channel %u busy, deferring the move to %d (%s)", ch, ESPNOW_WIFI_CHANNEL,
                 esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "moved radio from channel %u to %d for the controller", ch, ESPNOW_WIFI_CHANNEL);
}
