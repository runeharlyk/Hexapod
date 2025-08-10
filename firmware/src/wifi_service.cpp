#include <wifi_service.h>

WiFiService::WiFiService() {}

WiFiService::~WiFiService() {}

void WiFiService::begin() {
    EventBus<WiFiSettingsMsg>::consume([](const WiFiSettingsMsg &s) { cfg = s; });
    EventBus<WiFiSettingsMsg>::peek(cfg);
    WiFi.mode(WIFI_MODE_STA);

    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);

    WiFi.onEvent(std::bind(&WiFiService::onStationModeDisconnected, this, std::placeholders::_1, std::placeholders::_2),
                 WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.onEvent(std::bind(&WiFiService::onStationModeStop, this, std::placeholders::_1, std::placeholders::_2),
                 WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_STOP);

    WiFi.onEvent(onStationModeGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

    reconfigureWiFiConnection();

}

void WiFiService::reconfigureWiFiConnection() {
    _lastConnectionAttempt = 0;

    if (WiFi.disconnect(true)) _stopping = true;
}

void WiFiService::loop() { EXECUTE_EVERY_N_MS(reconnectDelay, manageSTA()); }

void WiFiService::manageSTA() {
    if (WiFi.isConnected()) return;
    if ((WiFi.getMode() & WIFI_STA) == 0) connectToWiFi();
}

void WiFiService::connectToWiFi() {
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
    WiFi.setHostname(cfg.hostname.c_str());

    WiFi.begin(cfg.ssid.c_str(), cfg.password.c_str());

#if CONFIG_IDF_TARGET_ESP32C3
    WiFi.setTxPower(WIFI_POWER_8_5dBm); // https://www.wemos.cc/en/latest/c3/c3_mini_1_0_0.html#about-wifi
#endif
}

void WiFiService::onStationModeDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    WiFi.disconnect(true);
    ESP_LOGI("WiFiStatus", "WiFi Disconnected. Reason code=%d", info.wifi_sta_disconnected.reason);
}

void WiFiService::onStationModeStop(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (_stopping) {
        _lastConnectionAttempt = 0;
        _stopping = false;
    }
    ESP_LOGI("WiFiStatus", "WiFi Connected.");
}

void WiFiService::onStationModeGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
    ESP_LOGI("WiFiStatus", "WiFi Got IP. localIP=%s, hostName=%s", WiFi.localIP().toString().c_str(),
             WiFi.getHostname());
}