#include <wifi_service.h>
#include <communication/webserver.h>
#include <event_bus.h>

static const char *TAG = "WiFiService";

WiFiService::WiFiService()
    : _persistence(WiFiSettings_read, WiFiSettings_update, this, WIFI_SETTINGS_FILE, api_WifiSettings_fields,
                   api_WifiSettings_size, WiFiSettings_defaults()),
      _lastConnectionAttempt(0),
      _stopping(false) {
    addUpdateHandler([&](const std::string &originId) { reconfigureWiFiConnection(); }, false);
}

WiFiService::~WiFiService() {}

void WiFiService::begin() {
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);

    WiFi.onEvent([this](int32_t event, void *data) { this->onStationModeDisconnected(event, data); },
                 WIFI_EVENT_STA_DISCONNECTED);
    WiFi.onEvent([this](int32_t event, void *data) { this->onStationModeStop(event, data); }, WIFI_EVENT_STA_STOP);
    WiFi.onEvent(onStationModeGotIP, IP_EVENT_STA_GOT_IP_IDF);

    _persistence.readFromFS();
    _lastConnectionAttempt = 0;

    if (state().wifi_networks_count >= 1) {
        WiFi.mode(WIFI_MODE_STA);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        uint32_t idx = state().selected_network;
        if (idx >= state().wifi_networks_count) idx = 0;
        configureNetwork(state().wifi_networks[idx]);
    }
}

void WiFiService::reconfigureWiFiConnection() {
    _lastConnectionAttempt = 0;
    if (WiFi.disconnect(true)) _stopping = true;
}

void WiFiService::selectNetwork(uint32_t index) {
    if (index >= state().wifi_networks_count) return;
    updateWithoutPropagation([&](WiFiSettings &settings) {
        settings.selected_network = index;
        return StateUpdateResult::CHANGED;
    });
    _persistence.writeToFS();
    reconfigureWiFiConnection();
}

void WiFiService::loop() { EXECUTE_EVERY_N_MS(1000, manageSTA()); }

int WiFiService::networksProto(api_WifiNetworkScan *buf, size_t maxCount) {
    int numNetworks = WiFi.scanComplete();
    if (numNetworks == -1) return -1;
    if (numNetworks < -1) {
        // -2 = scan refused (radio busy, STA mid-connection); surface it so the client stops polling.
        return (WiFi.scanNetworks(true) == -2) ? -2 : -1;
    }

    size_t count = (static_cast<size_t>(numNetworks) > maxCount) ? maxCount : static_cast<size_t>(numNetworks);
    for (size_t i = 0; i < count; i++) {
        buf[i] = api_WifiNetworkScan_init_zero;
        buf[i].rssi = WiFi.RSSI(i);
        strncpy(buf[i].ssid, WiFi.SSID(i).c_str(), sizeof(buf[i].ssid) - 1);
        strncpy(buf[i].bssid, WiFi.BSSIDstr(i).c_str(), sizeof(buf[i].bssid) - 1);
        buf[i].channel = WiFi.channel(i);
        buf[i].encryption_type = static_cast<uint32_t>(WiFi.encryptionType(i));
    }
    return static_cast<int>(count);
}

void WiFiService::setupMDNS(const char *hostname) {
    mdns_init();
    mdns_hostname_set(state().hostname);
    mdns_instance_name_set(hostname);
    mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
    mdns_service_add(nullptr, "_ws", "_tcp", 80, nullptr, 0);
    mdns_txt_item_t txtData = {"Firmware Version", APP_VERSION};
    mdns_service_txt_set("_http", "_tcp", &txtData, 1);
}

void WiFiService::statusProto(api_WifiStatus &wifiStatus) {
    wl_status_t status = WiFi.status();
    wifiStatus.status = static_cast<uint32_t>(status);

    if (status == WL_CONNECTED) {
        wifiStatus.local_ip = static_cast<uint32_t>(WiFi.localIP());
        strncpy(wifiStatus.mac_address, WiFi.macAddress().c_str(), sizeof(wifiStatus.mac_address) - 1);
        wifiStatus.rssi = WiFi.RSSI();
        strncpy(wifiStatus.ssid, WiFi.SSID().c_str(), sizeof(wifiStatus.ssid) - 1);
        strncpy(wifiStatus.bssid, WiFi.BSSIDstr().c_str(), sizeof(wifiStatus.bssid) - 1);
        wifiStatus.channel = WiFi.channel();
        wifiStatus.subnet_mask = static_cast<uint32_t>(WiFi.subnetMask());
        wifiStatus.gateway_ip = static_cast<uint32_t>(WiFi.gatewayIP());
        IPAddress dnsIP1 = WiFi.dnsIP(0);
        IPAddress dnsIP2 = WiFi.dnsIP(1);
        if (dnsIP1 != IPAddress(0, 0, 0, 0)) {
            wifiStatus.dns_ip_1 = static_cast<uint32_t>(dnsIP1);
        }
        if (dnsIP2 != IPAddress(0, 0, 0, 0)) {
            wifiStatus.dns_ip_2 = static_cast<uint32_t>(dnsIP2);
        }
    }
}

void WiFiService::manageSTA() {
    if (WiFi.isConnected() || state().wifi_networks_count == 0) return;
    if (_stopping) return;

    // Throttle to reconnectDelay; reconfigureWiFiConnection() zeroes the timestamp to reconnect now.
    uint32_t now = esp_timer_get_time() / 1000;
    if (_lastConnectionAttempt != 0 && (now - _lastConnectionAttempt) < reconnectDelay) return;
    _lastConnectionAttempt = now;

    wifi_mode_t mode = WiFi.getMode();
    if (mode == WIFI_MODE_AP) WiFi.mode(WIFI_MODE_APSTA);
    else if (mode == WIFI_MODE_NULL) WiFi.mode(WIFI_MODE_STA);

    uint32_t idx = state().selected_network;
    if (idx >= state().wifi_networks_count) idx = 0;
    ESP_LOGI(TAG, "Connecting to: %s", state().wifi_networks[idx].ssid);
    configureNetwork(state().wifi_networks[idx]);
}

void WiFiService::configureNetwork(WiFiNetwork &network) {
    if (network.static_ip_config) {
        WiFi.config(IPAddress(network.local_ip), IPAddress(network.gateway_ip), IPAddress(network.subnet_mask),
                    IPAddress(network.dns_ip_1), IPAddress(network.dns_ip_2));
    } else {
        WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
    }
    WiFi.setHostname(state().hostname);
    WiFi.begin(network.ssid, network.password);

#if CONFIG_IDF_TARGET_ESP32C3
    WiFi.setTxPower(8);
#endif
}

void WiFiService::publishStatus() {
    api_WifiStatus status = api_WifiStatus_init_zero;
    statusProto(status);
    EventBus<api_WifiStatus>::publish(status);
}

void WiFiService::onStationModeDisconnected(int32_t event, void *event_data) {
    WiFi.disconnect(true);
    wifi_event_sta_disconnected_t *info = static_cast<wifi_event_sta_disconnected_t *>(event_data);
    ESP_LOGI(TAG, "WiFi Disconnected. Reason code=%d", info ? info->reason : 0);
    publishStatus();
}

void WiFiService::onStationModeStop(int32_t event, void *event_data) {
    if (_stopping) {
        _lastConnectionAttempt = 0;
        _stopping = false;
    }
    ESP_LOGI(TAG, "WiFi STA stopped.");
    publishStatus();
}

void WiFiService::onStationModeGotIP(int32_t event, void *event_data) {
    ESP_LOGI(TAG, "WiFi Got IP. localIP=%s, hostName=%s", WiFi.localIP().toString().c_str(), WiFi.getHostname());
    publishStatus();
}
