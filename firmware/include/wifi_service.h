#pragma once

#include <esp_http_server.h>
#include <wifi/wifi_idf.h>
#include <string>

#include <filesystem.h>
#include <utils/timing.h>
#include <template/stateful_service.h>
#include <template/stateful_persistence.h>
#include <settings/wifi_settings.h>

#define WIFI_EVENT_STA_DISCONNECTED_IDF WIFI_EVENT_STA_DISCONNECTED
#define WIFI_EVENT_STA_STOP_IDF WIFI_EVENT_STA_STOP
#define IP_EVENT_STA_GOT_IP_IDF 1000

class WiFiService : public StatefulService<WiFiSettings> {
  public:
    WiFiService();
    ~WiFiService();

    void begin();
    void loop();

    void selectNetwork(uint32_t index);

    const char *getHostname() { return state().hostname; }

    // Returns the count, -1 while a scan runs (auto-starting one), or -2 if the radio refused.
    static int networksProto(api_WifiNetworkScan *buf, size_t maxCount);
    static void statusProto(api_WifiStatus &status);
    static void publishStatus();

  private:
    void onStationModeDisconnected(int32_t event, void *event_data);
    void onStationModeStop(int32_t event, void *event_data);
    static void onStationModeGotIP(int32_t event, void *event_data);

    FSPersistencePB<WiFiSettings> _persistence;

    void reconfigureWiFiConnection();
    void manageSTA();
    void configureNetwork(WiFiNetwork &network);

    unsigned long _lastConnectionAttempt;
    bool _stopping;
    uint8_t _failedAttempts {0};

    // Each STA retry toggles AP<->APSTA, which churns internal DMA RAM and can starve the BT
    // controller. Back off exponentially from reconnectDelay up to maxReconnectDelay so a
    // persistently-failing network (e.g. wrong password) stops thrashing the radio.
    constexpr static uint16_t reconnectDelay {10000};
    constexpr static uint32_t maxReconnectDelay {60000};
};
