#pragma once

#include <PsychicHttp.h>
#include <IPAddress.h>
#include <WiFi.h>

#include <filesystem.h>
#include <utils/timing.h>
#include <event_bus.h>
#include <message_types.h>

class WiFiService {
  private:
    void onStationModeDisconnected(WiFiEvent_t event, WiFiEventInfo_t info);
    void onStationModeStop(WiFiEvent_t event, WiFiEventInfo_t info);
    static void onStationModeGotIP(WiFiEvent_t event, WiFiEventInfo_t info);

    void reconfigureWiFiConnection();
    void manageSTA();
    void connectToWiFi();

    unsigned long _lastConnectionAttempt;
    bool _stopping;

    constexpr static uint16_t reconnectDelay {10000};
    inline static WiFiSettingsMsg cfg;

  public:
    WiFiService();
    ~WiFiService();

    void begin();
    void loop();

    static esp_err_t handleScan(PsychicRequest *request);
    static esp_err_t getNetworks(PsychicRequest *request);
    static esp_err_t getNetworkStatus(PsychicRequest *request);
};