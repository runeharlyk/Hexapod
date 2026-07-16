#pragma once

#include <esp_now.h>
#include <esp_idf_version.h>
#include <cstdint>

/*
 * Receives broadcast packets from the ESP-NOW handheld controller and
 * republishes them onto the EventBus so the existing MotionService drives the
 * robot with no changes. Input-only: it never sends. Mapping:
 *   - sticks -> CommandMsg (axes / 1000)
 *   - left button  -> cycle motion mode (DEACTIVATED->IDLE->STAND->WALK->...)
 *   - right button -> toggle TRI/BI gait, EXCEPT in STAND where holding it and
 *                     sliding RY sets a latched body-height trim
 *   - both buttons -> emergency stop (DEACTIVATED)
 *
 * Channel: ESP-NOW only hears traffic on the radio's current channel, and the
 * controller broadcasts on a fixed channel (ESPNOW_WIFI_CHANNEL). For reliable
 * reception the robot's radio must be on that channel — easiest is AP mode with
 * the AP channel set to ESPNOW_WIFI_CHANNEL and no STA join (a STA connection
 * forces the radio to the router's channel). begin() logs the active channel
 * and, when not connected as STA, locks it to ESPNOW_WIFI_CHANNEL.
 */
class EspNowAdapter {
  public:
    void begin();

    static bool controllerActive(uint32_t windowMs = 1500);

  private:
// Arduino-ESP32 3.x / IDF 5.x changed the recv-callback signature.
#if ESP_IDF_VERSION_MAJOR >= 5
    static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);
#else
    static void onRecv(const uint8_t* mac, const uint8_t* data, int len);
#endif
};
