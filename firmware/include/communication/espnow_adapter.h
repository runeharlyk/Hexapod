#pragma once

#include <esp_now.h>
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
 * controller broadcasts on a fixed channel (ESPNOW_WIFI_CHANNEL). One radio
 * serves STA, AP and ESP-NOW, so joining a router on any other channel silently
 * stops reception — there is no way to listen on two channels at once. Run the
 * robot AP-only, or put the router on ESPNOW_WIFI_CHANNEL.
 *
 * The channel is re-evaluated on every STA connect/disconnect, not just at
 * startup: at boot the join has not happened yet, so a check there would always
 * see "not connected", pin the channel, and then be silently overridden.
 */
class EspNowAdapter {
  public:
    void begin();

  private:
    // Pins the radio when it is ours to pin, warns when the STA holds it elsewhere.
    static void applyChannel();
    static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);
};
