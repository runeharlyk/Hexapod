#pragma once

#include <PsychicHttp.h>
#include <list>
#include <map>
#include <vector>

#include <communication/comm_base.h>

class Websocket : public CommAdapterBase {
  public:
    Websocket();

    PsychicWebSocketHandler *getHandler() { return &_socket; }

  private:
    PsychicWebSocketHandler _socket;

    void onWSOpen(PsychicWebSocketClient *client);
    void onWSClose(PsychicWebSocketClient *client);
    esp_err_t onFrame(PsychicWebSocketRequest *request, httpd_ws_frame *frame);

    void send(const uint8_t *data, size_t len, int cid = -1) override;
};
