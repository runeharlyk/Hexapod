#pragma once

#include <PsychicHttp.h>
#include <list>
#include <map>
#include <vector>

#include <communication/comm_base.h>

class EventSource : public CommAdapterBase {
  public:
    EventSource(PsychicHttpServer &server, const char *route = "/api/sse");

    void begin() override;

  private:
    PsychicEventSource _eventSource;
    PsychicHttpServer &_server;
    const char *_route;

    void send(const uint8_t *data, size_t len, int cid = -1) override;
    void onSSEOpen(PsychicEventSourceClient *client);
    void onSSEClose(PsychicEventSourceClient *client);
};
