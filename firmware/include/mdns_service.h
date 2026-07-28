#pragma once

#include <esp_http_server.h>
#include <mdns.h>
#include <template/stateful_service.h>
#include <template/stateful_persistence.h>
#include <settings/mdns_settings.h>
#include <utils/timing.h>

class MDNSService : public StatefulService<MDNSSettings> {
  public:
    MDNSService();
    ~MDNSService();

    void begin();

    void statusProto(api_MDNSStatus &status);
    void queryProto(const api_MDNSQueryRequest &req, api_MDNSQueryResponse &resp);

  private:
    FSPersistencePB<MDNSSettings> _persistence;
    bool _started {false};

    void reconfigureMDNS();
    void startMDNS();
    void stopMDNS();
    void addServices();
};
