#pragma once

#include <esp_http_server.h>
#include <mdns.h>
#include <mdns_scan.h>
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

  private:
    FSPersistencePB<MDNSSettings> _persistence;
    bool _started {false};

    void reconfigureMDNS();
    void startMDNS();
    void stopMDNS();
    void addServices();
};

// The scan's queries on esp-mdns: a PTR query that collects answers on the mDNS task while the service task polls it.
// mDNS starts and stops on the service task too, so a query never outlives the mDNS server it runs on.
class EspMdnsBrowser : public MdnsBrowser {
  public:
    bool begin(const char *service, const char *protocol, uint32_t timeoutMs) override;
    bool ended() override;
    void take(api_MDNSQueryResponse &out) override;

  private:
    mdns_search_once_t *_search {nullptr};
    mdns_result_t *_results {nullptr};
};
