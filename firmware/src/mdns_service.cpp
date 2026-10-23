#include <mdns_service.h>
#include <communication/webserver.h>
#include <esp_netif.h>

static const char *TAG = "MDNSService";

MDNSService::MDNSService()
    : _persistence(MDNSSettings_read, MDNSSettings_update, this, MDNS_SETTINGS_FILE, api_MDNSSettings_fields,
                   api_MDNSSettings_size, MDNSSettings_defaults()) {
    addUpdateHandler([&](const std::string &originId) { _reconfigurePending = true; }, false);
}

MDNSService::~MDNSService() {
    if (_started) {
        stopMDNS();
    }
}

void MDNSService::begin() {
    _persistence.readFromFS();
    startMDNS();
}

void MDNSService::applyPendingReconfigure(bool scanRunning) {
    if (!scanRunning && _reconfigurePending.exchange(false)) reconfigureMDNS();
}

void MDNSService::reconfigureMDNS() {
    if (_started) {
        stopMDNS();
    }
    startMDNS();
}

void MDNSService::startMDNS() {
    ESP_LOGV(TAG, "Starting MDNS with hostname: %s", state().hostname);

    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MDNS: %s", esp_err_to_name(err));
        _started = false;
        return;
    }

    err = mdns_hostname_set(state().hostname);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MDNS hostname: %s", esp_err_to_name(err));
        mdns_free();
        _started = false;
        return;
    }

    err = mdns_instance_name_set(state().instance);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set MDNS instance name: %s", esp_err_to_name(err));
    }

    _started = true;
    addServices();

    ESP_LOGI(TAG, "MDNS started successfully with hostname: %s", state().hostname);
}

void MDNSService::stopMDNS() {
    ESP_LOGV(TAG, "Stopping MDNS");
    mdns_free();
    _started = false;
}

void MDNSService::addServices() {
    for (size_t i = 0; i < state().services_count; i++) {
        const auto &service = state().services[i];
        esp_err_t err = mdns_service_add(nullptr, service.service, service.protocol, service.port, nullptr, 0);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to add service %s: %s", service.service, esp_err_to_name(err));
            continue;
        }

        for (size_t j = 0; j < service.txt_records_count; j++) {
            const auto &txt = service.txt_records[j];
            mdns_service_txt_item_set(service.service, service.protocol, txt.key, txt.value);
        }
    }

    for (size_t j = 0; j < state().services_count; j++) {
        const auto &service = state().services[j];
        for (size_t i = 0; i < state().global_txt_records_count; i++) {
            const auto &txt = state().global_txt_records[i];
            mdns_service_txt_item_set(service.service, service.protocol, txt.key, txt.value);
        }
        mdns_service_txt_item_set(service.service, service.protocol, MDNS_FIRMWARE_VERSION_KEY, APP_VERSION);
    }
}

bool EspMdnsBrowser::begin(const char *service, const char *protocol, uint32_t timeoutMs) {
    ESP_LOGI(TAG, "Querying for service: %s, protocol: %s", service, protocol);
    constexpr size_t capacity = sizeof(api_MDNSQueryResponse::services) / sizeof(api_MDNSQueryResult);
    _search = mdns_query_async_new(nullptr, service, protocol, MDNS_TYPE_PTR, timeoutMs, capacity, nullptr);
    if (!_search) ESP_LOGW(TAG, "MDNS query could not start");
    return _search != nullptr;
}

bool EspMdnsBrowser::ended() { return mdns_query_async_get_results(_search, 0, &_results, nullptr); }

void EspMdnsBrowser::take(api_MDNSQueryResponse &resp) {
    constexpr size_t capacity = sizeof(resp.services) / sizeof(resp.services[0]);
    size_t i = 0;
    for (mdns_result_t *r = _results; r && i < capacity; r = r->next, i++) {
        if (r->hostname) {
            strncpy(resp.services[i].name, r->hostname, sizeof(resp.services[i].name) - 1);
        }
        for (mdns_ip_addr_t *a = r->addr; a; a = a->next) {
            if (a->addr.type != ESP_IPADDR_TYPE_V4) continue;
            char ip_str[16];
            esp_ip4addr_ntoa(&a->addr.u_addr.ip4, ip_str, sizeof(ip_str));
            strncpy(resp.services[i].ip, ip_str, sizeof(resp.services[i].ip) - 1);
            break;
        }
        resp.services[i].port = r->port;
    }
    resp.services_count = i;

    ESP_LOGI(TAG, "Found %d services", (int)i);
    mdns_query_results_free(_results);
    mdns_query_async_delete(_search);
    _results = nullptr;
    _search = nullptr;
}
