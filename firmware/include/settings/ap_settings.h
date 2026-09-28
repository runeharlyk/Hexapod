#pragma once

#include <wifi/wifi_idf.h>
#include <wifi/dns_server.h>
#include <template/state_result.h>
#include <settings/placeholders.h>
#include <platform_shared/api.pb.h>
#include <cstring>

#ifndef FACTORY_AP_PROVISION_MODE
#define FACTORY_AP_PROVISION_MODE api_APProvisionMode_AP_MODE_DISCONNECTED
#endif

#ifndef FACTORY_AP_SSID
#define FACTORY_AP_SSID "Hexapod-#{unique_id}"
#endif

#ifndef FACTORY_AP_PASSWORD
#define FACTORY_AP_PASSWORD "hexapod1"
#endif

#ifndef FACTORY_AP_LOCAL_IP
#define FACTORY_AP_LOCAL_IP "192.168.4.1"
#endif

#ifndef FACTORY_AP_GATEWAY_IP
#define FACTORY_AP_GATEWAY_IP "192.168.4.1"
#endif

#ifndef FACTORY_AP_SUBNET_MASK
#define FACTORY_AP_SUBNET_MASK "255.255.255.0"
#endif

#ifndef FACTORY_AP_CHANNEL
#define FACTORY_AP_CHANNEL 1
#endif

#ifndef FACTORY_AP_SSID_HIDDEN
#define FACTORY_AP_SSID_HIDDEN false
#endif

#ifndef FACTORY_AP_MAX_CLIENTS
#define FACTORY_AP_MAX_CLIENTS 4
#endif

#define AP_MODE_ALWAYS api_APProvisionMode_AP_MODE_ALWAYS
#define AP_MODE_DISCONNECTED api_APProvisionMode_AP_MODE_DISCONNECTED
#define AP_MODE_NEVER api_APProvisionMode_AP_MODE_NEVER

#define MANAGE_NETWORK_DELAY 10000
#define DNS_PORT 53

using APNetworkStatus = api_APNetworkStatus;
#define ACTIVE api_APNetworkStatus_AP_ACTIVE
#define INACTIVE api_APNetworkStatus_AP_INACTIVE
#define LINGERING api_APNetworkStatus_AP_LINGERING

inline uint32_t parseIPv4(const char *str) {
    IPAddress ip;
    ip.fromString(str);
    return (uint32_t)ip;
}

using APSettings = api_APSettings;

inline APSettings APSettings_defaults() {
    APSettings settings = {};
    settings.provision_mode = FACTORY_AP_PROVISION_MODE;
    strncpy(settings.ssid, substitutePlaceholders(FACTORY_AP_SSID).c_str(), sizeof(settings.ssid) - 1);
    strncpy(settings.password, FACTORY_AP_PASSWORD, sizeof(settings.password) - 1);
    settings.channel = FACTORY_AP_CHANNEL;
    settings.ssid_hidden = FACTORY_AP_SSID_HIDDEN;
    settings.max_clients = FACTORY_AP_MAX_CLIENTS;
    settings.local_ip = parseIPv4(FACTORY_AP_LOCAL_IP);
    settings.gateway_ip = parseIPv4(FACTORY_AP_GATEWAY_IP);
    settings.subnet_mask = parseIPv4(FACTORY_AP_SUBNET_MASK);
    return settings;
}

inline void APSettings_read(const APSettings &settings, APSettings &proto) { proto = settings; }

inline bool APSettings_equals(const APSettings &a, const APSettings &b) {
    return a.provision_mode == b.provision_mode && strncmp(a.ssid, b.ssid, sizeof(a.ssid)) == 0 &&
           strncmp(a.password, b.password, sizeof(a.password)) == 0 && a.channel == b.channel &&
           a.ssid_hidden == b.ssid_hidden && a.max_clients == b.max_clients && a.local_ip == b.local_ip &&
           a.gateway_ip == b.gateway_ip && a.subnet_mask == b.subnet_mask;
}

inline StateUpdateResult APSettings_update(const APSettings &proto, APSettings &settings) {
    APSettings candidate = proto;
    // Reads redact the password, so an empty one from a client means "keep the stored password".
    if (strnlen(candidate.password, sizeof(candidate.password)) == 0) {
        memcpy(candidate.password, settings.password, sizeof(candidate.password));
    }

    switch (candidate.provision_mode) {
        case AP_MODE_ALWAYS:
        case AP_MODE_DISCONNECTED:
        case AP_MODE_NEVER: break;
        default: candidate.provision_mode = AP_MODE_DISCONNECTED;
    }

    size_t ssid_length = strnlen(candidate.ssid, sizeof(candidate.ssid));
    if (ssid_length < 1 || ssid_length > 32) {
        ESP_LOGE("APSettings", "AP SSID length is invalid");
        return StateUpdateResult::ERROR;
    }

    // softAP() picks WPA2 as soon as the password is non-empty, and WPA2 needs 8..63 characters.
    size_t password_length = strnlen(candidate.password, sizeof(candidate.password));
    if (password_length > 0 && (password_length < 8 || password_length > 63)) {
        ESP_LOGE("APSettings", "AP password length is invalid");
        return StateUpdateResult::ERROR;
    }

    if (candidate.channel < 1 || candidate.channel > 13) candidate.channel = FACTORY_AP_CHANNEL;
    if (candidate.max_clients < 1 || candidate.max_clients > ESP_WIFI_MAX_CONN_NUM) {
        candidate.max_clients = FACTORY_AP_MAX_CLIENTS;
    }

    // An unreachable AP is worse than a non-default one, so an incomplete address falls back wholesale.
    if (candidate.local_ip == 0 || candidate.gateway_ip == 0 || candidate.subnet_mask == 0) {
        ESP_LOGW("APSettings", "Incomplete AP IP configuration - using factory addresses");
        candidate.local_ip = parseIPv4(FACTORY_AP_LOCAL_IP);
        candidate.gateway_ip = parseIPv4(FACTORY_AP_GATEWAY_IP);
        candidate.subnet_mask = parseIPv4(FACTORY_AP_SUBNET_MASK);
    }

    if (APSettings_equals(candidate, settings)) return StateUpdateResult::UNCHANGED;

    settings = candidate;
    return StateUpdateResult::CHANGED;
}
