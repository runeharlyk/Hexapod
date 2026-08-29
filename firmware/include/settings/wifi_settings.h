#pragma once

#include <wifi/wifi_idf.h>
#include <template/state_result.h>
#include <platform_shared/api.pb.h>
#include <esp_log.h>
#include <cstring>

#ifndef FACTORY_WIFI_SSID
#define FACTORY_WIFI_SSID ""
#endif

#ifndef FACTORY_WIFI_PASSWORD
#define FACTORY_WIFI_PASSWORD ""
#endif

#ifndef FACTORY_WIFI_HOSTNAME
#define FACTORY_WIFI_HOSTNAME "#{platform}-#{unique_id}"
#endif

#ifndef FACTORY_WIFI_RSSI_THRESHOLD
#define FACTORY_WIFI_RSSI_THRESHOLD -80
#endif

using WiFiNetwork = api_WifiNetwork;
using WiFiSettings = api_WifiSettings;

inline WiFiNetwork WiFiNetwork_defaults() {
    WiFiNetwork network = api_WifiNetwork_init_zero;
    strncpy(network.ssid, FACTORY_WIFI_SSID, sizeof(network.ssid) - 1);
    strncpy(network.password, FACTORY_WIFI_PASSWORD, sizeof(network.password) - 1);
    network.static_ip_config = false;
    network.local_ip = 0;
    network.gateway_ip = 0;
    network.subnet_mask = 0;
    network.dns_ip_1 = 0;
    network.dns_ip_2 = 0;
    return network;
}

inline WiFiSettings WiFiSettings_defaults() {
    WiFiSettings settings = api_WifiSettings_init_zero;
    strncpy(settings.hostname, FACTORY_WIFI_HOSTNAME, sizeof(settings.hostname) - 1);
    settings.priority_rssi = true;
    settings.wifi_networks_count = 0;
    settings.selected_network = 0;
    if (strlen(FACTORY_WIFI_SSID) > 0) {
        settings.wifi_networks[0] = WiFiNetwork_defaults();
        settings.wifi_networks_count = 1;
    }
    return settings;
}

inline void WiFiSettings_read(const WiFiSettings &settings, WiFiSettings &proto) { proto = settings; }

// Normalises a candidate network in place; returns false when it is unusable and must be dropped.
inline bool WiFiNetwork_sanitize(WiFiNetwork &network) {
    size_t ssid_length = strnlen(network.ssid, sizeof(network.ssid));
    size_t password_length = strnlen(network.password, sizeof(network.password));
    if (ssid_length < 1 || ssid_length > 32 || password_length > 64) {
        ESP_LOGE("WiFiSettings", "SSID or password length is invalid");
        return false;
    }

    if (network.static_ip_config) {
        if (network.dns_ip_1 == 0 && network.dns_ip_2 != 0) {
            network.dns_ip_1 = network.dns_ip_2;
            network.dns_ip_2 = 0;
        }
        if (network.local_ip == 0 || network.gateway_ip == 0 || network.subnet_mask == 0) {
            ESP_LOGW("WiFiSettings", "Invalid static IP configuration - falling back to DHCP");
            network.static_ip_config = false;
        }
    }

    if (!network.static_ip_config) {
        network.local_ip = 0;
        network.gateway_ip = 0;
        network.subnet_mask = 0;
        network.dns_ip_1 = 0;
        network.dns_ip_2 = 0;
    }
    return true;
}

inline bool WiFiNetwork_equals(const WiFiNetwork &a, const WiFiNetwork &b) {
    return strncmp(a.ssid, b.ssid, sizeof(a.ssid)) == 0 && strncmp(a.password, b.password, sizeof(a.password)) == 0 &&
           a.static_ip_config == b.static_ip_config && a.local_ip == b.local_ip && a.gateway_ip == b.gateway_ip &&
           a.subnet_mask == b.subnet_mask && a.dns_ip_1 == b.dns_ip_1 && a.dns_ip_2 == b.dns_ip_2;
}

inline bool WiFiSettings_equals(const WiFiSettings &a, const WiFiSettings &b) {
    if (strncmp(a.hostname, b.hostname, sizeof(a.hostname)) != 0) return false;
    if (a.priority_rssi != b.priority_rssi) return false;
    if (a.selected_network != b.selected_network) return false;
    if (a.wifi_networks_count != b.wifi_networks_count) return false;
    for (pb_size_t i = 0; i < a.wifi_networks_count; i++) {
        if (!WiFiNetwork_equals(a.wifi_networks[i], b.wifi_networks[i])) return false;
    }
    return true;
}

inline StateUpdateResult WiFiSettings_update(const WiFiSettings &proto, WiFiSettings &settings) {
    WiFiSettings candidate = proto;

    if (strnlen(candidate.hostname, sizeof(candidate.hostname)) == 0) {
        strncpy(candidate.hostname, FACTORY_WIFI_HOSTNAME, sizeof(candidate.hostname) - 1);
        candidate.hostname[sizeof(candidate.hostname) - 1] = '\0';
    }

    pb_size_t accepted = 0;
    for (pb_size_t i = 0; i < candidate.wifi_networks_count; i++) {
        WiFiNetwork network = candidate.wifi_networks[i];
        if (WiFiNetwork_sanitize(network)) candidate.wifi_networks[accepted++] = network;
    }
    // Keeping the usable networks lets a partially corrupt stored config still boot; a payload where
    // nothing survives is a client error and is reported as such.
    if (accepted == 0 && candidate.wifi_networks_count > 0) return StateUpdateResult::ERROR;
    candidate.wifi_networks_count = accepted;

    if (candidate.selected_network >= accepted) candidate.selected_network = 0;

    if (WiFiSettings_equals(candidate, settings)) return StateUpdateResult::UNCHANGED;

    settings = candidate;
    return StateUpdateResult::CHANGED;
}
