#pragma once

#include <esp_mac.h>
#include <sdkconfig.h>
#include <cctype>
#include <cstdio>
#include <string>

// Expands the factory-settings placeholders (factory_settings.ini): "#{platform}" becomes the IDF
// target (e.g. "esp32s3") and "#{unique_id}" the last three bytes of the STA MAC as hex.
inline std::string substitutePlaceholders(const char *tmpl) {
    std::string out(tmpl);
    auto replaceAll = [&out](const std::string &token, const std::string &value) {
        for (size_t pos = out.find(token); pos != std::string::npos; pos = out.find(token, pos + value.size())) {
            out.replace(pos, token.size(), value);
        }
    };
    replaceAll("#{platform}", CONFIG_IDF_TARGET);
    if (out.find("#{unique_id}") != std::string::npos) {
        uint8_t mac[6] = {};
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        char id[7];
        snprintf(id, sizeof(id), "%02X%02X%02X", mac[3], mac[4], mac[5]);
        replaceAll("#{unique_id}", id);
    }
    return out;
}

// RFC 1123 host label: letters, digits and inner hyphens, at most 63 characters. Anything else in
// the expanded template becomes a hyphen so DHCP and mDNS accept the name.
inline std::string toHostLabel(const char *tmpl) {
    std::string label;
    for (char c : substitutePlaceholders(tmpl)) {
        label += std::isalnum(static_cast<unsigned char>(c)) ? c : '-';
    }
    const size_t first = label.find_first_not_of('-');
    if (first == std::string::npos) return "";
    label = label.substr(first, 63);
    return label.substr(0, label.find_last_not_of('-') + 1);
}
