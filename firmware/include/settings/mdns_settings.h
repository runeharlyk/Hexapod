#pragma once

#include <template/state_result.h>
#include <platform_shared/api.pb.h>
#include <string>
#include <cstring>

#ifndef FACTORY_MDNS_HOSTNAME
#define FACTORY_MDNS_HOSTNAME "hexapod"
#endif

#ifndef FACTORY_MDNS_INSTANCE
#define FACTORY_MDNS_INSTANCE "Hexapod"
#endif

// Advertised from the running build by MDNSService, never stored: a stored copy goes stale with every
// firmware update.
constexpr const char* MDNS_FIRMWARE_VERSION_KEY = "Firmware Version";

// The factory names of the ESP32 template the firmware grew from, which earlier firmware stored. No
// version of the app could change them, so a stored copy is a leftover rather than a choice.
constexpr const char* MDNS_LEGACY_HOSTNAME = "esp32";
constexpr const char* MDNS_LEGACY_INSTANCE = "ESP32 Device";

using MDNSTxtRecord = api_MDNSTxtRecord;
using MDNSServiceDef = api_MDNSServiceDef;
using MDNSSettings = api_MDNSSettings;

inline MDNSSettings MDNSSettings_defaults() {
    MDNSSettings settings = api_MDNSSettings_init_zero;
    strncpy(settings.hostname, FACTORY_MDNS_HOSTNAME, sizeof(settings.hostname) - 1);
    strncpy(settings.instance, FACTORY_MDNS_INSTANCE, sizeof(settings.instance) - 1);

    settings.services_count = 2;
    strncpy(settings.services[0].service, "_http", sizeof(settings.services[0].service) - 1);
    strncpy(settings.services[0].protocol, "_tcp", sizeof(settings.services[0].protocol) - 1);
    settings.services[0].port = 80;
    settings.services[0].txt_records_count = 0;

    strncpy(settings.services[1].service, "_ws", sizeof(settings.services[1].service) - 1);
    strncpy(settings.services[1].protocol, "_tcp", sizeof(settings.services[1].protocol) - 1);
    settings.services[1].port = 80;
    settings.services[1].txt_records_count = 0;

    return settings;
}

inline void MDNSSettings_read(const MDNSSettings& settings, MDNSSettings& proto) {
    proto = settings;
}

inline bool MDNSTxtRecord_equals(const MDNSTxtRecord& a, const MDNSTxtRecord& b) {
    return strncmp(a.key, b.key, sizeof(a.key)) == 0 && strncmp(a.value, b.value, sizeof(a.value)) == 0;
}

inline bool MDNSServiceDef_equals(const MDNSServiceDef& a, const MDNSServiceDef& b) {
    if (strncmp(a.service, b.service, sizeof(a.service)) != 0) return false;
    if (strncmp(a.protocol, b.protocol, sizeof(a.protocol)) != 0) return false;
    if (a.port != b.port) return false;
    if (a.txt_records_count != b.txt_records_count) return false;
    for (pb_size_t i = 0; i < a.txt_records_count; i++) {
        if (!MDNSTxtRecord_equals(a.txt_records[i], b.txt_records[i])) return false;
    }
    return true;
}

inline bool MDNSSettings_equals(const MDNSSettings& a, const MDNSSettings& b) {
    if (strncmp(a.hostname, b.hostname, sizeof(a.hostname)) != 0) return false;
    if (strncmp(a.instance, b.instance, sizeof(a.instance)) != 0) return false;
    if (a.services_count != b.services_count) return false;
    if (a.global_txt_records_count != b.global_txt_records_count) return false;
    for (pb_size_t i = 0; i < a.services_count; i++) {
        if (!MDNSServiceDef_equals(a.services[i], b.services[i])) return false;
    }
    for (pb_size_t i = 0; i < a.global_txt_records_count; i++) {
        if (!MDNSTxtRecord_equals(a.global_txt_records[i], b.global_txt_records[i])) return false;
    }
    return true;
}

// IDF mdns expects DNS-SD labels with their leading underscore ("_http", "_tcp"). Older settings
// files stored them bare, so a missing underscore is added on load and update.
template <size_t N>
inline void MDNS_ensureUnderscore(char (&label)[N]) {
    const size_t len = strnlen(label, N);
    if (len == 0 || label[0] == '_' || len + 1 >= N) return;
    memmove(label + 1, label, len + 1);
    label[0] = '_';
}

template <size_t N>
inline void MDNS_replaceIf(char (&field)[N], const char* unwanted, const char* replacement) {
    if (strncmp(field, unwanted, N) != 0) return;
    strncpy(field, replacement, N - 1);
    field[N - 1] = '\0';
}

// A hostname a client may store: one DNS label of lowercase letters, digits and hyphens, with no hyphen at either end,
// as the WiFi hostname is kept to (toHostLabel in settings/placeholders.h), and no longer than the wire carries (32
// characters, the field's max_size less its terminator). A client's name is refused rather than changed, so the app
// never shows a name the robot does not answer to.
inline bool MDNSSettings_validHostname(const char* hostname) {
    constexpr size_t maxLength = sizeof(MDNSSettings::hostname) - 1;
    const size_t length = strnlen(hostname, maxLength + 1);
    if (length == 0 || length > maxLength || hostname[0] == '-' || hostname[length - 1] == '-') return false;
    for (size_t i = 0; i < length; i++) {
        const char c = hostname[i];
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-')) return false;
    }
    return true;
}

inline StateUpdateResult MDNSSettings_update(const MDNSSettings& proto, MDNSSettings& settings) {
    MDNSSettings candidate = proto;

    MDNS_replaceIf(candidate.hostname, "", FACTORY_MDNS_HOSTNAME);
    MDNS_replaceIf(candidate.hostname, MDNS_LEGACY_HOSTNAME, FACTORY_MDNS_HOSTNAME);
    MDNS_replaceIf(candidate.instance, MDNS_LEGACY_INSTANCE, FACTORY_MDNS_INSTANCE);

    pb_size_t keptTxt = 0;
    for (pb_size_t i = 0; i < candidate.global_txt_records_count; i++) {
        const MDNSTxtRecord& txt = candidate.global_txt_records[i];
        if (strncmp(txt.key, MDNS_FIRMWARE_VERSION_KEY, sizeof(txt.key)) == 0) continue;
        candidate.global_txt_records[keptTxt++] = txt;
    }
    candidate.global_txt_records_count = keptTxt;

    // A service without a name, protocol or port cannot be registered, so it is dropped rather than
    // failing mdns_service_add on every restart.
    pb_size_t accepted = 0;
    for (pb_size_t i = 0; i < candidate.services_count; i++) {
        MDNSServiceDef& service = candidate.services[i];
        MDNS_ensureUnderscore(service.service);
        MDNS_ensureUnderscore(service.protocol);
        if (strnlen(service.service, sizeof(service.service)) == 0) continue;
        if (strnlen(service.protocol, sizeof(service.protocol)) == 0) continue;
        if (service.port == 0 || service.port > 65535) continue;
        candidate.services[accepted++] = service;
    }
    candidate.services_count = accepted;

    if (MDNSSettings_equals(candidate, settings)) return StateUpdateResult::UNCHANGED;

    settings = candidate;
    return StateUpdateResult::CHANGED;
}
