#pragma once

#include <template/state_result.h>
#include <platform_shared/api.pb.h>
#include <string>
#include <cstring>

#ifndef FACTORY_MDNS_HOSTNAME
#define FACTORY_MDNS_HOSTNAME "hexapod"
#endif

#ifndef FACTORY_MDNS_INSTANCE
#define FACTORY_MDNS_INSTANCE "ESP32 Device"
#endif

using MDNSTxtRecord = api_MDNSTxtRecord;
using MDNSServiceDef = api_MDNSServiceDef;
using MDNSSettings = api_MDNSSettings;
using MDNSStatus = api_MDNSStatus;

inline MDNSSettings MDNSSettings_defaults() {
    MDNSSettings settings = api_MDNSSettings_init_zero;
    strncpy(settings.hostname, FACTORY_MDNS_HOSTNAME, sizeof(settings.hostname) - 1);
    strncpy(settings.instance, FACTORY_MDNS_INSTANCE, sizeof(settings.instance) - 1);

    settings.services_count = 2;
    strncpy(settings.services[0].service, "http", sizeof(settings.services[0].service) - 1);
    strncpy(settings.services[0].protocol, "tcp", sizeof(settings.services[0].protocol) - 1);
    settings.services[0].port = 80;
    settings.services[0].txt_records_count = 0;

    strncpy(settings.services[1].service, "ws", sizeof(settings.services[1].service) - 1);
    strncpy(settings.services[1].protocol, "tcp", sizeof(settings.services[1].protocol) - 1);
    settings.services[1].port = 80;
    settings.services[1].txt_records_count = 0;

    settings.global_txt_records_count = 1;
    strncpy(settings.global_txt_records[0].key, "Firmware Version", sizeof(settings.global_txt_records[0].key) - 1);
    strncpy(settings.global_txt_records[0].value, APP_VERSION, sizeof(settings.global_txt_records[0].value) - 1);

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

inline StateUpdateResult MDNSSettings_update(const MDNSSettings& proto, MDNSSettings& settings) {
    MDNSSettings candidate = proto;

    if (strnlen(candidate.hostname, sizeof(candidate.hostname)) == 0) {
        strncpy(candidate.hostname, FACTORY_MDNS_HOSTNAME, sizeof(candidate.hostname) - 1);
        candidate.hostname[sizeof(candidate.hostname) - 1] = '\0';
    }

    // A service without a name, protocol or port cannot be registered, so it is dropped rather than
    // failing mdns_service_add on every restart.
    pb_size_t accepted = 0;
    for (pb_size_t i = 0; i < candidate.services_count; i++) {
        const MDNSServiceDef& service = candidate.services[i];
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
